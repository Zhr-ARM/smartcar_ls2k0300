#include "driver/vision/vision_pipeline.h"

#include "driver/vision/vision_config.h"
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_transport.h"

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace
{
// 当前主链固定处理分辨率与 full 分辨率。
static constexpr int kProcWidth = VISION_DOWNSAMPLED_WIDTH;
static constexpr int kProcHeight = VISION_DOWNSAMPLED_HEIGHT;
static constexpr int kFullWidth = UVC_WIDTH;
static constexpr int kFullHeight = UVC_HEIGHT;

enum target_board_state_enum
{
    TARGET_BOARD_NONE = 0,
    TARGET_BOARD_WEAPON = 1,
    TARGET_BOARD_SUPPLY = 2,
    TARGET_BOARD_VEHICLE = 3
};

static target_board_state_enum g_target_board_state = TARGET_BOARD_NONE;
static uint32 g_last_processed_infer_result_seq = 0;
static float g_center_target_offset_restore_px = g_vision_runtime_config.ipm_center_target_offset_from_left_px;
static int g_target_board_candidate_class_id = -1;
static int g_target_board_confirm_count = 0;
static int g_target_board_no_red_count = 0;
static float g_target_board_applied_offset_px = g_vision_runtime_config.ipm_center_target_offset_from_left_px;

static void reset_target_board_candidate_state()
{
    g_target_board_candidate_class_id = -1;
    g_target_board_confirm_count = 0;
}

// TinyClassifier 6 类顺序：ambulance, armored_car, bomb, gun, medicine, telescope。
static target_board_state_enum resolve_target_board_state_for_class_id(int class_id)
{
    if (class_id == 2 || class_id == 3)
    {
        return TARGET_BOARD_WEAPON;
    }
    if (class_id == 4 || class_id == 5)
    {
        return TARGET_BOARD_SUPPLY;
    }
    if (class_id == 0 || class_id == 1)
    {
        return TARGET_BOARD_VEHICLE;
    }
    return TARGET_BOARD_NONE;
}

static const char *target_board_state_name(target_board_state_enum state)
{
    switch (state)
    {
        case TARGET_BOARD_WEAPON:
            return "weapon";
        case TARGET_BOARD_SUPPLY:
            return "supply";
        case TARGET_BOARD_VEHICLE:
            return "vehicle";
        case TARGET_BOARD_NONE:
        default:
            return "none";
    }
}

static float clamp_center_target_offset(float offset_px)
{
    const float track_width_px = std::max(0.0f, g_vision_runtime_config.ipm_track_width_px);
    return std::clamp(offset_px, 0.0f, track_width_px);
}

static float resolve_target_board_offset(target_board_state_enum state, float restore_offset_px)
{
    const float delta_px = std::max(0.0f, g_vision_runtime_config.infer_avoid_offset_delta_px);
    switch (state)
    {
        case TARGET_BOARD_WEAPON:
            return clamp_center_target_offset(restore_offset_px - delta_px);
        case TARGET_BOARD_SUPPLY:
            return clamp_center_target_offset(restore_offset_px + delta_px);
        case TARGET_BOARD_VEHICLE:
        case TARGET_BOARD_NONE:
        default:
            return clamp_center_target_offset(restore_offset_px);
    }
}

static void restore_target_board_state_if_needed()
{
    if (g_target_board_state == TARGET_BOARD_NONE)
    {
        return;
    }
    vision_image_processor_set_ipm_center_target_offset_from_left_px(g_center_target_offset_restore_px);
    g_target_board_state = TARGET_BOARD_NONE;
    g_target_board_no_red_count = 0;
    g_target_board_applied_offset_px = g_center_target_offset_restore_px;
    reset_target_board_candidate_state();
}

static void reset_dynamic_center_target_offset_state(bool restore_default)
{
    if (restore_default)
    {
        restore_target_board_state_if_needed();
    }
    g_last_processed_infer_result_seq = 0;
    g_target_board_no_red_count = 0;
    reset_target_board_candidate_state();
    if (g_target_board_state == TARGET_BOARD_NONE)
    {
        g_center_target_offset_restore_px = vision_image_processor_ipm_center_target_offset_from_left_px();
        g_target_board_applied_offset_px = g_center_target_offset_restore_px;
    }
}

static void enter_target_board_state(target_board_state_enum state)
{
    g_center_target_offset_restore_px = vision_image_processor_ipm_center_target_offset_from_left_px();
    g_target_board_state = state;
    g_target_board_no_red_count = 0;
    g_target_board_applied_offset_px = resolve_target_board_offset(state, g_center_target_offset_restore_px);
    vision_image_processor_set_ipm_center_target_offset_from_left_px(g_target_board_applied_offset_px);
    reset_target_board_candidate_state();
}

static void enforce_active_target_board_offset()
{
    if (g_target_board_state == TARGET_BOARD_NONE)
    {
        return;
    }
    // 配置热更新/旧初始化路径可能会把动态偏移刷回 TOML 默认值；
    // 目标板状态未退出前，每帧处理开始前重新下发一次绕行偏移。
    vision_image_processor_set_ipm_center_target_offset_from_left_px(g_target_board_applied_offset_px);
}

static void update_active_target_board_state(bool red_found)
{
    if (red_found)
    {
        g_target_board_no_red_count = 0;
        return;
    }

    ++g_target_board_no_red_count;
    if (g_target_board_no_red_count >= std::max(1, g_vision_runtime_config.infer_target_exit_no_red_count))
    {
        restore_target_board_state_if_needed();
    }
}

static void update_idle_target_board_candidate(const vision_infer_async_result_t &result)
{
    const float threshold = std::clamp(g_vision_runtime_config.infer_target_confidence_threshold, 0.0f, 1.0f);
    const bool valid_candidate = result.found &&
                                 result.ncnn_infer_valid &&
                                 result.ncnn_top_score >= threshold &&
                                 resolve_target_board_state_for_class_id(result.ncnn_top_class_id) != TARGET_BOARD_NONE;
    if (!valid_candidate)
    {
        reset_target_board_candidate_state();
        return;
    }

    if (result.ncnn_top_class_id == g_target_board_candidate_class_id)
    {
        ++g_target_board_confirm_count;
    }
    else
    {
        g_target_board_candidate_class_id = result.ncnn_top_class_id;
        g_target_board_confirm_count = 1;
    }

    if (g_target_board_confirm_count >= std::max(1, g_vision_runtime_config.infer_target_confirm_count))
    {
        enter_target_board_state(resolve_target_board_state_for_class_id(result.ncnn_top_class_id));
    }
}

static void update_dynamic_center_target_offset_from_infer_result(const vision_infer_async_result_t &result)
{
    if (result.result_seq == 0 || result.result_seq == g_last_processed_infer_result_seq)
    {
        return;
    }
    g_last_processed_infer_result_seq = result.result_seq;

    if (g_target_board_state != TARGET_BOARD_NONE)
    {
        update_active_target_board_state(result.found);
        return;
    }

    update_idle_target_board_candidate(result);
}

static cv::Rect map_full_rect_to_proc_rect(int x, int y, int w, int h)
{
    if (w <= 0 || h <= 0 || kFullWidth <= 0 || kFullHeight <= 0)
    {
        return cv::Rect();
    }
    const int x0 = static_cast<int>(std::floor(static_cast<double>(x) * kProcWidth / kFullWidth));
    const int y0 = static_cast<int>(std::floor(static_cast<double>(y) * kProcHeight / kFullHeight));
    const int x1 = static_cast<int>(std::ceil(static_cast<double>(x + w) * kProcWidth / kFullWidth));
    const int y1 = static_cast<int>(std::ceil(static_cast<double>(y + h) * kProcHeight / kFullHeight));
    return cv::Rect(x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0)) &
           cv::Rect(0, 0, kProcWidth, kProcHeight);
}

// 作用：清空 image_processor 内缓存的推理结果。
// 调用关系：推理关闭、无结果、cleanup 时调用。
static void clear_infer_result_in_image_processor()
{
    vision_image_processor_set_last_red_detect_us(0);
    vision_image_processor_set_red_rect(false, 0, 0, 0, 0, 0, 0, 0);
    vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);
}

// 作用：将异步推理结果写回 image_processor（供发送/状态展示复用）。
// 参数：result 为最近一帧异步推理结果。
// 如何修改：如需新增绘制元素，可在此处扩展叠加逻辑。
static void apply_infer_result_to_image(vision_infer_async_result_t *result)
{
    if (result == nullptr)
    {
        clear_infer_result_in_image_processor();
        return;
    }

    vision_image_processor_set_last_red_detect_us(result->red_detect_us);
    if (!result->found)
    {
        vision_image_processor_set_red_rect(false, 0, 0, 0, 0, 0, 0, 0);
        vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);
        return;
    }

    vision_image_processor_set_red_rect(true,
                                        result->red_x,
                                        result->red_y,
                                        result->red_w,
                                        result->red_h,
                                        result->red_cx,
                                        result->red_cy,
                                        result->red_area);

    if (!result->ncnn_roi_valid)
    {
        vision_image_processor_set_ncnn_roi(false, 0, 0, 0, 0);
        return;
    }

    vision_image_processor_set_ncnn_roi(true,
                                        result->ncnn_roi_x,
                                        result->ncnn_roi_y,
                                        result->ncnn_roi_w,
                                        result->ncnn_roi_h);

    const uint8 *bgr_proc_data = vision_image_processor_bgr_image();
    const uint8 *gray_data = vision_image_processor_gray_image();
    const cv::Rect red_proc_rect = map_full_rect_to_proc_rect(result->red_x,
                                                              result->red_y,
                                                              result->red_w,
                                                              result->red_h);
    const cv::Rect roi_proc_rect = map_full_rect_to_proc_rect(result->ncnn_roi_x,
                                                              result->ncnn_roi_y,
                                                              result->ncnn_roi_w,
                                                              result->ncnn_roi_h);
    if (bgr_proc_data != nullptr)
    {
        cv::Mat proc_frame(kProcHeight, kProcWidth, CV_8UC3, const_cast<uint8 *>(bgr_proc_data));
        if (red_proc_rect.width > 0 && red_proc_rect.height > 0)
        {
            cv::rectangle(proc_frame, red_proc_rect, cv::Scalar(0, 0, 255), 1, cv::LINE_8);
        }
        if (roi_proc_rect.width > 0 && roi_proc_rect.height > 0)
        {
            cv::rectangle(proc_frame, roi_proc_rect, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
        }
    }
    if (gray_data != nullptr)
    {
        cv::Mat gray(kProcHeight, kProcWidth, CV_8UC1, const_cast<uint8 *>(gray_data));
        if (red_proc_rect.width > 0 && red_proc_rect.height > 0)
        {
            cv::rectangle(gray, red_proc_rect, cv::Scalar(200), 1, cv::LINE_8);
        }
        if (roi_proc_rect.width > 0 && roi_proc_rect.height > 0)
        {
            cv::rectangle(gray, roi_proc_rect, cv::Scalar(255), 1, cv::LINE_8);
        }
    }
}

} // namespace

bool vision_pipeline_init(const char *camera_path, LQ_NCNN *ncnn, bool ncnn_enabled)
{
    // 初始化顺序：处理模块 -> 推理模块 -> 发送模块。
    if (!vision_image_processor_init(camera_path))
    {
        return false;
    }

    if (!vision_infer_async_init(ncnn, ncnn_enabled))
    {
        vision_image_processor_cleanup();
        return false;
    }

    vision_transport_init();
    clear_infer_result_in_image_processor();
    reset_dynamic_center_target_offset_state(false);
    return true;
}

void vision_pipeline_cleanup()
{
    vision_infer_async_cleanup();
    reset_dynamic_center_target_offset_state(true);
    clear_infer_result_in_image_processor();
    vision_image_processor_cleanup();
}

bool vision_pipeline_process_step()
{
    enforce_active_target_board_offset();

    // 1) 固定先跑采图+巡线/逆透视。
    if (!vision_image_processor_process_step())
    {
        return false;
    }

    const uint8 *bgr_proc_data = vision_image_processor_bgr_image();
    const uint8 *bgr_full_data = vision_image_processor_bgr_full_image();
    // 2) 推理关闭或处理图无效时，清空推理相关状态但不影响巡线主链。
    if (!vision_infer_async_enabled() || bgr_proc_data == nullptr)
    {
        reset_dynamic_center_target_offset_state(true);
        clear_infer_result_in_image_processor();
        return true;
    }

    // 3) 提交当前帧到异步推理线程（不会阻塞主线程）。
    vision_infer_async_submit_frame(bgr_proc_data,
                                    kProcWidth,
                                    kProcHeight,
                                    bgr_full_data,
                                    kFullWidth,
                                    kFullHeight);

    // 4) 取“最近完成”的异步结果并回填到可视化状态。
    vision_infer_async_result_t latest{};
    if (!vision_infer_async_fetch_latest(&latest))
    {
        clear_infer_result_in_image_processor();
        return true;
    }

    update_dynamic_center_target_offset_from_infer_result(latest);
    apply_infer_result_to_image(&latest);
    return true;
}

void vision_pipeline_send_step()
{
    vision_transport_send_step();
}

bool vision_pipeline_step()
{
    if (!vision_pipeline_process_step())
    {
        return false;
    }
    vision_pipeline_send_step();
    return true;
}

const uint8 *vision_pipeline_bgr_image()
{
    return vision_image_processor_bgr_image();
}

void vision_pipeline_set_send_mode(vision_send_mode_enum mode)
{
    vision_transport_set_send_mode(mode);
}

vision_send_mode_enum vision_pipeline_get_send_mode()
{
    return vision_transport_get_send_mode();
}

void vision_pipeline_set_send_max_fps(uint32 max_fps)
{
    vision_transport_set_send_max_fps(max_fps);
}

uint32 vision_pipeline_get_send_max_fps()
{
    return vision_transport_get_send_max_fps();
}

void vision_pipeline_set_send_enabled(bool enabled)
{
    vision_transport_set_send_enabled(enabled);
}

bool vision_pipeline_is_send_enabled()
{
    return vision_transport_is_send_enabled();
}

void vision_pipeline_set_infer_enabled(bool enabled)
{
    vision_infer_async_set_enabled(enabled);
    if (!enabled)
    {
        reset_dynamic_center_target_offset_state(true);
        clear_infer_result_in_image_processor();
    }
}

bool vision_pipeline_infer_enabled()
{
    return vision_infer_async_enabled();
}

void vision_pipeline_set_ncnn_enabled(bool enabled)
{
    vision_infer_async_set_ncnn_enabled(enabled);
}

bool vision_pipeline_ncnn_enabled()
{
    return vision_infer_async_ncnn_enabled();
}

const char *vision_pipeline_target_board_state_name()
{
    return target_board_state_name(g_target_board_state);
}

bool vision_pipeline_target_board_active()
{
    return g_target_board_state != TARGET_BOARD_NONE;
}

int vision_pipeline_target_board_confirm_count()
{
    return g_target_board_confirm_count;
}

int vision_pipeline_target_board_no_red_count()
{
    return g_target_board_no_red_count;
}

float vision_pipeline_target_board_offset_px()
{
    return g_target_board_applied_offset_px;
}

void vision_pipeline_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy)
{
    vision_image_processor_get_red_rect(found, x, y, w, h, cx, cy);
}

int vision_pipeline_get_red_rect_area()
{
    return vision_image_processor_get_red_rect_area();
}
