#include "driver/vision/vision_pipeline.h"

#include "driver/vision/vision_config.h"
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_infer_async.h"
#include "driver/vision/vision_transport.h"

#include <algorithm>
#include <array>
#include <opencv2/opencv.hpp>

namespace
{
// 当前主链固定处理分辨率与 full 分辨率。
static constexpr int kProcWidth = VISION_DOWNSAMPLED_WIDTH;
static constexpr int kProcHeight = VISION_DOWNSAMPLED_HEIGHT;
static constexpr int kFullWidth = UVC_WIDTH;
static constexpr int kFullHeight = UVC_HEIGHT;

static bool g_center_target_offset_override_active = false;
static int g_center_target_offset_no_red_count = 0;
static uint32 g_last_processed_infer_result_seq = 0;
static float g_center_target_offset_restore_px = g_vision_runtime_config.ipm_center_target_offset_from_left_px;
static int g_infer_offset_vote_valid_result_count = 0;
static std::array<float, VISION_NCNN_MAX_CLASSES> g_infer_offset_prob_sums = {};

static void reset_infer_offset_vote_state()
{
    g_infer_offset_vote_valid_result_count = 0;
    g_infer_offset_prob_sums.fill(0.0f);
}

// 作用：为“识别类别 -> 目标偏移”预留映射接口。
// 当前策略：
// - weapons  -> 左偏配置；
// - supplies -> 右偏配置；
// - vehicles -> 赛道宽度一半（居中）。
static float resolve_center_target_offset_for_class_id(int class_id)
{
    if (class_id == 2)
    {
        return g_vision_runtime_config.ipm_center_target_offset_weapons_from_left_px;
    }
    if (class_id == 0)
    {
        return g_vision_runtime_config.ipm_center_target_offset_supplies_from_left_px;
    }
    return std::max(0.0f, g_vision_runtime_config.ipm_track_width_px * 0.5f);
}

static int select_best_class_id_from_prob_sums()
{
    float best_sum = -1.0f;
    int best_class_id = -1;
    const int class_count = std::clamp(VISION_NCNN_MAX_CLASSES, 0, VISION_NCNN_MAX_CLASSES);
    for (int i = 0; i < class_count; ++i)
    {
        if (g_infer_offset_prob_sums[i] > best_sum)
        {
            best_sum = g_infer_offset_prob_sums[i];
            best_class_id = i;
        }
    }
    return best_class_id;
}

static void restore_center_target_offset_if_needed()
{
    if (!g_center_target_offset_override_active)
    {
        return;
    }
    vision_image_processor_set_ipm_center_target_offset_from_left_px(g_center_target_offset_restore_px);
    g_center_target_offset_override_active = false;
    g_center_target_offset_no_red_count = 0;
    reset_infer_offset_vote_state();
}

static void reset_dynamic_center_target_offset_state(bool restore_default)
{
    if (restore_default)
    {
        restore_center_target_offset_if_needed();
    }
    g_last_processed_infer_result_seq = 0;
    g_center_target_offset_no_red_count = 0;
    reset_infer_offset_vote_state();
    if (!g_center_target_offset_override_active)
    {
        g_center_target_offset_restore_px = vision_image_processor_ipm_center_target_offset_from_left_px();
    }
}

static void update_dynamic_center_target_offset_from_infer_result(const vision_infer_async_result_t &result)
{
    if (result.result_seq == 0 || result.result_seq == g_last_processed_infer_result_seq)
    {
        return;
    }
    g_last_processed_infer_result_seq = result.result_seq;

    if (result.found && result.ncnn_infer_valid)
    {
        g_center_target_offset_no_red_count = 0;
        if (g_center_target_offset_override_active)
        {
            return;
        }

        if (g_infer_offset_vote_valid_result_count == 0)
        {
            g_center_target_offset_restore_px = vision_image_processor_ipm_center_target_offset_from_left_px();
        }

        const int class_count = std::clamp(result.ncnn_class_count, 0, VISION_NCNN_MAX_CLASSES);
        for (int i = 0; i < class_count; ++i)
        {
            g_infer_offset_prob_sums[i] += std::max(0.0f, result.ncnn_probs[i]);
        }
        ++g_infer_offset_vote_valid_result_count;

        const int vote_result_count = std::max(1, g_vision_runtime_config.infer_offset_vote_result_count);
        if (g_infer_offset_vote_valid_result_count < vote_result_count)
        {
            return;
        }

        const int best_class_id = select_best_class_id_from_prob_sums();
        vision_image_processor_set_ipm_center_target_offset_from_left_px(
            resolve_center_target_offset_for_class_id(best_class_id));
        g_center_target_offset_override_active = true;
        reset_infer_offset_vote_state();
        return;
    }

    if (result.found)
    {
        g_center_target_offset_no_red_count = 0;
        return;
    }

    if (!g_center_target_offset_override_active)
    {
        return;
    }

    ++g_center_target_offset_no_red_count;
    if (g_center_target_offset_no_red_count >= std::max(1, g_vision_runtime_config.infer_offset_restore_no_red_count))
    {
        restore_center_target_offset_if_needed();
    }
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
    if (bgr_proc_data != nullptr)
    {
        cv::Mat proc_frame(kProcHeight, kProcWidth, CV_8UC3, const_cast<uint8 *>(bgr_proc_data));
        cv::rectangle(proc_frame,
                      cv::Rect(result->red_x, result->red_y, result->red_w, result->red_h),
                      cv::Scalar(0, 0, 255),
                      1,
                      cv::LINE_8);
        cv::rectangle(proc_frame,
                      cv::Rect(result->ncnn_roi_x, result->ncnn_roi_y, result->ncnn_roi_w, result->ncnn_roi_h),
                      cv::Scalar(0, 255, 0),
                      1,
                      cv::LINE_8);
    }
    if (gray_data != nullptr)
    {
        cv::Mat gray(kProcHeight, kProcWidth, CV_8UC1, const_cast<uint8 *>(gray_data));
        cv::rectangle(gray,
                      cv::Rect(result->red_x, result->red_y, result->red_w, result->red_h),
                      cv::Scalar(200),
                      1,
                      cv::LINE_8);
        cv::rectangle(gray,
                      cv::Rect(result->ncnn_roi_x, result->ncnn_roi_y, result->ncnn_roi_w, result->ncnn_roi_h),
                      cv::Scalar(255),
                      1,
                      cv::LINE_8);
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

void vision_pipeline_set_roi_capture_mode(bool enabled)
{
    vision_infer_async_set_roi_capture_mode(enabled);
}

bool vision_pipeline_roi_capture_mode_enabled()
{
    return vision_infer_async_roi_capture_mode_enabled();
}

void vision_pipeline_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy)
{
    vision_image_processor_get_red_rect(found, x, y, w, h, cx, cy);
}

int vision_pipeline_get_red_rect_area()
{
    return vision_image_processor_get_red_rect_area();
}
