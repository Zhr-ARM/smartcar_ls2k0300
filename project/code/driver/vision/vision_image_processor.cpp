#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_frame_capture.h"
#include "driver/vision/vision_line_error_layer.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>

// 当前视觉流程：
// 1) 固定采集 160x120（与处理分辨率一致）；
// 2) 采集与处理解耦为两个线程（采图在 vision_frame_capture）；
// 3) 使用全图 OTSU 阈值；
// 4) 迷宫法提取左右边线；
// 5) 输出原图边线/中线，并缓存逆透视后的边线与拟合中线。
// 调用关系：
// - 上层入口：vision_pipeline_process_step；
// - 控制使用：line_follow_thread 读取 line_error；
// - 发送使用：transport 读取图像/边线/状态。

#define VISION_BOUNDARY_NUM (VISION_DOWNSAMPLED_HEIGHT * 2)
static constexpr int kRefProcWidth = 160;
static constexpr int kRefProcHeight = 120;
static constexpr int kRefProcPixels = kRefProcWidth * kRefProcHeight;
static constexpr int kProcWidth = VISION_DOWNSAMPLED_WIDTH;
static constexpr int kProcHeight = VISION_DOWNSAMPLED_HEIGHT;

static inline int scale_by_width(int ref_px)
{
    if (ref_px <= 0)
    {
        return 0;
    }
    return std::max(1, (ref_px * kProcWidth + kRefProcWidth / 2) / kRefProcWidth);
}

static inline int scale_by_height(int ref_px)
{
    if (ref_px <= 0)
    {
        return 0;
    }
    return std::max(1, (ref_px * kProcHeight + kRefProcHeight / 2) / kRefProcHeight);
}

static inline int scale_by_area(int ref_area_px)
{
    if (ref_area_px <= 0)
    {
        return 0;
    }
    const int64_t scaled = static_cast<int64_t>(ref_area_px) * kProcWidth * kProcHeight;
    return std::max(1, static_cast<int>((scaled + kRefProcPixels / 2) / kRefProcPixels));
}

static constexpr int kIpmOutputWidth = VISION_IPM_WIDTH;
static constexpr int kIpmOutputHeight = VISION_IPM_HEIGHT;
static constexpr int kMazeStartMinBoundaryGapPx = 5;

// 原图坐标系边线缓存：x1/x2/x3 对应 左/中/右边线，y1/y2/y3 为对应 y。
static uint16 g_xy_x1_boundary[VISION_BOUNDARY_NUM];
static uint16 g_xy_x2_boundary[VISION_BOUNDARY_NUM];
static uint16 g_xy_x3_boundary[VISION_BOUNDARY_NUM];
static uint16 g_xy_y1_boundary[VISION_BOUNDARY_NUM];
static uint16 g_xy_y2_boundary[VISION_BOUNDARY_NUM];
static uint16 g_xy_y3_boundary[VISION_BOUNDARY_NUM];
// 原图坐标系辅助线（角点触发爬线）。
static uint16 g_xy_aux_left_x[VISION_BOUNDARY_NUM];
static uint16 g_xy_aux_left_y[VISION_BOUNDARY_NUM];
static int g_xy_aux_left_count = 0;
static uint16 g_xy_aux_right_x[VISION_BOUNDARY_NUM];
static uint16 g_xy_aux_right_y[VISION_BOUNDARY_NUM];
static int g_xy_aux_right_count = 0;
// 原图坐标系辅助线标记起点（角点偏移点）。
static uint16 g_xy_aux_left_seed_x[1];
static uint16 g_xy_aux_left_seed_y[1];
static int g_xy_aux_left_seed_count = 0;
static uint16 g_xy_aux_right_seed_x[1];
static uint16 g_xy_aux_right_seed_y[1];
static int g_xy_aux_right_seed_count = 0;
static int g_boundary_count = 0;
static int g_boundary_left_count = 0;
static int g_boundary_right_count = 0;
// 逆透视后边界数组（另存）
// 逆透视坐标系边线缓存：供网页状态上报和调试显示使用。
static uint16 g_ipm_xy_x1_boundary[VISION_BOUNDARY_NUM];
static uint16 g_ipm_xy_x2_boundary[VISION_BOUNDARY_NUM];
static uint16 g_ipm_xy_x3_boundary[VISION_BOUNDARY_NUM];
static uint16 g_ipm_xy_y1_boundary[VISION_BOUNDARY_NUM];
static uint16 g_ipm_xy_y2_boundary[VISION_BOUNDARY_NUM];
static uint16 g_ipm_xy_y3_boundary[VISION_BOUNDARY_NUM];
static int g_ipm_boundary_count = 0;
static int g_ipm_boundary_left_count = 0;
static int g_ipm_boundary_right_count = 0;
// 逆透视处理链边界的 7 点 SG 曲率（基于等距采样边线）。
static float g_ipm_left_boundary_sg_curvature[VISION_BOUNDARY_NUM];
static float g_ipm_right_boundary_sg_curvature[VISION_BOUNDARY_NUM];
static int g_ipm_left_boundary_sg_curvature_count = 0;
static int g_ipm_right_boundary_sg_curvature_count = 0;
// 逆透视处理链边界三点法夹角 cos（基于等距采样边线）。
static float g_ipm_left_boundary_angle_cos[VISION_BOUNDARY_NUM];
static float g_ipm_right_boundary_angle_cos[VISION_BOUNDARY_NUM];
static int g_ipm_left_boundary_angle_cos_count = 0;
static int g_ipm_right_boundary_angle_cos_count = 0;
static int g_ipm_left_boundary_corner_indices[VISION_BOUNDARY_NUM];
static int g_ipm_right_boundary_corner_indices[VISION_BOUNDARY_NUM];
static int g_ipm_left_boundary_corner_count = 0;
static int g_ipm_right_boundary_corner_count = 0;
static uint16 g_ipm_left_boundary_corner_x[VISION_BOUNDARY_NUM];
static uint16 g_ipm_left_boundary_corner_y[VISION_BOUNDARY_NUM];
static uint16 g_ipm_right_boundary_corner_x[VISION_BOUNDARY_NUM];
static uint16 g_ipm_right_boundary_corner_y[VISION_BOUNDARY_NUM];
static int g_ipm_left_boundary_corner_point_count = 0;
static int g_ipm_right_boundary_corner_point_count = 0;
static uint16 g_src_left_boundary_corner_x[VISION_BOUNDARY_NUM];
static uint16 g_src_left_boundary_corner_y[VISION_BOUNDARY_NUM];
static uint16 g_src_right_boundary_corner_x[VISION_BOUNDARY_NUM];
static uint16 g_src_right_boundary_corner_y[VISION_BOUNDARY_NUM];
static int g_src_left_boundary_corner_point_count = 0;
static int g_src_right_boundary_corner_point_count = 0;
// 逆透视处理链平移中线（左边界右移/右边界左移）。
static uint16 g_ipm_shift_left_center_x[VISION_BOUNDARY_NUM];
static uint16 g_ipm_shift_left_center_y[VISION_BOUNDARY_NUM];
static int g_ipm_shift_left_center_count = 0;
static uint16 g_ipm_shift_right_center_x[VISION_BOUNDARY_NUM];
static uint16 g_ipm_shift_right_center_y[VISION_BOUNDARY_NUM];
static int g_ipm_shift_right_center_count = 0;
static uint16 g_src_shift_left_center_x[VISION_BOUNDARY_NUM];
static uint16 g_src_shift_left_center_y[VISION_BOUNDARY_NUM];
static int g_src_shift_left_center_count = 0;
static uint16 g_src_shift_right_center_x[VISION_BOUNDARY_NUM];
static uint16 g_src_shift_right_center_y[VISION_BOUNDARY_NUM];
static int g_src_shift_right_center_count = 0;

// 图像缓存：
// - g_image_bgr_full: full 分辨率原图；
// - g_image_bgr/g_image_gray/g_image_binary_u8/g_image_rgb565: 处理分辨率图像。
static uint8 g_image_bgr_full[UVC_HEIGHT * UVC_WIDTH * 3];
static uint8 g_image_bgr[kProcHeight * kProcWidth * 3];
static uint8 g_image_gray[kProcHeight * kProcWidth];
static uint8 g_image_binary_u8[kProcHeight * kProcWidth];
static uint8 g_image_rgb565[kProcHeight * kProcWidth * 2];
// 调试输出缓存：逆透视彩色图。
static uint8 g_image_ipm_bgr[kIpmOutputHeight * kIpmOutputWidth * 3];
// 去畸变映射表：输出(去畸变后)像素 -> 输入(原始畸变图)像素。
static cv::Mat g_undistort_map_x;
static cv::Mat g_undistort_map_y;
static bool g_undistort_ready = false;
// 逆透视矩阵正向映射（原图->俯视）及初始化标志。
static double g_ipm_forward_mat[3][3] = {{0.0}};
static bool g_ipm_forward_ready = false;

// 红框检测结果缓存（由 infer 模块写入，本模块/transport 读取）。
static bool g_red_rect_found = false;
static int g_red_rect_x = 0;
static int g_red_rect_y = 0;
static int g_red_rect_w = 0;
static int g_red_rect_h = 0;
static int g_red_rect_cx = 0;
static int g_red_rect_cy = 0;
static int g_red_rect_area = 0;

// ncnn ROI 缓存（由 infer 模块写入，发送/显示层读取）。
static bool g_ncnn_roi_valid = false;
static int g_ncnn_roi_x = 0;
static int g_ncnn_roi_y = 0;
static int g_ncnn_roi_w = 0;
static int g_ncnn_roi_h = 0;
static std::mutex g_detect_result_mutex;

// 最近一帧处理耗时（us）：用于性能统计与 TCP 状态上报。
static uint32 g_last_capture_wait_us = 0;
static uint32 g_last_preprocess_us = 0;
static uint32 g_last_red_detect_us = 0;
static uint32 g_last_otsu_us = 0;
static uint32 g_last_maze_us = 0;
static uint32 g_last_total_us = 0;
static uint8 g_last_otsu_threshold = 127;
static uint32 g_last_maze_setup_us = 0;
static uint32 g_last_maze_start_us = 0;
static uint32 g_last_maze_trace_left_us = 0;
static uint32 g_last_maze_trace_right_us = 0;
static uint32 g_last_maze_post_us = 0;
static uint16 g_last_maze_left_points = 0;
static uint16 g_last_maze_right_points = 0;
static bool g_last_maze_left_ok = false;
static bool g_last_maze_right_ok = false;
// 记录“最近一次成功识别”的左右起点 x，用于下一帧起点搜索偏移。
static int g_last_maze_left_start_x = -1;
static int g_last_maze_right_start_x = -1;

// 迷宫法起始搜索行（可运行时配置）。
static std::atomic<int> g_maze_start_row(g_vision_runtime_config.maze_start_row);
static std::atomic<int> g_maze_trace_y_fallback_stop_delta(g_vision_runtime_config.maze_trace_y_fallback_stop_delta);
static std::atomic<int> g_maze_trace_x_min(g_vision_processor_config.default_maze_trace_x_min);
static std::atomic<int> g_maze_trace_x_max(g_vision_processor_config.default_maze_trace_x_max);
// 去畸变开关（默认开启）。
static std::atomic<bool> g_undistort_enabled(g_vision_runtime_config.undistort_enabled);
// 逆透视边界三角滤波开关（默认开启）。
static std::atomic<bool> g_ipm_triangle_filter_enabled(g_vision_runtime_config.ipm_triangle_filter_enabled);
// 逆透视边界等距采样开关与参数（默认开启，步长1px）。
static std::atomic<bool> g_ipm_resample_enabled(g_vision_runtime_config.ipm_resample_enabled);
static std::atomic<float> g_ipm_resample_step_px(g_vision_runtime_config.ipm_resample_step_px);
static std::atomic<float> g_ipm_boundary_min_point_dist_px(g_vision_runtime_config.ipm_boundary_min_point_dist_px);
static std::atomic<float> g_ipm_boundary_spike_short_seg_max_px(g_vision_runtime_config.ipm_boundary_spike_short_seg_max_px);
static std::atomic<float> g_ipm_boundary_spike_reverse_cos_threshold(g_vision_runtime_config.ipm_boundary_spike_reverse_cos_threshold);
static std::atomic<float> g_ipm_boundary_kappa_sample_spacing_cm(g_vision_runtime_config.ipm_boundary_kappa_sample_spacing_cm);
static std::atomic<int> g_ipm_boundary_angle_step(g_vision_runtime_config.ipm_boundary_angle_step);
static std::atomic<float> g_ipm_boundary_corner_cos_threshold(g_vision_runtime_config.ipm_boundary_corner_cos_threshold);
static std::atomic<int> g_ipm_boundary_corner_nms_radius(g_vision_runtime_config.ipm_boundary_corner_nms_radius);
static std::atomic<float> g_ipm_boundary_shift_distance_px(g_vision_runtime_config.ipm_boundary_shift_distance_px);
static std::atomic<int> g_ipm_aux_seed_offset_x(g_vision_runtime_config.ipm_aux_seed_offset_x);
static std::atomic<int> g_ipm_aux_seed_offset_y(g_vision_runtime_config.ipm_aux_seed_offset_y);
static std::atomic<int> g_ipm_aux_vertical_min_white_count(g_vision_runtime_config.ipm_aux_vertical_min_white_count);
static std::atomic<int> g_ipm_aux_trace_max_points(g_vision_runtime_config.ipm_aux_trace_max_points);
// 逆透视处理中线独立配置。
static std::atomic<bool> g_ipm_centerline_postprocess_enabled(g_vision_runtime_config.ipm_centerline_postprocess_enabled);
static std::atomic<bool> g_ipm_centerline_triangle_filter_enabled(g_vision_runtime_config.ipm_centerline_triangle_filter_enabled);
static std::atomic<bool> g_ipm_centerline_resample_enabled(g_vision_runtime_config.ipm_centerline_resample_enabled);
static std::atomic<float> g_ipm_centerline_resample_step_px(g_vision_runtime_config.ipm_centerline_resample_step_px);
static std::atomic<bool> g_keep_last_centerline_on_double_loss(g_vision_runtime_config.keep_last_centerline_on_double_loss);
// 平移中线偏好源（左/右/无偏好自动），用于每帧选边。
static std::atomic<int> g_ipm_line_error_preferred_source(static_cast<int>(g_vision_runtime_config.ipm_line_error_source));

// 对外暴露控制量（line_follow_thread 直接读取）。
int line_error = 0;
float line_sample_ratio = g_vision_processor_config.default_line_sample_ratio;

struct maze_point_t
{
    int x;
    int y;
};

static void fill_boundary_arrays_from_maze(const maze_point_t *left_pts,
                                           int left_num,
                                           const maze_point_t *right_pts,
                                           int right_num);
static void trace_auxiliary_lines_from_corners(const uint8 *classify_img,
                                               uint8 white_threshold,
                                               int y_min,
                                               int x_min,
                                               int x_max);
static void append_points_with_bridge_inplace(maze_point_t *base_pts,
                                              int *base_num,
                                              const maze_point_t *extra_pts,
                                              int extra_num,
                                              float bridge_step_px,
                                              int width,
                                              int height);
static int previous_src_centerline_first_x();
static inline bool pixel_is_white(const uint8 *img, int x, int y, uint8 white_threshold);
static void resample_boundary_points_equal_spacing_inplace(maze_point_t *pts, int *num, int width, int height, float step_px);
static bool init_undistort_remap_table();
static bool init_ipm_forward_matrix();
static bool ipm_point_to_src_point(int ipm_x, int ipm_y, int *src_x, int *src_y);
static void get_maze_trace_x_range_clamped(int *x_min, int *x_max);
static bool find_maze_start_from_row(const uint8 *classify_img,
                                     uint8 white_threshold,
                                     bool search_left,
                                     int search_y,
                                     int x_min,
                                     int x_max,
                                     int *start_x,
                                     int *start_y,
                                     bool *wall_is_white,
                                     int preferred_center_x);
static void validate_maze_start_pair(const uint8 *classify_img,
                                     uint8 white_threshold,
                                     int x_min,
                                     int x_max,
                                     bool *left_ok,
                                     int left_start_x,
                                     int left_start_y,
                                     bool *right_ok,
                                     int right_start_x,
                                     int right_start_y);

/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
static constexpr int kDirFront[4][2] = {
    {0, -1},
    {1, 0},
    {0, 1},
    {-1, 0}
};
static constexpr int kDirFrontLeft[4][2] = {
    {-1, -1},
    {1, -1},
    {1, 1},
    {-1, 1}
};
static constexpr int kDirFrontRight[4][2] = {
    {1, -1},
    {1, 1},
    {-1, 1},
    {-1, -1}
};

static void get_maze_trace_x_range_clamped(int *x_min, int *x_max)
{
    const int min_limit = 1;
    const int max_limit = kProcWidth - 2;
    int left = std::clamp(g_maze_trace_x_min.load(), min_limit, max_limit);
    int right = std::clamp(g_maze_trace_x_max.load(), min_limit, max_limit);
    if (left > right)
    {
        std::swap(left, right);
    }

    if (x_min) *x_min = left;
    if (x_max) *x_max = right;
}

static int previous_src_centerline_first_x()
{
    const int preferred_source = vision_line_error_layer_source();
    if (preferred_source == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT))
    {
        if (g_src_shift_right_center_count > 0)
        {
            return static_cast<int>(g_src_shift_right_center_x[0]);
        }
        if (g_src_shift_left_center_count > 0)
        {
            return static_cast<int>(g_src_shift_left_center_x[0]);
        }
    }
    else
    {
        if (g_src_shift_left_center_count > 0)
        {
            return static_cast<int>(g_src_shift_left_center_x[0]);
        }
        if (g_src_shift_right_center_count > 0)
        {
            return static_cast<int>(g_src_shift_right_center_x[0]);
        }
    }
    return kProcWidth / 2;
}

static bool select_shift_source_from_preference_and_counts(int preferred_source, int left_count, int right_count)
{
    const int safe_left_count = std::max(0, left_count);
    const int safe_right_count = std::max(0, right_count);
    const bool left_available = (safe_left_count > 0);
    const bool right_available = (safe_right_count > 0);

    if (preferred_source == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT) && left_available)
    {
        return false;
    }
    if (preferred_source == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT) && right_available)
    {
        return true;
    }

    if (!left_available && !right_available)
    {
        return (vision_line_error_layer_source() == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT));
    }
    if (!left_available)
    {
        return true;
    }
    if (!right_available)
    {
        return false;
    }
    if (safe_right_count > safe_left_count)
    {
        return true;
    }
    if (safe_left_count > safe_right_count)
    {
        return false;
    }
    return (vision_line_error_layer_source() == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT));
}

static void clear_boundary_arrays()
{
    // 作用：清空原图边线缓存，避免上帧残留。
    std::fill_n(g_xy_x1_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_xy_x2_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_xy_x3_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_xy_y1_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_xy_y2_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_xy_y3_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_xy_aux_left_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_xy_aux_left_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_xy_aux_right_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_xy_aux_right_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_xy_aux_left_seed_x, 1, static_cast<uint16>(0));
    std::fill_n(g_xy_aux_left_seed_y, 1, static_cast<uint16>(0));
    std::fill_n(g_xy_aux_right_seed_x, 1, static_cast<uint16>(0));
    std::fill_n(g_xy_aux_right_seed_y, 1, static_cast<uint16>(0));
    g_xy_aux_left_count = 0;
    g_xy_aux_right_count = 0;
    g_xy_aux_left_seed_count = 0;
    g_xy_aux_right_seed_count = 0;
    g_boundary_count = 0;
    g_boundary_left_count = 0;
    g_boundary_right_count = 0;
}

static void clear_ipm_saved_arrays()
{
    // 作用：清空逆透视边线和中线缓存。
    std::fill_n(g_ipm_xy_x1_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_xy_x2_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_xy_x3_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_xy_y1_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_xy_y2_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_xy_y3_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_left_boundary_sg_curvature, VISION_BOUNDARY_NUM, 0.0f);
    std::fill_n(g_ipm_right_boundary_sg_curvature, VISION_BOUNDARY_NUM, 0.0f);
    std::fill_n(g_ipm_left_boundary_angle_cos, VISION_BOUNDARY_NUM, 0.0f);
    std::fill_n(g_ipm_right_boundary_angle_cos, VISION_BOUNDARY_NUM, 0.0f);
    std::fill_n(g_ipm_left_boundary_corner_indices, VISION_BOUNDARY_NUM, -1);
    std::fill_n(g_ipm_right_boundary_corner_indices, VISION_BOUNDARY_NUM, -1);
    std::fill_n(g_ipm_left_boundary_corner_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_left_boundary_corner_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_right_boundary_corner_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_right_boundary_corner_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_src_left_boundary_corner_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_src_left_boundary_corner_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_src_right_boundary_corner_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_src_right_boundary_corner_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_shift_left_center_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_shift_left_center_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_shift_right_center_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_shift_right_center_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    g_ipm_boundary_count = 0;
    g_ipm_boundary_left_count = 0;
    g_ipm_boundary_right_count = 0;
    g_ipm_left_boundary_sg_curvature_count = 0;
    g_ipm_right_boundary_sg_curvature_count = 0;
    g_ipm_left_boundary_angle_cos_count = 0;
    g_ipm_right_boundary_angle_cos_count = 0;
    g_ipm_left_boundary_corner_count = 0;
    g_ipm_right_boundary_corner_count = 0;
    g_ipm_left_boundary_corner_point_count = 0;
    g_ipm_right_boundary_corner_point_count = 0;
    g_src_left_boundary_corner_point_count = 0;
    g_src_right_boundary_corner_point_count = 0;
    g_ipm_shift_left_center_count = 0;
    g_ipm_shift_right_center_count = 0;
}

static uint8 compute_global_otsu_threshold_u8(const uint8 *gray_img)
{
    if (gray_img == nullptr)
    {
        return 127;
    }

    std::array<uint32, 256> histogram{};
    uint64_t gray_sum = 0;
    constexpr int kPixelCount = kProcWidth * kProcHeight;

    for (int i = 0; i < kPixelCount; ++i)
    {
        uint8 v = gray_img[i];
        ++histogram[v];
        gray_sum += static_cast<uint64_t>(v);
    }

    uint32 weight_bg = 0;
    uint64_t sum_bg = 0;
    double best_between_var = -1.0;
    uint8 best_threshold = 127;

    for (int t = 0; t < 256; ++t)
    {
        weight_bg += histogram[t];
        if (weight_bg == 0)
        {
            continue;
        }

        uint32 weight_fg = static_cast<uint32>(kPixelCount) - weight_bg;
        if (weight_fg == 0)
        {
            break;
        }

        sum_bg += static_cast<uint64_t>(histogram[t]) * static_cast<uint64_t>(t);

        double mean_bg = static_cast<double>(sum_bg) / static_cast<double>(weight_bg);
        double mean_fg = static_cast<double>(gray_sum - sum_bg) / static_cast<double>(weight_fg);
        double diff = mean_bg - mean_fg;
        double between_var = static_cast<double>(weight_bg) * static_cast<double>(weight_fg) * diff * diff;

        if (between_var > best_between_var)
        {
            best_between_var = between_var;
            best_threshold = static_cast<uint8>(t);
        }
    }

    return best_threshold;
}

static inline bool pixel_is_white(const uint8 *img, int x, int y, uint8 white_threshold)
{
    return img[y * kProcWidth + x] > white_threshold;
}

static inline bool pixel_is_wall(const uint8 *img, int x, int y, uint8 white_threshold, bool wall_is_white)
{
    return wall_is_white ? pixel_is_white(img, x, y, white_threshold) : !pixel_is_white(img, x, y, white_threshold);
}

static void build_binary_image_from_gray_threshold(const uint8 *gray_img, uint8 threshold)
{
    if (gray_img == nullptr)
    {
        std::fill_n(g_image_binary_u8, kProcWidth * kProcHeight, static_cast<uint8>(0));
        return;
    }

    constexpr int kPixelCount = kProcWidth * kProcHeight;
    for (int i = 0; i < kPixelCount; ++i)
    {
        g_image_binary_u8[i] = (gray_img[i] > threshold) ? static_cast<uint8>(255) : static_cast<uint8>(0);
    }
}

static bool init_undistort_remap_table()
{
    if (g_undistort_ready)
    {
        return true;
    }

    g_undistort_map_x = cv::Mat(kProcHeight, kProcWidth, CV_32FC1);
    g_undistort_map_y = cv::Mat(kProcHeight, kProcWidth, CV_32FC1);

    const double fx = g_vision_processor_config.camera_matrix[0][0];
    const double fy = g_vision_processor_config.camera_matrix[1][1];
    const double ux = g_vision_processor_config.camera_matrix[0][2];
    const double uy = g_vision_processor_config.camera_matrix[1][2];
    if (std::fabs(fx) < 1e-9 || std::fabs(fy) < 1e-9)
    {
        g_undistort_ready = false;
        return false;
    }

    const double k1 = g_vision_processor_config.dist_coeffs[0];
    const double k2 = g_vision_processor_config.dist_coeffs[1];
    const double p1 = g_vision_processor_config.dist_coeffs[2];
    const double p2 = g_vision_processor_config.dist_coeffs[3];
    const double k3 = g_vision_processor_config.dist_coeffs[4];
    const int move_x = g_vision_processor_config.undistort_move_x;
    const int move_y = g_vision_processor_config.undistort_move_y;

    for (int y = 0; y < kProcHeight; ++y)
    {
        float *row_x = g_undistort_map_x.ptr<float>(y);
        float *row_y = g_undistort_map_y.ptr<float>(y);
        for (int x = 0; x < kProcWidth; ++x)
        {
            const double xu = static_cast<double>(x - move_x);
            const double yu = static_cast<double>(y - move_y);

            const double x_corrected = (xu - ux) / fx;
            const double y_corrected = (yu - uy) / fy;
            const double r2 = x_corrected * x_corrected + y_corrected * y_corrected;
            const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            const double delta_tx = 2.0 * p1 * x_corrected * y_corrected + p2 * (r2 + 2.0 * x_corrected * x_corrected);
            const double delta_ty = p1 * (r2 + 2.0 * y_corrected * y_corrected) + 2.0 * p2 * x_corrected * y_corrected;

            const double x_distorted = (x_corrected * radial + delta_tx) * fx + ux;
            const double y_distorted = (y_corrected * radial + delta_ty) * fy + uy;
            row_x[x] = static_cast<float>(x_distorted);
            row_y[x] = static_cast<float>(y_distorted);
        }
    }

    g_undistort_ready = true;
    return true;
}

[[maybe_unused]] static cv::Rect build_red_search_roi_from_midline()
{
    const int width = kProcWidth;
    const int height = kProcHeight;

    // 无中线时回退到图像中部窗口。
    auto fallback_center_roi = [width, height]() -> cv::Rect {
        const int rw = std::clamp(width / 2, scale_by_width(48), width);
        const int rh = std::clamp(height / 2, scale_by_height(36), height);
        int rx = (width - rw) / 2;
        int ry = (height - rh) / 2;
        rx = std::clamp(rx, 0, std::max(0, width - rw));
        ry = std::clamp(ry, 0, std::max(0, height - rh));
        return cv::Rect(rx, ry, rw, rh);
    };

    if (g_boundary_count <= 0)
    {
        return fallback_center_roi();
    }

    int min_x = width - 1;
    int max_x = 0;
    int min_y = height - 1;
    int max_y = 0;
    int valid = 0;
    for (int i = 0; i < g_boundary_count; ++i)
    {
        const int x = std::clamp(static_cast<int>(g_xy_x2_boundary[i]), 0, width - 1);
        const int y = std::clamp(static_cast<int>(g_xy_y2_boundary[i]), 0, height - 1);
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        ++valid;
    }
    if (valid <= 0)
    {
        return fallback_center_roi();
    }

    // 只在中线附近搜索，减少计算量。
    const int kExpandX = scale_by_width(28);
    const int kExpandY = scale_by_height(24);
    int x0 = std::max(0, min_x - kExpandX);
    int y0 = std::max(0, min_y - kExpandY);
    int x1 = std::min(width - 1, max_x + kExpandX);
    int y1 = std::min(height - 1, max_y + kExpandY);

    // 保证ROI最小尺寸，避免中线太短导致窗口过小。
    const int kMinRoiW = scale_by_width(48);
    const int kMinRoiH = scale_by_height(36);
    int rw = x1 - x0 + 1;
    int rh = y1 - y0 + 1;
    if (rw < kMinRoiW)
    {
        const int cx = (x0 + x1) / 2;
        x0 = std::max(0, cx - kMinRoiW / 2);
        x1 = std::min(width - 1, x0 + kMinRoiW - 1);
        x0 = std::max(0, x1 - kMinRoiW + 1);
        rw = x1 - x0 + 1;
    }
    if (rh < kMinRoiH)
    {
        const int cy = (y0 + y1) / 2;
        y0 = std::max(0, cy - kMinRoiH / 2);
        y1 = std::min(height - 1, y0 + kMinRoiH - 1);
        y0 = std::max(0, y1 - kMinRoiH + 1);
        rh = y1 - y0 + 1;
    }

    if (rw <= 0 || rh <= 0)
    {
        return fallback_center_roi();
    }
    return cv::Rect(x0, y0, rw, rh);
}

[[maybe_unused]] static bool detect_red_rectangle_bbox(const cv::Mat &bgr, cv::Rect *bbox, int *area_px)
{
    if (bbox == nullptr || area_px == nullptr || bgr.empty() || bgr.type() != CV_8UC3)
    {
        return false;
    }

    const cv::Rect roi = build_red_search_roi_from_midline();
    if (roi.width <= 0 || roi.height <= 0)
    {
        return false;
    }

    cv::Mat mask(roi.height, roi.width, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < roi.height; ++y)
    {
        const cv::Vec3b *src = bgr.ptr<cv::Vec3b>(roi.y + y);
        uint8 *dst = mask.ptr<uint8>(y);
        for (int x = 0; x < roi.width; ++x)
        {
            const cv::Vec3b &px = src[roi.x + x];
            const int b = static_cast<int>(px[0]);
            const int g = static_cast<int>(px[1]);
            const int r = static_cast<int>(px[2]);
            const int rg = r - g;
            const int rb = r - b;
            if (r >= 80 && rg >= 28 && rb >= 22)
            {
                dst[x] = 255;
            }
        }
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int comp_num = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    if (comp_num <= 1)
    {
        return false;
    }

    const int kExpectedAreaPx = scale_by_area(60);
    const int kAreaMinPx = scale_by_area(20);
    const int kAreaMaxPx = scale_by_area(450);
    const int kMinCompW = scale_by_width(3);
    const int kMinCompH = scale_by_height(3);

    int best_label = -1;
    int best_score = std::numeric_limits<int>::max();
    int best_area = 0;
    for (int i = 1; i < comp_num; ++i)
    {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        const int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        const int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        if (area < kAreaMinPx || area > kAreaMaxPx || w < kMinCompW || h < kMinCompH)
        {
            continue;
        }

        const int side_min = std::min(w, h);
        const int side_max = std::max(w, h);
        if (side_min <= 0 || (side_max > side_min * 6))
        {
            continue;
        }

        const int area_cost = std::abs(area - kExpectedAreaPx);
        const int shape_cost = std::abs((w * 10) - (h * 24)); // 12:5 近似为 24:10
        const int score = area_cost * 4 + shape_cost;
        if (score < best_score || (score == best_score && area > best_area))
        {
            best_score = score;
            best_label = i;
            best_area = area;
        }
    }

    if (best_label < 0)
    {
        return false;
    }

    const int x = stats.at<int>(best_label, cv::CC_STAT_LEFT) + roi.x;
    const int y = stats.at<int>(best_label, cv::CC_STAT_TOP) + roi.y;
    const int w = stats.at<int>(best_label, cv::CC_STAT_WIDTH);
    const int h = stats.at<int>(best_label, cv::CC_STAT_HEIGHT);
    *bbox = cv::Rect(x, y, w, h);
    *area_px = best_area;
    return true;
}

static bool init_ipm_forward_matrix()
{
    cv::Mat h_inv(3, 3, CV_64F, const_cast<double *>(&g_vision_processor_config.change_un_mat[0][0]));
    cv::Mat h(3, 3, CV_64F);
    const double det = cv::invert(h_inv, h, cv::DECOMP_LU);
    if (std::fabs(det) < 1e-12)
    {
        g_ipm_forward_ready = false;
        return false;
    }

    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            g_ipm_forward_mat[r][c] = h.at<double>(r, c);
        }
    }
    g_ipm_forward_ready = true;
    return true;
}

static bool src_point_to_ipm_point(int src_x, int src_y, int *ipm_x, int *ipm_y)
{
    if (ipm_x == nullptr || ipm_y == nullptr)
    {
        return false;
    }

    if (!g_ipm_forward_ready && !init_ipm_forward_matrix())
    {
        return false;
    }

    // 逆透视矩阵按 160x120 标定；在其他分辨率下先映射回标定坐标系。
    const double src_x_ref = (kProcWidth > 1)
                                 ? (static_cast<double>(src_x) * static_cast<double>(kRefProcWidth - 1) /
                                    static_cast<double>(kProcWidth - 1))
                                 : 0.0;
    const double src_y_ref = (kProcHeight > 1)
                                 ? (static_cast<double>(src_y) * static_cast<double>(kRefProcHeight - 1) /
                                    static_cast<double>(kProcHeight - 1))
                                 : 0.0;

    const double w = g_ipm_forward_mat[2][0] * src_x_ref + g_ipm_forward_mat[2][1] * src_y_ref + g_ipm_forward_mat[2][2];
    if (std::fabs(w) < 1e-9)
    {
        return false;
    }

    const int x = static_cast<int>(std::lround((g_ipm_forward_mat[0][0] * src_x_ref + g_ipm_forward_mat[0][1] * src_y_ref + g_ipm_forward_mat[0][2]) / w));
    const int y = static_cast<int>(std::lround((g_ipm_forward_mat[1][0] * src_x_ref + g_ipm_forward_mat[1][1] * src_y_ref + g_ipm_forward_mat[1][2]) / w));
    if (x < 0 || x >= kIpmOutputWidth || y < 0 || y >= kIpmOutputHeight)
    {
        return false;
    }

    *ipm_x = x;
    *ipm_y = y;
    return true;
}

static bool ipm_point_to_src_point(int ipm_x, int ipm_y, int *src_x, int *src_y)
{
    if (src_x == nullptr || src_y == nullptr)
    {
        return false;
    }

    const double (*h_inv)[3] = g_vision_processor_config.change_un_mat;
    const double w = h_inv[2][0] * ipm_x + h_inv[2][1] * ipm_y + h_inv[2][2];
    if (std::fabs(w) < 1e-9)
    {
        return false;
    }

    const double src_x_ref = (h_inv[0][0] * ipm_x + h_inv[0][1] * ipm_y + h_inv[0][2]) / w;
    const double src_y_ref = (h_inv[1][0] * ipm_x + h_inv[1][1] * ipm_y + h_inv[1][2]) / w;
    const int x = (kRefProcWidth > 1)
                      ? static_cast<int>(std::lround(src_x_ref * static_cast<double>(kProcWidth - 1) /
                                                     static_cast<double>(kRefProcWidth - 1)))
                      : 0;
    const int y = (kRefProcHeight > 1)
                      ? static_cast<int>(std::lround(src_y_ref * static_cast<double>(kProcHeight - 1) /
                                                     static_cast<double>(kRefProcHeight - 1)))
                      : 0;
    if (x < 0 || x >= kProcWidth || y < 0 || y >= kProcHeight)
    {
        return false;
    }

    *src_x = x;
    *src_y = y;
    return true;
}

static inline uint16 clamp_u16_to_range(int v, int max_inclusive)
{
    return static_cast<uint16>(std::clamp(v, 0, std::max(0, max_inclusive)));
}

static void fill_boundary_arrays_from_points_to_target(const maze_point_t *left_pts,
                                                       int left_num,
                                                       const maze_point_t *right_pts,
                                                       int right_num,
                                                       uint16 *x1,
                                                       uint16 *x2,
                                                       uint16 *x3,
                                                       uint16 *y1,
                                                       uint16 *y2,
                                                       uint16 *y3,
                                                       int *count,
                                                       int width,
                                                       int height)
{
    if (x1 == nullptr || x2 == nullptr || x3 == nullptr ||
        y1 == nullptr || y2 == nullptr || y3 == nullptr || count == nullptr)
    {
        return;
    }

    std::fill_n(x1, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(x2, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(x3, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(y1, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(y2, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(y3, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    *count = 0;

    const int clipped_left_num = std::clamp(left_num, 0, VISION_BOUNDARY_NUM);
    const int clipped_right_num = std::clamp(right_num, 0, VISION_BOUNDARY_NUM);
    const int out_num = std::max(clipped_left_num, clipped_right_num);
    if (out_num <= 0)
    {
        return;
    }

    const int max_x = width - 1;
    const int max_y = height - 1;
    for (int i = 0; i < out_num; ++i)
    {
        const int li = (clipped_left_num > 0) ? std::min(i, clipped_left_num - 1) : 0;
        const int ri = (clipped_right_num > 0) ? std::min(i, clipped_right_num - 1) : 0;

        int lx = (clipped_left_num > 0 && i < clipped_left_num) ? left_pts[li].x : 0;
        int ly = (clipped_left_num > 0 && i < clipped_left_num) ? left_pts[li].y : 0;
        int rx = (clipped_right_num > 0 && i < clipped_right_num) ? right_pts[ri].x : 0;
        int ry = (clipped_right_num > 0 && i < clipped_right_num) ? right_pts[ri].y : 0;

        lx = std::clamp(lx, 0, max_x);
        ly = std::clamp(ly, 0, max_y);
        rx = std::clamp(rx, 0, max_x);
        ry = std::clamp(ry, 0, max_y);

        x1[i] = clamp_u16_to_range(lx, max_x);
        y1[i] = clamp_u16_to_range(ly, max_y);
        x3[i] = clamp_u16_to_range(rx, max_x);
        y3[i] = clamp_u16_to_range(ry, max_y);
        if (clipped_left_num > 0 && clipped_right_num > 0)
        {
            x2[i] = clamp_u16_to_range((lx + rx) / 2, max_x);
            y2[i] = clamp_u16_to_range((ly + ry) / 2, max_y);
        }
        else if (clipped_left_num > 0 && i < clipped_left_num)
        {
            x2[i] = clamp_u16_to_range(lx, max_x);
            y2[i] = clamp_u16_to_range(ly, max_y);
        }
        else if (clipped_right_num > 0 && i < clipped_right_num)
        {
            x2[i] = clamp_u16_to_range(rx, max_x);
            y2[i] = clamp_u16_to_range(ry, max_y);
        }
    }
    *count = out_num;
}

static void fill_single_line_arrays_from_points(const maze_point_t *pts,
                                                int num,
                                                uint16 *xs,
                                                uint16 *ys,
                                                int *count,
                                                int width,
                                                int height)
{
    if (pts == nullptr || xs == nullptr || ys == nullptr || count == nullptr)
    {
        return;
    }

    std::fill_n(xs, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(ys, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    *count = 0;

    const int out_num = std::clamp(num, 0, VISION_BOUNDARY_NUM);
    if (out_num <= 0)
    {
        return;
    }

    const int max_x = width - 1;
    const int max_y = height - 1;
    for (int i = 0; i < out_num; ++i)
    {
        xs[i] = clamp_u16_to_range(pts[i].x, max_x);
        ys[i] = clamp_u16_to_range(pts[i].y, max_y);
    }
    *count = out_num;
}

static void shift_boundary_along_normal(const maze_point_t *src,
                                        int src_num,
                                        const maze_point_t *guide,
                                        int guide_num,
                                        maze_point_t *dst,
                                        int *dst_num,
                                        int width,
                                        int height,
                                        float shift_dist_px,
                                        bool first_point_prefer_positive_x)
{
    if (dst_num) *dst_num = 0;
    if (src == nullptr || dst == nullptr || dst_num == nullptr || src_num <= 0 || width <= 0 || height <= 0)
    {
        return;
    }

    const int num = std::clamp(src_num, 0, VISION_BOUNDARY_NUM);
    const float dist = std::max(0.0f, shift_dist_px);
    const int max_x = width - 1;
    const int max_y = height - 1;
    float prev_nx = 0.0f;
    float prev_ny = 0.0f;
    bool has_prev_normal = false;

    for (int i = 0; i < num; ++i)
    {
        const int prev_i = (i > 0) ? (i - 1) : i;
        const int next_i = (i + 1 < num) ? (i + 1) : i;
        const float tx = static_cast<float>(src[next_i].x - src[prev_i].x);
        const float ty = static_cast<float>(src[next_i].y - src[prev_i].y);

        // 两个候选法向：(-ty, tx) 和 (ty, -tx)。
        float nx1 = -ty;
        float ny1 = tx;
        float nx2 = ty;
        float ny2 = -tx;

        // 动态方向约束：法向应朝向“对侧边界对应点”方向（非固定dx/dy符号）。
        float gx = 0.0f;
        float gy = 0.0f;
        bool has_guide = false;
        if (guide != nullptr && guide_num > 0)
        {
            const int gi = (guide_num <= 1 || num <= 1) ? 0 : (i * (guide_num - 1)) / (num - 1);
            gx = static_cast<float>(guide[gi].x - src[i].x);
            gy = static_cast<float>(guide[gi].y - src[i].y);
            has_guide = (std::fabs(gx) + std::fabs(gy)) > 1e-6f;
        }

        float score1 = 0.0f;
        float score2 = 0.0f;
        if (i == 0)
        {
            // 首点方向固定规则：
            // - left:  prefer_positive_x = true  -> 往右；
            // - right: prefer_positive_x = false -> 往左。
            score1 = first_point_prefer_positive_x ? nx1 : -nx1;
            score2 = first_point_prefer_positive_x ? nx2 : -nx2;
        }
        else if (has_guide)
        {
            score1 = nx1 * gx + ny1 * gy;
            score2 = nx2 * gx + ny2 * gy;
        }
        else if (has_prev_normal)
        {
            score1 = nx1 * prev_nx + ny1 * prev_ny;
            score2 = nx2 * prev_nx + ny2 * prev_ny;
        }
        else
        {
            score1 = std::fabs(nx1) + std::fabs(ny1);
            score2 = std::fabs(nx2) + std::fabs(ny2);
        }

        float nx = (score1 >= score2) ? nx1 : nx2;
        float ny = (score1 >= score2) ? ny1 : ny2;
        float nlen = std::sqrt(nx * nx + ny * ny);
        if (nlen < 1e-6f)
        {
            if (has_prev_normal)
            {
                nx = prev_nx;
                ny = prev_ny;
                nlen = std::sqrt(nx * nx + ny * ny);
            }
            if (nlen < 1e-6f)
            {
                nx = 1.0f;
                ny = 0.0f;
                nlen = 1.0f;
            }
        }

        // 连续性约束：与上一点法向保持同向，避免翻转跳变。
        if (has_prev_normal)
        {
            const float dot = nx * prev_nx + ny * prev_ny;
            if (dot < 0.0f)
            {
                nx = -nx;
                ny = -ny;
            }
        }

        const int x = std::clamp(static_cast<int>(std::lround(static_cast<float>(src[i].x) + dist * (nx / nlen))), 0, max_x);
        const int y = std::clamp(static_cast<int>(std::lround(static_cast<float>(src[i].y) + dist * (ny / nlen))), 0, max_y);
        dst[i] = {x, y};
        prev_nx = nx / nlen;
        prev_ny = ny / nlen;
        has_prev_normal = true;
    }
    *dst_num = num;
}

static int transform_boundary_points_to_ipm(const maze_point_t *src_pts, int src_num, maze_point_t *dst_pts, int max_pts)
{
    if (src_pts == nullptr || dst_pts == nullptr || src_num <= 0 || max_pts <= 0)
    {
        return 0;
    }

    int out = 0;
    for (int i = 0; i < src_num && out < max_pts; ++i)
    {
        int ipm_x = 0;
        int ipm_y = 0;
        if (!src_point_to_ipm_point(src_pts[i].x, src_pts[i].y, &ipm_x, &ipm_y))
        {
            continue;
        }

        dst_pts[out++] = {ipm_x, ipm_y};
    }
    return out;
}

static int transform_boundary_points_from_ipm_to_src(const maze_point_t *src_pts, int src_num, maze_point_t *dst_pts, int max_pts)
{
    if (src_pts == nullptr || dst_pts == nullptr || src_num <= 0 || max_pts <= 0)
    {
        return 0;
    }

    int out = 0;
    for (int i = 0; i < src_num && out < max_pts; ++i)
    {
        int src_x = 0;
        int src_y = 0;
        if (!ipm_point_to_src_point(src_pts[i].x, src_pts[i].y, &src_x, &src_y))
        {
            continue;
        }

        dst_pts[out++] = {src_x, src_y};
    }
    return out;
}

static void triangle_filter_boundary_points_inplace(maze_point_t *pts, int num, int width, int height)
{
    if (pts == nullptr || num < 3 || width <= 0 || height <= 0)
    {
        return;
    }

    std::array<maze_point_t, VISION_BOUNDARY_NUM> src{};
    const int copy_num = std::clamp(num, 0, static_cast<int>(src.size()));
    for (int i = 0; i < copy_num; ++i)
    {
        src[static_cast<size_t>(i)] = pts[i];
    }

    const int max_x = width - 1;
    const int max_y = height - 1;
    for (int i = 1; i + 1 < copy_num; ++i)
    {
        const int fx = src[static_cast<size_t>(i - 1)].x +
                       src[static_cast<size_t>(i)].x * 2 +
                       src[static_cast<size_t>(i + 1)].x;
        const int fy = src[static_cast<size_t>(i - 1)].y +
                       src[static_cast<size_t>(i)].y * 2 +
                       src[static_cast<size_t>(i + 1)].y;
        pts[i].x = std::clamp((fx + 2) / 4, 0, max_x);
        pts[i].y = std::clamp((fy + 2) / 4, 0, max_y);
    }
}

static void compute_span10_boundary_curvature_inplace(const maze_point_t *pts,
                                                      int num,
                                                      float sample_spacing,
                                                      float *kappa_out,
                                                      int *kappa_count)
{
    if (kappa_count) *kappa_count = 0;
    if (pts == nullptr || kappa_out == nullptr || kappa_count == nullptr || num <= 0)
    {
        return;
    }

    const int n = std::clamp(num, 0, VISION_BOUNDARY_NUM);
    const float h = std::max(1e-6f, sample_spacing);
    // i±5（跨度10点）导数，首尾通过索引夹紧进行“补点”。
    static constexpr int kSpan = 5;
    const float inv_2sh = 1.0f / (2.0f * static_cast<float>(kSpan) * h);
    const float inv_sh2 = 1.0f / (static_cast<float>(kSpan * kSpan) * h * h);

    std::fill_n(kappa_out, VISION_BOUNDARY_NUM, 0.0f);
    if (n < 2)
    {
        *kappa_count = n;
        return;
    }

    for (int i = 0; i < n; ++i)
    {
        const int im = std::clamp(i - kSpan, 0, n - 1);
        const int ip = std::clamp(i + kSpan, 0, n - 1);
        if (im == ip)
        {
            continue;
        }

        const float xm = static_cast<float>(pts[im].x);
        const float x0 = static_cast<float>(pts[i].x);
        const float xp = static_cast<float>(pts[ip].x);

        const float ym = static_cast<float>(pts[im].y);
        const float y0 = static_cast<float>(pts[i].y);
        const float yp = static_cast<float>(pts[ip].y);

        // 一阶导数（二点中心差分，跨度10点）。
        const float x1 = (xp - xm) * inv_2sh;
        const float y1 = (yp - ym) * inv_2sh;
        // 二阶导数（三点中心差分，跨度10点）。
        const float x2 = (xp - 2.0f * x0 + xm) * inv_sh2;
        const float y2 = (yp - 2.0f * y0 + ym) * inv_sh2;

        kappa_out[i] = x1 * y2 - x2 * y1;
    }

    // 曲率三点平滑：内部点用 (prev + 2*cur + next) / 4。
    std::array<float, VISION_BOUNDARY_NUM> kappa_src{};
    for (int i = 0; i < n; ++i)
    {
        kappa_src[static_cast<size_t>(i)] = kappa_out[i];
    }
    for (int i = 1; i + 1 < n; ++i)
    {
        kappa_out[i] = (kappa_src[static_cast<size_t>(i - 1)] +
                        2.0f * kappa_src[static_cast<size_t>(i)] +
                        kappa_src[static_cast<size_t>(i + 1)]) * 0.25f;
    }

    *kappa_count = n;
}

static void compute_boundary_angle_cos_3point_inplace(const maze_point_t *pts,
                                                      int num,
                                                      int step,
                                                      float *angle_cos_out,
                                                      int *angle_cos_count)
{
    if (angle_cos_count) *angle_cos_count = 0;
    if (pts == nullptr || angle_cos_out == nullptr || angle_cos_count == nullptr || num <= 0)
    {
        return;
    }

    const int n = std::clamp(num, 0, VISION_BOUNDARY_NUM);
    const int s = std::max(1, step);
    std::fill_n(angle_cos_out, VISION_BOUNDARY_NUM, 0.0f);
    // 首尾各 step 范围内无法构成完整三点夹角，按需求置为 1。
    const int head_tail_span = std::min(s, n);
    for (int i = 0; i < head_tail_span; ++i)
    {
        angle_cos_out[i] = 1.0f;
    }
    for (int i = std::max(0, n - head_tail_span); i < n; ++i)
    {
        angle_cos_out[i] = 1.0f;
    }
    if (n < (2 * s + 1))
    {
        *angle_cos_count = n;
        return;
    }

    for (int i = s; i + s < n; ++i)
    {
        const float vx1 = static_cast<float>(pts[i].x - pts[i - s].x);
        const float vy1 = static_cast<float>(pts[i].y - pts[i - s].y);
        const float vx2 = static_cast<float>(pts[i + s].x - pts[i].x);
        const float vy2 = static_cast<float>(pts[i + s].y - pts[i].y);
        const float norm1 = std::sqrt(vx1 * vx1 + vy1 * vy1);
        const float norm2 = std::sqrt(vx2 * vx2 + vy2 * vy2);
        if (norm1 < 1e-6f || norm2 < 1e-6f)
        {
            angle_cos_out[i] = 0.0f;
            continue;
        }
        const float cos_v = (vx1 * vx2 + vy1 * vy2) / (norm1 * norm2);
        angle_cos_out[i] = std::clamp(cos_v, -1.0f, 1.0f);
    }

    *angle_cos_count = n;
}

static void remove_near_duplicate_boundary_points_inplace(maze_point_t *pts, int *num, float min_dist_px)
{
    if (pts == nullptr || num == nullptr || *num <= 1)
    {
        return;
    }

    const int in_num = std::clamp(*num, 0, VISION_BOUNDARY_NUM);
    if (in_num <= 1)
    {
        *num = in_num;
        return;
    }

    const float min_dist_sq = std::max(0.0f, min_dist_px) * std::max(0.0f, min_dist_px);
    int out = 1;
    for (int i = 1; i < in_num; ++i)
    {
        const float dx = static_cast<float>(pts[i].x - pts[out - 1].x);
        const float dy = static_cast<float>(pts[i].y - pts[out - 1].y);
        if ((dx * dx + dy * dy) >= min_dist_sq)
        {
            pts[out++] = pts[i];
        }
    }
    *num = out;
}

static void remove_backtrack_spikes_inplace(maze_point_t *pts,
                                            int *num,
                                            float short_seg_max_px,
                                            float reverse_cos_threshold)
{
    if (pts == nullptr || num == nullptr || *num < 3)
    {
        return;
    }

    int n = std::clamp(*num, 0, VISION_BOUNDARY_NUM);
    if (n < 3)
    {
        *num = n;
        return;
    }

    const float short_seg_max_sq = std::max(0.0f, short_seg_max_px) * std::max(0.0f, short_seg_max_px);
    const float cos_th = std::clamp(reverse_cos_threshold, -1.0f, 1.0f);

    bool changed = true;
    int pass = 0;
    while (changed && pass < 3)
    {
        changed = false;
        ++pass;

        int out = 1;
        int i = 1;
        while (i + 1 < n)
        {
            const float v1x = static_cast<float>(pts[i].x - pts[out - 1].x);
            const float v1y = static_cast<float>(pts[i].y - pts[out - 1].y);
            const float v2x = static_cast<float>(pts[i + 1].x - pts[i].x);
            const float v2y = static_cast<float>(pts[i + 1].y - pts[i].y);
            const float d1 = v1x * v1x + v1y * v1y;
            const float d2 = v2x * v2x + v2y * v2y;
            const float n1 = std::sqrt(d1);
            const float n2 = std::sqrt(d2);
            float cos_v = 1.0f;
            if (n1 > 1e-6f && n2 > 1e-6f)
            {
                cos_v = (v1x * v2x + v1y * v2y) / (n1 * n2);
            }

            // 删除 1 点回跳毛刺：相邻两段都很短，且方向明显反转。
            if (d1 <= short_seg_max_sq && d2 <= short_seg_max_sq && cos_v <= cos_th)
            {
                changed = true;
                ++i;
                continue;
            }

            pts[out++] = pts[i];
            ++i;
        }
        pts[out++] = pts[n - 1];
        n = out;
    }

    *num = n;
}

static void detect_corner_indices_from_angle_cos(const float *angle_cos,
                                                 int angle_count,
                                                 float cos_threshold,
                                                 int nms_radius,
                                                 int *corner_indices,
                                                 int *corner_count)
{
    if (corner_count) *corner_count = 0;
    if (corner_indices == nullptr || corner_count == nullptr || angle_cos == nullptr || angle_count <= 0)
    {
        return;
    }

    const int n = std::clamp(angle_count, 0, VISION_BOUNDARY_NUM);
    std::fill_n(corner_indices, VISION_BOUNDARY_NUM, -1);
    if (n < 3)
    {
        return;
    }

    const int radius = std::max(1, nms_radius);
    const float th = std::clamp(cos_threshold, -1.0f, 1.0f);
    int out = 0;
    for (int i = 1; i + 1 < n && out < VISION_BOUNDARY_NUM; ++i)
    {
        const float c = angle_cos[i];
        if (!std::isfinite(c) || c > th)
        {
            continue;
        }
        // 局部极小值。
        if (!(c <= angle_cos[i - 1] && c <= angle_cos[i + 1]))
        {
            continue;
        }
        // NMS：在邻域内保留 cos 最小（更尖锐）的点。
        bool keep = true;
        const int l = std::max(1, i - radius);
        const int r = std::min(n - 2, i + radius);
        for (int j = l; j <= r; ++j)
        {
            if (j == i)
            {
                continue;
            }
            const float cj = angle_cos[j];
            if (!std::isfinite(cj))
            {
                continue;
            }
            if (cj < c || (cj == c && j < i))
            {
                keep = false;
                break;
            }
        }
        if (keep)
        {
            corner_indices[out++] = i;
        }
    }
    *corner_count = out;
}

static void build_corner_points_from_indices(const maze_point_t *ipm_pts,
                                             int ipm_num,
                                             const int *corner_indices,
                                             int corner_count,
                                             uint16 *ipm_x,
                                             uint16 *ipm_y,
                                             int *ipm_count,
                                             uint16 *src_x,
                                             uint16 *src_y,
                                             int *src_count)
{
    if (ipm_count) *ipm_count = 0;
    if (src_count) *src_count = 0;
    if (ipm_pts == nullptr || corner_indices == nullptr || ipm_x == nullptr || ipm_y == nullptr ||
        src_x == nullptr || src_y == nullptr || ipm_count == nullptr || src_count == nullptr)
    {
        return;
    }

    const int n = std::clamp(ipm_num, 0, VISION_BOUNDARY_NUM);
    const int m = std::clamp(corner_count, 0, VISION_BOUNDARY_NUM);
    int out_ipm = 0;
    int out_src = 0;
    for (int i = 0; i < m && out_ipm < VISION_BOUNDARY_NUM; ++i)
    {
        const int idx = corner_indices[i];
        if (idx < 0 || idx >= n)
        {
            continue;
        }
        const int px = std::clamp(ipm_pts[idx].x, 0, kIpmOutputWidth - 1);
        const int py = std::clamp(ipm_pts[idx].y, 0, kIpmOutputHeight - 1);
        ipm_x[out_ipm] = static_cast<uint16>(px);
        ipm_y[out_ipm] = static_cast<uint16>(py);
        ++out_ipm;

        int sx = 0;
        int sy = 0;
        if (out_src < VISION_BOUNDARY_NUM && ipm_point_to_src_point(px, py, &sx, &sy))
        {
            src_x[out_src] = static_cast<uint16>(std::clamp(sx, 0, kProcWidth - 1));
            src_y[out_src] = static_cast<uint16>(std::clamp(sy, 0, kProcHeight - 1));
            ++out_src;
        }
    }

    *ipm_count = out_ipm;
    *src_count = out_src;
}

static void truncate_by_first_corner_inplace(maze_point_t *proc_pts,
                                             int *proc_num,
                                             maze_point_t *angle_pts,
                                             int *angle_num,
                                             float *angle_cos,
                                             int *angle_cos_count,
                                             int *corner_indices,
                                             int *corner_count)
{
    if (proc_pts == nullptr || proc_num == nullptr || angle_pts == nullptr || angle_num == nullptr ||
        angle_cos == nullptr || angle_cos_count == nullptr || corner_indices == nullptr || corner_count == nullptr)
    {
        return;
    }
    if (*corner_count <= 0)
    {
        return;
    }

    const int n_proc = std::clamp(*proc_num, 0, VISION_BOUNDARY_NUM);
    const int n_ang = std::clamp(*angle_num, 0, VISION_BOUNDARY_NUM);
    const int n_cos = std::clamp(*angle_cos_count, 0, VISION_BOUNDARY_NUM);
    const int n = std::min(n_proc, std::min(n_ang, n_cos));
    if (n <= 0)
    {
        *proc_num = 0;
        *angle_num = 0;
        *angle_cos_count = 0;
        *corner_count = 0;
        return;
    }

    int first_idx = std::clamp(corner_indices[0], 0, n - 1);
    *proc_num = std::min(n_proc, first_idx + 1);
    *angle_num = std::min(n_ang, first_idx + 1);
    *angle_cos_count = std::min(n_cos, first_idx + 1);

    // 仅保留首个角点，后续点视为“截断后无效”。
    corner_indices[0] = first_idx;
    *corner_count = 1;
    for (int i = 1; i < VISION_BOUNDARY_NUM; ++i)
    {
        corner_indices[i] = -1;
    }
    for (int i = *angle_cos_count; i < VISION_BOUNDARY_NUM; ++i)
    {
        angle_cos[i] = 0.0f;
    }
}

static void postprocess_shifted_centerline_inplace(maze_point_t *pts, int *num, int width, int height)
{
    if (pts == nullptr || num == nullptr)
    {
        return;
    }

    if (!g_ipm_centerline_postprocess_enabled.load())
    {
        return;
    }

    // 偏移中线复用边界预处理：近点去重 + 小回跳毛刺抑制。
    static constexpr float kCenterlineMinPointDistPx = 1.4f;
    static constexpr float kCenterlineShortSpikeSegMaxPx = 2.0f;
    static constexpr float kCenterlineReverseCosThreshold = -0.2f;
    remove_near_duplicate_boundary_points_inplace(pts, num, kCenterlineMinPointDistPx);
    remove_backtrack_spikes_inplace(pts, num, kCenterlineShortSpikeSegMaxPx, kCenterlineReverseCosThreshold);

    if (g_ipm_centerline_triangle_filter_enabled.load())
    {
        triangle_filter_boundary_points_inplace(pts, *num, width, height);
    }

    if (g_ipm_centerline_resample_enabled.load())
    {
        resample_boundary_points_equal_spacing_inplace(pts,
                                                       num,
                                                       width,
                                                       height,
                                                       g_ipm_centerline_resample_step_px.load());
    }
}

static void resample_boundary_points_equal_spacing_inplace(maze_point_t *pts, int *num, int width, int height, float step_px)
{
    if (pts == nullptr || num == nullptr || *num <= 1 || width <= 0 || height <= 0)
    {
        return;
    }

    const float step = std::max(0.1f, step_px);
    const int in_num = std::clamp(*num, 0, VISION_BOUNDARY_NUM);
    if (in_num <= 1)
    {
        return;
    }

    std::array<maze_point_t, VISION_BOUNDARY_NUM> src{};
    for (int i = 0; i < in_num; ++i)
    {
        src[static_cast<size_t>(i)] = pts[i];
    }

    const int max_x = width - 1;
    const int max_y = height - 1;
    int out = 0;
    pts[out++] = src[0];

    float dist_acc = 0.0f;
    for (int i = 1; i < in_num && out < VISION_BOUNDARY_NUM; ++i)
    {
        const float ax = static_cast<float>(src[static_cast<size_t>(i - 1)].x);
        const float ay = static_cast<float>(src[static_cast<size_t>(i - 1)].y);
        const float bx = static_cast<float>(src[static_cast<size_t>(i)].x);
        const float by = static_cast<float>(src[static_cast<size_t>(i)].y);
        const float vx = bx - ax;
        const float vy = by - ay;
        const float seg_len = std::sqrt(vx * vx + vy * vy);
        if (seg_len < 1e-6f)
        {
            continue;
        }

        float used = 0.0f;
        while ((dist_acc + (seg_len - used)) >= step && out < VISION_BOUNDARY_NUM)
        {
            const float remain = step - dist_acc;
            const float t = (used + remain) / seg_len;
            const int px = std::clamp(static_cast<int>(std::lround(ax + vx * t)), 0, max_x);
            const int py = std::clamp(static_cast<int>(std::lround(ay + vy * t)), 0, max_y);
            pts[out++] = {px, py};
            used += remain;
            dist_acc = 0.0f;
        }
        dist_acc += (seg_len - used);
    }

    const maze_point_t &last = src[static_cast<size_t>(in_num - 1)];
    if (out < VISION_BOUNDARY_NUM)
    {
        const maze_point_t &tail = pts[out - 1];
        if (tail.x != last.x || tail.y != last.y)
        {
            pts[out++] = {std::clamp(last.x, 0, max_x), std::clamp(last.y, 0, max_y)};
        }
    }

    *num = out;
}

static int render_ipm_boundary_image_and_update_boundaries(const maze_point_t *left_pts,
                                                           int left_num,
                                                           const maze_point_t *right_pts,
                                                           int right_num,
                                                           const uint8 *classify_img,
                                                           uint8 classify_white_threshold,
                                                           int y_min,
                                                           int maze_trace_x_min,
                                                           int maze_trace_x_max,
                                                           int preferred_source)
{
    // 关闭整图逆透视彩色调试图，避免 warpPerspective + resize 带来的额外耗时。
    std::memset(g_image_ipm_bgr, 0, kIpmOutputWidth * kIpmOutputHeight * 3);
    const std::array<uint16, VISION_BOUNDARY_NUM> prev_ipm_shift_left_center_x = [&]() {
        std::array<uint16, VISION_BOUNDARY_NUM> out{};
        std::copy_n(g_ipm_shift_left_center_x, VISION_BOUNDARY_NUM, out.data());
        return out;
    }();
    const std::array<uint16, VISION_BOUNDARY_NUM> prev_ipm_shift_left_center_y = [&]() {
        std::array<uint16, VISION_BOUNDARY_NUM> out{};
        std::copy_n(g_ipm_shift_left_center_y, VISION_BOUNDARY_NUM, out.data());
        return out;
    }();
    const int prev_ipm_shift_left_center_count = g_ipm_shift_left_center_count;
    const std::array<uint16, VISION_BOUNDARY_NUM> prev_ipm_shift_right_center_x = [&]() {
        std::array<uint16, VISION_BOUNDARY_NUM> out{};
        std::copy_n(g_ipm_shift_right_center_x, VISION_BOUNDARY_NUM, out.data());
        return out;
    }();
    const std::array<uint16, VISION_BOUNDARY_NUM> prev_ipm_shift_right_center_y = [&]() {
        std::array<uint16, VISION_BOUNDARY_NUM> out{};
        std::copy_n(g_ipm_shift_right_center_y, VISION_BOUNDARY_NUM, out.data());
        return out;
    }();
    const int prev_ipm_shift_right_center_count = g_ipm_shift_right_center_count;
    const std::array<uint16, VISION_BOUNDARY_NUM> prev_src_shift_left_center_x = [&]() {
        std::array<uint16, VISION_BOUNDARY_NUM> out{};
        std::copy_n(g_src_shift_left_center_x, VISION_BOUNDARY_NUM, out.data());
        return out;
    }();
    const std::array<uint16, VISION_BOUNDARY_NUM> prev_src_shift_left_center_y = [&]() {
        std::array<uint16, VISION_BOUNDARY_NUM> out{};
        std::copy_n(g_src_shift_left_center_y, VISION_BOUNDARY_NUM, out.data());
        return out;
    }();
    const int prev_src_shift_left_center_count = g_src_shift_left_center_count;
    const std::array<uint16, VISION_BOUNDARY_NUM> prev_src_shift_right_center_x = [&]() {
        std::array<uint16, VISION_BOUNDARY_NUM> out{};
        std::copy_n(g_src_shift_right_center_x, VISION_BOUNDARY_NUM, out.data());
        return out;
    }();
    const std::array<uint16, VISION_BOUNDARY_NUM> prev_src_shift_right_center_y = [&]() {
        std::array<uint16, VISION_BOUNDARY_NUM> out{};
        std::copy_n(g_src_shift_right_center_y, VISION_BOUNDARY_NUM, out.data());
        return out;
    }();
    const int prev_src_shift_right_center_count = g_src_shift_right_center_count;
    clear_ipm_saved_arrays();

    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_ipm{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_ipm{};
    const int left_ipm_num = transform_boundary_points_to_ipm(left_pts, left_num, left_ipm.data(), static_cast<int>(left_ipm.size()));
    const int right_ipm_num = transform_boundary_points_to_ipm(right_pts, right_num, right_ipm.data(), static_cast<int>(right_ipm.size()));

    // 共同预处理：近点去重 -> 小回跳毛刺抑制 -> 等弧长重采样。
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_proc{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_proc{};
    left_proc = left_ipm;
    right_proc = right_ipm;
    int left_proc_num = left_ipm_num;
    int right_proc_num = right_ipm_num;
    const float kBoundaryMinPointDistPx = std::max(0.0f, g_ipm_boundary_min_point_dist_px.load());
    const float kBoundaryShortSpikeSegMaxPx = std::max(0.0f, g_ipm_boundary_spike_short_seg_max_px.load());
    const float kBoundaryReverseCosThreshold = std::clamp(g_ipm_boundary_spike_reverse_cos_threshold.load(), -1.0f, 1.0f);
    remove_near_duplicate_boundary_points_inplace(left_proc.data(), &left_proc_num, kBoundaryMinPointDistPx);
    remove_near_duplicate_boundary_points_inplace(right_proc.data(), &right_proc_num, kBoundaryMinPointDistPx);
    remove_backtrack_spikes_inplace(left_proc.data(), &left_proc_num, kBoundaryShortSpikeSegMaxPx, kBoundaryReverseCosThreshold);
    remove_backtrack_spikes_inplace(right_proc.data(), &right_proc_num, kBoundaryShortSpikeSegMaxPx, kBoundaryReverseCosThreshold);
    if (g_ipm_resample_enabled.load())
    {
        resample_boundary_points_equal_spacing_inplace(left_proc.data(),
                                                       &left_proc_num,
                                                       kIpmOutputWidth,
                                                       kIpmOutputHeight,
                                                       g_ipm_resample_step_px.load());
        resample_boundary_points_equal_spacing_inplace(right_proc.data(),
                                                       &right_proc_num,
                                                       kIpmOutputWidth,
                                                       kIpmOutputHeight,
                                                       g_ipm_resample_step_px.load());
    }

    // 直角点支路：弱平滑（1 次三角）后计算三点夹角 cos，并做局部极小值 + NMS。
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_angle{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_angle{};
    left_angle = left_proc;
    right_angle = right_proc;
    int left_angle_num = left_proc_num;
    int right_angle_num = right_proc_num;
    if (g_ipm_triangle_filter_enabled.load())
    {
        triangle_filter_boundary_points_inplace(left_angle.data(), left_angle_num, kIpmOutputWidth, kIpmOutputHeight);
        triangle_filter_boundary_points_inplace(right_angle.data(), right_angle_num, kIpmOutputWidth, kIpmOutputHeight);
    }
    const int kBoundaryAngleStep = g_ipm_boundary_angle_step.load();
    compute_boundary_angle_cos_3point_inplace(left_angle.data(),
                                              left_angle_num,
                                              kBoundaryAngleStep,
                                              g_ipm_left_boundary_angle_cos,
                                              &g_ipm_left_boundary_angle_cos_count);
    compute_boundary_angle_cos_3point_inplace(right_angle.data(),
                                              right_angle_num,
                                              kBoundaryAngleStep,
                                              g_ipm_right_boundary_angle_cos,
                                              &g_ipm_right_boundary_angle_cos_count);
    const float kCornerCosThreshold = std::clamp(g_ipm_boundary_corner_cos_threshold.load(), -1.0f, 1.0f);
    const int kCornerNmsRadius = std::max(1, g_ipm_boundary_corner_nms_radius.load());
    detect_corner_indices_from_angle_cos(g_ipm_left_boundary_angle_cos,
                                         g_ipm_left_boundary_angle_cos_count,
                                         kCornerCosThreshold,
                                         kCornerNmsRadius,
                                         g_ipm_left_boundary_corner_indices,
                                         &g_ipm_left_boundary_corner_count);
    detect_corner_indices_from_angle_cos(g_ipm_right_boundary_angle_cos,
                                         g_ipm_right_boundary_angle_cos_count,
                                         kCornerCosThreshold,
                                         kCornerNmsRadius,
                                         g_ipm_right_boundary_corner_indices,
                                         &g_ipm_right_boundary_corner_count);

    // 若识别到直角角点，按首个角点截断其后边界，再进入曲率与中线计算。
    truncate_by_first_corner_inplace(left_proc.data(),
                                     &left_proc_num,
                                     left_angle.data(),
                                     &left_angle_num,
                                     g_ipm_left_boundary_angle_cos,
                                     &g_ipm_left_boundary_angle_cos_count,
                                     g_ipm_left_boundary_corner_indices,
                                     &g_ipm_left_boundary_corner_count);
    truncate_by_first_corner_inplace(right_proc.data(),
                                     &right_proc_num,
                                     right_angle.data(),
                                     &right_angle_num,
                                     g_ipm_right_boundary_angle_cos,
                                     &g_ipm_right_boundary_angle_cos_count,
                                     g_ipm_right_boundary_corner_indices,
                                     &g_ipm_right_boundary_corner_count);

    build_corner_points_from_indices(left_angle.data(),
                                     left_angle_num,
                                     g_ipm_left_boundary_corner_indices,
                                     g_ipm_left_boundary_corner_count,
                                     g_ipm_left_boundary_corner_x,
                                     g_ipm_left_boundary_corner_y,
                                     &g_ipm_left_boundary_corner_point_count,
                                     g_src_left_boundary_corner_x,
                                     g_src_left_boundary_corner_y,
                                     &g_src_left_boundary_corner_point_count);
    build_corner_points_from_indices(right_angle.data(),
                                     right_angle_num,
                                     g_ipm_right_boundary_corner_indices,
                                     g_ipm_right_boundary_corner_count,
                                     g_ipm_right_boundary_corner_x,
                                     g_ipm_right_boundary_corner_y,
                                     &g_ipm_right_boundary_corner_point_count,
                                     g_src_right_boundary_corner_x,
                                     g_src_right_boundary_corner_y,
                                     &g_src_right_boundary_corner_point_count);

    // 角点触发辅助线：在曲率与中线前处理。
    trace_auxiliary_lines_from_corners(classify_img,
                                       classify_white_threshold,
                                       y_min,
                                       maze_trace_x_min,
                                       maze_trace_x_max);

    auto merge_aux_line_if_has_corner = [&](bool is_left_side,
                                            maze_point_t *main_proc,
                                            int *main_num) {
        if (main_proc == nullptr || main_num == nullptr)
        {
            return;
        }

        uint16 *aux_src_x = is_left_side ? g_xy_aux_left_x : g_xy_aux_right_x;
        uint16 *aux_src_y = is_left_side ? g_xy_aux_left_y : g_xy_aux_right_y;
        int aux_src_num = is_left_side ? g_xy_aux_left_count : g_xy_aux_right_count;
        if (aux_src_num <= 0)
        {
            return;
        }

        std::array<maze_point_t, VISION_BOUNDARY_NUM> aux_src_pts{};
        const int src_n = std::clamp(aux_src_num, 0, VISION_BOUNDARY_NUM);
        for (int i = 0; i < src_n; ++i)
        {
            aux_src_pts[i].x = std::clamp(static_cast<int>(aux_src_x[i]), 0, kProcWidth - 1);
            aux_src_pts[i].y = std::clamp(static_cast<int>(aux_src_y[i]), 0, kProcHeight - 1);
        }

        std::array<maze_point_t, VISION_BOUNDARY_NUM> aux_ipm{};
        int aux_ipm_num = transform_boundary_points_to_ipm(aux_src_pts.data(), src_n, aux_ipm.data(), static_cast<int>(aux_ipm.size()));
        if (aux_ipm_num <= 0)
        {
            return;
        }

        // 辅助线按边界同样预处理（不做曲率）。
        static constexpr float kBoundaryMinPointDistPx = 1.4f;
        static constexpr float kBoundaryShortSpikeSegMaxPx = 2.0f;
        static constexpr float kBoundaryReverseCosThreshold = -0.2f;
        remove_near_duplicate_boundary_points_inplace(aux_ipm.data(), &aux_ipm_num, kBoundaryMinPointDistPx);
        remove_backtrack_spikes_inplace(aux_ipm.data(), &aux_ipm_num, kBoundaryShortSpikeSegMaxPx, kBoundaryReverseCosThreshold);
        if (g_ipm_resample_enabled.load())
        {
            resample_boundary_points_equal_spacing_inplace(aux_ipm.data(),
                                                           &aux_ipm_num,
                                                           kIpmOutputWidth,
                                                           kIpmOutputHeight,
                                                           g_ipm_resample_step_px.load());
        }
        if (aux_ipm_num <= 0)
        {
            return;
        }

        std::array<maze_point_t, VISION_BOUNDARY_NUM> aux_angle = aux_ipm;
        int aux_angle_num = aux_ipm_num;
        if (g_ipm_triangle_filter_enabled.load())
        {
            triangle_filter_boundary_points_inplace(aux_angle.data(), aux_angle_num, kIpmOutputWidth, kIpmOutputHeight);
        }

        std::array<float, VISION_BOUNDARY_NUM> aux_cos{};
        int aux_cos_count = 0;
        std::array<int, VISION_BOUNDARY_NUM> aux_corner_idx{};
        int aux_corner_count = 0;
        const int kBoundaryAngleStep = g_ipm_boundary_angle_step.load();
        static constexpr float kCornerCosThreshold = 0.55f;
        static constexpr int kCornerNmsRadius = 3;
        compute_boundary_angle_cos_3point_inplace(aux_angle.data(),
                                                  aux_angle_num,
                                                  kBoundaryAngleStep,
                                                  aux_cos.data(),
                                                  &aux_cos_count);
        detect_corner_indices_from_angle_cos(aux_cos.data(),
                                             aux_cos_count,
                                             kCornerCosThreshold,
                                             kCornerNmsRadius,
                                             aux_corner_idx.data(),
                                             &aux_corner_count);
        if (aux_corner_count <= 0)
        {
            return;
        }

        // 保留辅助线角点及其后段（删除角点前段）。
        const int keep_from = std::clamp(aux_corner_idx[0], 0, aux_ipm_num - 1);
        const int keep_num = std::max(0, aux_ipm_num - keep_from);
        if (keep_num <= 0)
        {
            return;
        }
        std::array<maze_point_t, VISION_BOUNDARY_NUM> aux_tail{};
        for (int i = 0; i < keep_num && i < VISION_BOUNDARY_NUM; ++i)
        {
            aux_tail[i] = aux_ipm[keep_from + i];
        }

        // 将辅助线拼接到该侧主边界后，连接段按 1.4px 间隔补点。
        append_points_with_bridge_inplace(main_proc,
                                          main_num,
                                          aux_tail.data(),
                                          keep_num,
                                          1.4f,
                                          kIpmOutputWidth,
                                          kIpmOutputHeight);
    };

    merge_aux_line_if_has_corner(true, left_proc.data(), &left_proc_num);
    merge_aux_line_if_has_corner(false, right_proc.data(), &right_proc_num);

    // 弯道支路：i±5（跨度10点）曲率 + 三点平滑。
    const float kBoundarySampleSpacingCm = g_ipm_boundary_kappa_sample_spacing_cm.load();
    compute_span10_boundary_curvature_inplace(left_proc.data(),
                                              left_proc_num,
                                              kBoundarySampleSpacingCm,
                                              g_ipm_left_boundary_sg_curvature,
                                              &g_ipm_left_boundary_sg_curvature_count);
    compute_span10_boundary_curvature_inplace(right_proc.data(),
                                              right_proc_num,
                                              kBoundarySampleSpacingCm,
                                              g_ipm_right_boundary_sg_curvature,
                                              &g_ipm_right_boundary_sg_curvature_count);

    // 处理链边界法向平移中线：
    // 先看偏好源，再按左右边界点数择优，只计算并保留一条平移中线。
    const bool selected_is_right = select_shift_source_from_preference_and_counts(preferred_source,
                                                                                   left_proc_num,
                                                                                   right_proc_num);
    const float shift_dist_px = g_ipm_boundary_shift_distance_px.load();
    std::array<maze_point_t, VISION_BOUNDARY_NUM> center_selected{};
    int center_selected_num = 0;
    if (selected_is_right)
    {
        shift_boundary_along_normal(right_proc.data(),
                                    right_proc_num,
                                    left_proc.data(),
                                    left_proc_num,
                                    center_selected.data(),
                                    &center_selected_num,
                                    kIpmOutputWidth,
                                    kIpmOutputHeight,
                                    shift_dist_px,
                                    false);
    }
    else
    {
        shift_boundary_along_normal(left_proc.data(),
                                    left_proc_num,
                                    right_proc.data(),
                                    right_proc_num,
                                    center_selected.data(),
                                    &center_selected_num,
                                    kIpmOutputWidth,
                                    kIpmOutputHeight,
                                    shift_dist_px,
                                    true);
    }

    // 中线独立后处理：可选三角滤波、可选等距采样。
    postprocess_shifted_centerline_inplace(center_selected.data(),
                                           &center_selected_num,
                                           kIpmOutputWidth,
                                           kIpmOutputHeight);

    std::array<maze_point_t, VISION_BOUNDARY_NUM> src_center_selected{};
    const int src_center_selected_num = transform_boundary_points_from_ipm_to_src(center_selected.data(),
                                                                                  center_selected_num,
                                                                                  src_center_selected.data(),
                                                                                  static_cast<int>(src_center_selected.size()));

    if (selected_is_right)
    {
        fill_single_line_arrays_from_points(center_selected.data(),
                                            center_selected_num,
                                            g_ipm_shift_right_center_x,
                                            g_ipm_shift_right_center_y,
                                            &g_ipm_shift_right_center_count,
                                            kIpmOutputWidth,
                                            kIpmOutputHeight);
        fill_single_line_arrays_from_points(src_center_selected.data(),
                                            src_center_selected_num,
                                            g_src_shift_right_center_x,
                                            g_src_shift_right_center_y,
                                            &g_src_shift_right_center_count,
                                            kProcWidth,
                                            kProcHeight);
    }
    else
    {
        fill_single_line_arrays_from_points(center_selected.data(),
                                            center_selected_num,
                                            g_ipm_shift_left_center_x,
                                            g_ipm_shift_left_center_y,
                                            &g_ipm_shift_left_center_count,
                                            kIpmOutputWidth,
                                            kIpmOutputHeight);
        fill_single_line_arrays_from_points(src_center_selected.data(),
                                            src_center_selected_num,
                                            g_src_shift_left_center_x,
                                            g_src_shift_left_center_y,
                                            &g_src_shift_left_center_count,
                                            kProcWidth,
                                            kProcHeight);
    }
    vision_line_error_layer_set_source(selected_is_right ? static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT)
                                                         : static_cast<int>(VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT));

    if (left_num <= 0 && right_num <= 0 && g_keep_last_centerline_on_double_loss.load())
    {
        std::copy_n(prev_ipm_shift_left_center_x.data(), VISION_BOUNDARY_NUM, g_ipm_shift_left_center_x);
        std::copy_n(prev_ipm_shift_left_center_y.data(), VISION_BOUNDARY_NUM, g_ipm_shift_left_center_y);
        g_ipm_shift_left_center_count = prev_ipm_shift_left_center_count;
        std::copy_n(prev_ipm_shift_right_center_x.data(), VISION_BOUNDARY_NUM, g_ipm_shift_right_center_x);
        std::copy_n(prev_ipm_shift_right_center_y.data(), VISION_BOUNDARY_NUM, g_ipm_shift_right_center_y);
        g_ipm_shift_right_center_count = prev_ipm_shift_right_center_count;
        std::copy_n(prev_src_shift_left_center_x.data(), VISION_BOUNDARY_NUM, g_src_shift_left_center_x);
        std::copy_n(prev_src_shift_left_center_y.data(), VISION_BOUNDARY_NUM, g_src_shift_left_center_y);
        g_src_shift_left_center_count = prev_src_shift_left_center_count;
        std::copy_n(prev_src_shift_right_center_x.data(), VISION_BOUNDARY_NUM, g_src_shift_right_center_x);
        std::copy_n(prev_src_shift_right_center_y.data(), VISION_BOUNDARY_NUM, g_src_shift_right_center_y);
        g_src_shift_right_center_count = prev_src_shift_right_center_count;
    }

    fill_boundary_arrays_from_points_to_target(left_proc.data(),
                                               left_proc_num,
                                               right_proc.data(),
                                               right_proc_num,
                                               g_ipm_xy_x1_boundary,
                                               g_ipm_xy_x2_boundary,
                                               g_ipm_xy_x3_boundary,
                                               g_ipm_xy_y1_boundary,
                                               g_ipm_xy_y2_boundary,
                                               g_ipm_xy_y3_boundary,
                                               &g_ipm_boundary_count,
                                               kIpmOutputWidth,
                                               kIpmOutputHeight);
    g_ipm_boundary_left_count = std::clamp(left_proc_num, 0, VISION_BOUNDARY_NUM);
    g_ipm_boundary_right_count = std::clamp(right_proc_num, 0, VISION_BOUNDARY_NUM);
    return 0;
}

static bool find_maze_start_from_row(const uint8 *classify_img,
                                     uint8 white_threshold,
                                     bool search_left,
                                     int search_y,
                                     int x_min,
                                     int x_max,
                                     int *start_x,
                                     int *start_y,
                                     bool *wall_is_white,
                                     int preferred_center_x = -1)
{
    if (classify_img == nullptr || start_x == nullptr || start_y == nullptr || wall_is_white == nullptr)
    {
        return false;
    }

    const int width = kProcWidth;
    const int height = kProcHeight;
    if (width < 4 || height < 3)
    {
        return false;
    }

    const int center_x = std::clamp((preferred_center_x >= x_min && preferred_center_x <= x_max) ? preferred_center_x : (width / 2),
                                    x_min,
                                    x_max);
    const int scan_y = std::clamp(search_y, 1, height - 2);
    if (x_min < 1 || x_max > width - 2 || x_min >= x_max)
    {
        return false;
    }

    int x_found = -1;
    bool path_is_white = false;
    bool wall_is_white_local = false;
    if (search_left)
    {
        for (int x = center_x; x >= x_min + 1; --x)
        {
            bool inner_white = pixel_is_white(classify_img, x, scan_y, white_threshold);
            bool outer_white = pixel_is_white(classify_img, x - 1, scan_y, white_threshold);
            if (inner_white != outer_white)
            {
                x_found = x; // 靠近图像中心的一侧
                path_is_white = inner_white;
                wall_is_white_local = outer_white;
                break;
            }
        }
    }
    else
    {
        for (int x = center_x; x <= x_max - 1; ++x)
        {
            bool inner_white = pixel_is_white(classify_img, x, scan_y, white_threshold);
            bool outer_white = pixel_is_white(classify_img, x + 1, scan_y, white_threshold);
            if (inner_white != outer_white)
            {
                x_found = x; // 靠近图像中心的一侧
                path_is_white = inner_white;
                wall_is_white_local = outer_white;
                break;
            }
        }
    }

    if (x_found < x_min || x_found > x_max)
    {
        return false;
    }

    // 迷宫法直接从指定行开始。
    const int track_y = scan_y;

    // 如果起点误落在墙侧，向图像中心方向微调到路径侧。
    int x_adjust = x_found;
    int center_step = search_left ? 1 : -1;
    for (int i = 0; i < 8; ++i)
    {
        bool cur_white = pixel_is_white(classify_img, x_adjust, track_y, white_threshold);
        if (cur_white == path_is_white)
        {
            break;
        }

        int next_x = x_adjust + center_step;
        if (next_x < x_min || next_x > x_max)
        {
            break;
        }
        x_adjust = next_x;
    }

    *start_x = x_adjust;
    *start_y = track_y;
    *wall_is_white = wall_is_white_local;
    return true;
}

static void validate_maze_start_pair(const uint8 *classify_img,
                                     uint8 white_threshold,
                                     int x_min,
                                     int x_max,
                                     bool *left_ok,
                                     int left_start_x,
                                     int left_start_y,
                                     bool *right_ok,
                                     int right_start_x,
                                     int right_start_y)
{
    (void)classify_img;
    (void)white_threshold;
    if (left_ok == nullptr || right_ok == nullptr)
    {
        return;
    }

    if (!(*left_ok) || !(*right_ok))
    {
        return;
    }

    if (right_start_x <= left_start_x || (right_start_x - left_start_x) <= kMazeStartMinBoundaryGapPx)
    {
        *left_ok = false;
        *right_ok = false;
    }
}

static bool single_boundary_side_vote_is_current_side(const uint8 *classify_img,
                                                       uint8 white_threshold,
                                                       int start_x,
                                                       int start_y,
                                                       bool sample_left_side)
{
    if (classify_img == nullptr)
    {
        return true;
    }

    const int y = std::clamp(start_y, 1, kProcHeight - 2);
    int white_count = 0;
    int black_count = 0;
    for (int i = 1; i <= 5; ++i)
    {
        const int x = start_x + (sample_left_side ? -i : i);
        if (x < 1 || x > (kProcWidth - 2))
        {
            continue;
        }
        if (pixel_is_white(classify_img, x, y, white_threshold))
        {
            ++white_count;
        }
        else
        {
            ++black_count;
        }
    }

    if ((white_count + black_count) <= 0)
    {
        return true;
    }

    // 规则：白点多/相等 -> 当前边界身份正确；黑点多 -> 应判为对侧边界。
    return white_count >= black_count;
}

static int maze_trace_left_hand(const uint8 *classify_img,
                                uint8 white_threshold,
                                bool wall_is_white,
                                int start_x,
                                int start_y,
                                int y_min,
                                int x_min,
                                int x_max,
                                maze_point_t *pts,
                                int max_pts)
{
    if (classify_img == nullptr || pts == nullptr || max_pts <= 0 || start_x <= x_min || start_x >= x_max)
    {
        return 0;
    }

    int x = start_x;
    int y = start_y;
    int min_y_seen = start_y;
    const int y_fallback_stop_delta = std::max(0, g_maze_trace_y_fallback_stop_delta.load());
    int dir = 0;
    int turn = 0;
    int step = 0;
    while (step < max_pts &&
           x > 0 && x < kProcWidth - 1 &&
           y > 0 && y < kProcHeight - 1 &&
           y >= y_min &&
           turn < 4)
    {

        int fx = x + kDirFront[dir][0];
        int fy = y + kDirFront[dir][1];
        int flx = x + kDirFrontLeft[dir][0];
        int fly = y + kDirFrontLeft[dir][1];

        bool front_is_wall = pixel_is_wall(classify_img, fx, fy, white_threshold, wall_is_white);
        bool frontleft_is_wall = pixel_is_wall(classify_img, flx, fly, white_threshold, wall_is_white);

        if (front_is_wall)
        {
            dir = (dir + 1) % 4;
            turn++;
        }
        else if (frontleft_is_wall)
        {
            if (fx <= x_min || fx >= x_max)
            {
                break;
            }
            x = fx;
            y = fy;
            min_y_seen = std::min(min_y_seen, y);
            if (y > min_y_seen + y_fallback_stop_delta)
            {
                break;
            }
            pts[step++] = {x, y};
            turn = 0;
        }
        else
        {
            if (flx <= x_min || flx >= x_max)
            {
                break;
            }
            x = flx;
            y = fly;
            dir = (dir + 3) % 4;
            min_y_seen = std::min(min_y_seen, y);
            if (y > min_y_seen + y_fallback_stop_delta)
            {
                break;
            }
            pts[step++] = {x, y};
            turn = 0;
        }
    }

    return step;
}

static int maze_trace_right_hand(const uint8 *classify_img,
                                 uint8 white_threshold,
                                 bool wall_is_white,
                                 int start_x,
                                 int start_y,
                                 int y_min,
                                 int x_min,
                                 int x_max,
                                 maze_point_t *pts,
                                 int max_pts)
{
    if (classify_img == nullptr || pts == nullptr || max_pts <= 0 || start_x <= x_min || start_x >= x_max)
    {
        return 0;
    }

    int x = start_x;
    int y = start_y;
    int min_y_seen = start_y;
    const int y_fallback_stop_delta = std::max(0, g_maze_trace_y_fallback_stop_delta.load());
    int dir = 0;
    int turn = 0;
    int step = 0;
    while (step < max_pts &&
           x > 0 && x < kProcWidth - 1 &&
           y > 0 && y < kProcHeight - 1 &&
           y >= y_min &&
           turn < 4)
    {

        int fx = x + kDirFront[dir][0];
        int fy = y + kDirFront[dir][1];
        int frx = x + kDirFrontRight[dir][0];
        int fry = y + kDirFrontRight[dir][1];

        bool front_is_wall = pixel_is_wall(classify_img, fx, fy, white_threshold, wall_is_white);
        bool frontright_is_wall = pixel_is_wall(classify_img, frx, fry, white_threshold, wall_is_white);

        if (front_is_wall)
        {
            dir = (dir + 3) % 4;
            turn++;
        }
        else if (frontright_is_wall)
        {
            if (fx <= x_min || fx >= x_max)
            {
                break;
            }
            x = fx;
            y = fy;
            min_y_seen = std::min(min_y_seen, y);
            if (y > min_y_seen + y_fallback_stop_delta)
            {
                break;
            }
            pts[step++] = {x, y};
            turn = 0;
        }
        else
        {
            if (frx <= x_min || frx >= x_max)
            {
                break;
            }
            x = frx;
            y = fry;
            dir = (dir + 1) % 4;
            min_y_seen = std::min(min_y_seen, y);
            if (y > min_y_seen + y_fallback_stop_delta)
            {
                break;
            }
            pts[step++] = {x, y};
            turn = 0;
        }
    }

    return step;
}

static bool build_aux_start_from_corner(const uint8 *classify_img,
                                        uint8 white_threshold,
                                        int corner_x,
                                        int corner_y,
                                        bool is_right_side,
                                        int y_min,
                                        int x_min,
                                        int x_max,
                                        int *seed_x,
                                        int *seed_y,
                                        int *start_x,
                                        int *start_y,
                                        bool *wall_is_white)
{
    if (classify_img == nullptr || seed_x == nullptr || seed_y == nullptr ||
        start_x == nullptr || start_y == nullptr || wall_is_white == nullptr)
    {
        return false;
    }

    const int offset_mag_x = std::max(0, g_ipm_aux_seed_offset_x.load());
    const int offset_y = std::max(0, g_ipm_aux_seed_offset_y.load());
    const int min_white_count = std::max(1, g_ipm_aux_vertical_min_white_count.load());
    const int offset_x = is_right_side ? offset_mag_x : -offset_mag_x;
    const int sx = std::clamp(corner_x + offset_x, x_min, x_max);
    const int sy = std::clamp(corner_y - offset_y, y_min, kProcHeight - 2);
    *seed_x = sx;
    *seed_y = sy;

    int white_count = 0;
    for (int y = sy; y >= y_min; --y)
    {
        const bool is_white = pixel_is_white(classify_img, sx, y, white_threshold);
        if (is_white)
        {
            ++white_count;
            continue;
        }

        if (white_count < min_white_count)
        {
            return false;
        }

        *start_x = sx;
        *start_y = y;
        *wall_is_white = pixel_is_white(classify_img, sx, y, white_threshold);
        return true;
    }

    return false;
}

static void trace_auxiliary_lines_from_corners(const uint8 *classify_img,
                                               uint8 white_threshold,
                                               int y_min,
                                               int x_min,
                                               int x_max)
{
    g_xy_aux_left_count = 0;
    g_xy_aux_right_count = 0;
    g_xy_aux_left_seed_count = 0;
    g_xy_aux_right_seed_count = 0;

    if (classify_img == nullptr)
    {
        return;
    }

    auto trace_one_side = [&](bool is_right_side,
                              const uint16 *corner_xs,
                              const uint16 *corner_ys,
                              int corner_num,
                              uint16 *aux_x,
                              uint16 *aux_y,
                              int *aux_count,
                              uint16 *seed_out_x,
                              uint16 *seed_out_y,
                              int *seed_count) {
        if (corner_xs == nullptr || corner_ys == nullptr || aux_x == nullptr || aux_y == nullptr ||
            aux_count == nullptr || seed_out_x == nullptr || seed_out_y == nullptr || seed_count == nullptr ||
            corner_num <= 0)
        {
            return;
        }

        const int corner_x = std::clamp(static_cast<int>(corner_xs[0]), x_min, x_max);
        const int corner_y = std::clamp(static_cast<int>(corner_ys[0]), y_min, kProcHeight - 2);
        int seed_x = 0;
        int seed_y = 0;
        int start_x = 0;
        int start_y = 0;
        bool wall_white = false;
        if (!build_aux_start_from_corner(classify_img,
                                         white_threshold,
                                         corner_x,
                                         corner_y,
                                         is_right_side,
                                         y_min,
                                         x_min,
                                         x_max,
                                         &seed_x,
                                         &seed_y,
                                         &start_x,
                                         &start_y,
                                         &wall_white))
        {
            return;
        }

        seed_out_x[0] = static_cast<uint16>(seed_x);
        seed_out_y[0] = static_cast<uint16>(seed_y);
        *seed_count = 1;

        std::array<maze_point_t, VISION_BOUNDARY_NUM> aux_pts{};
        const int max_trace = std::max(1, g_ipm_aux_trace_max_points.load());
        const int max_pts = std::min(static_cast<int>(aux_pts.size()), max_trace);
        const int n = is_right_side
                          ? maze_trace_right_hand(classify_img,
                                                  white_threshold,
                                                  wall_white,
                                                  start_x,
                                                  start_y,
                                                  y_min,
                                                  x_min,
                                                  x_max,
                                                  aux_pts.data(),
                                                  max_pts)
                          : maze_trace_left_hand(classify_img,
                                                 white_threshold,
                                                 wall_white,
                                                 start_x,
                                                 start_y,
                                                 y_min,
                                                 x_min,
                                                 x_max,
                                                 aux_pts.data(),
                                                 max_pts);
        fill_single_line_arrays_from_points(aux_pts.data(),
                                            n,
                                            aux_x,
                                            aux_y,
                                            aux_count,
                                            kProcWidth,
                                            kProcHeight);
    };

    trace_one_side(false,
                   g_src_left_boundary_corner_x,
                   g_src_left_boundary_corner_y,
                   g_src_left_boundary_corner_point_count,
                   g_xy_aux_left_x,
                   g_xy_aux_left_y,
                   &g_xy_aux_left_count,
                   g_xy_aux_left_seed_x,
                   g_xy_aux_left_seed_y,
                   &g_xy_aux_left_seed_count);
    trace_one_side(true,
                   g_src_right_boundary_corner_x,
                   g_src_right_boundary_corner_y,
                   g_src_right_boundary_corner_point_count,
                   g_xy_aux_right_x,
                   g_xy_aux_right_y,
                   &g_xy_aux_right_count,
                   g_xy_aux_right_seed_x,
                   g_xy_aux_right_seed_y,
                   &g_xy_aux_right_seed_count);
}

static void append_points_with_bridge_inplace(maze_point_t *base_pts,
                                              int *base_num,
                                              const maze_point_t *extra_pts,
                                              int extra_num,
                                              float bridge_step_px,
                                              int width,
                                              int height)
{
    if (base_pts == nullptr || base_num == nullptr || extra_pts == nullptr || extra_num <= 0 || width <= 0 || height <= 0)
    {
        return;
    }

    int out = std::clamp(*base_num, 0, VISION_BOUNDARY_NUM);
    const int in_extra = std::clamp(extra_num, 0, VISION_BOUNDARY_NUM);
    const int max_x = width - 1;
    const int max_y = height - 1;
    const float step = std::max(0.1f, bridge_step_px);

    if (out <= 0)
    {
        for (int i = 0; i < in_extra && i < VISION_BOUNDARY_NUM; ++i)
        {
            base_pts[i].x = std::clamp(extra_pts[i].x, 0, max_x);
            base_pts[i].y = std::clamp(extra_pts[i].y, 0, max_y);
        }
        *base_num = std::min(in_extra, VISION_BOUNDARY_NUM);
        return;
    }

    const maze_point_t tail = base_pts[out - 1];
    const maze_point_t head = extra_pts[0];
    const float ax = static_cast<float>(tail.x);
    const float ay = static_cast<float>(tail.y);
    const float bx = static_cast<float>(head.x);
    const float by = static_cast<float>(head.y);
    const float vx = bx - ax;
    const float vy = by - ay;
    const float seg_len = std::sqrt(vx * vx + vy * vy);
    if (seg_len > 1e-6f)
    {
        float d = step;
        while (d < seg_len && out < VISION_BOUNDARY_NUM)
        {
            const float t = d / seg_len;
            const int px = std::clamp(static_cast<int>(std::lround(ax + vx * t)), 0, max_x);
            const int py = std::clamp(static_cast<int>(std::lround(ay + vy * t)), 0, max_y);
            base_pts[out++] = {px, py};
            d += step;
        }
    }

    for (int i = 0; i < in_extra && out < VISION_BOUNDARY_NUM; ++i)
    {
        base_pts[out++] = {std::clamp(extra_pts[i].x, 0, max_x), std::clamp(extra_pts[i].y, 0, max_y)};
    }
    *base_num = out;
}

static void fill_boundary_arrays_from_maze(const maze_point_t *left_pts,
                                           int left_num,
                                           const maze_point_t *right_pts,
                                           int right_num)
{
    clear_boundary_arrays();
    if ((left_pts == nullptr && right_pts == nullptr) || (left_num <= 0 && right_num <= 0))
    {
        return;
    }

    int out_num = std::max(left_num, right_num);
    if (out_num > VISION_BOUNDARY_NUM)
    {
        out_num = VISION_BOUNDARY_NUM;
    }

    for (int i = 0; i < out_num; ++i)
    {
        int li = 0;
        int ri = 0;
        if (left_num > 0)
        {
            li = (i < left_num) ? i : (left_num - 1);
        }
        if (right_num > 0)
        {
            ri = (i < right_num) ? i : (right_num - 1);
        }

        int lx = (left_num > 0) ? left_pts[li].x : ((right_num > 0) ? right_pts[ri].x : 0);
        int ly = (left_num > 0) ? left_pts[li].y : ((right_num > 0) ? right_pts[ri].y : 0);
        int rx = (right_num > 0) ? right_pts[ri].x : ((left_num > 0) ? left_pts[li].x : 0);
        int ry = (right_num > 0) ? right_pts[ri].y : ((left_num > 0) ? left_pts[li].y : 0);
        lx = std::clamp(lx, 0, kProcWidth - 1);
        ly = std::clamp(ly, 0, kProcHeight - 1);
        rx = std::clamp(rx, 0, kProcWidth - 1);
        ry = std::clamp(ry, 0, kProcHeight - 1);

        g_xy_x1_boundary[i] = static_cast<uint16>(lx);
        g_xy_y1_boundary[i] = static_cast<uint16>(ly);

        // 中线使用左右边界的均值。
        int mx = (lx + rx) / 2;
        int my = (ly + ry) / 2;
        g_xy_x2_boundary[i] = static_cast<uint16>(mx);
        g_xy_y2_boundary[i] = static_cast<uint16>(my);

        g_xy_x3_boundary[i] = static_cast<uint16>(rx);
        g_xy_y3_boundary[i] = static_cast<uint16>(ry);
    }

    g_boundary_count = out_num;
    g_boundary_left_count = std::clamp(left_num, 0, VISION_BOUNDARY_NUM);
    g_boundary_right_count = std::clamp(right_num, 0, VISION_BOUNDARY_NUM);
}

bool vision_image_processor_init(const char *camera_path)
{
    // 初始化阶段仅做：采图初始化 + 缓存清空 + 线误差复位。
    if (!vision_frame_capture_init(camera_path))
    {
        return false;
    }

    clear_boundary_arrays();
    clear_ipm_saved_arrays();
    g_undistort_map_x.release();
    g_undistort_map_y.release();
    g_undistort_ready = false;
    init_undistort_remap_table();
    vision_line_error_layer_reset();
    line_error = 0;
    g_last_otsu_threshold = 127;
    g_last_maze_left_start_x = -1;
    g_last_maze_right_start_x = -1;
    return true;
}

void vision_image_processor_cleanup()
{
    vision_frame_capture_cleanup();
    g_undistort_map_x.release();
    g_undistort_map_y.release();
    g_undistort_ready = false;
}

bool vision_image_processor_process_step()
{
    // 主处理步骤：
    // 1) 等待新帧；
    // 2) 生成灰度/RGB565/二值图；
    // 3) 迷宫法双边线提取；
    // 4) 计算中线误差与逆透视结果；
    // 5) 更新时间统计。
    auto t0 = std::chrono::steady_clock::now();

    if (!vision_frame_capture_wait_next_bgr(g_image_bgr_full, sizeof(g_image_bgr_full), 100, &g_last_capture_wait_us))
    {
        g_last_preprocess_us = 0;
        {
            std::lock_guard<std::mutex> lk(g_detect_result_mutex);
            g_last_red_detect_us = 0;
        }
        g_last_otsu_us = 0;
        g_last_maze_us = 0;
        g_last_total_us = g_last_capture_wait_us;
        g_last_maze_setup_us = 0;
        g_last_maze_start_us = 0;
        g_last_maze_trace_left_us = 0;
        g_last_maze_trace_right_us = 0;
        g_last_maze_post_us = 0;
        g_last_maze_left_points = 0;
        g_last_maze_right_points = 0;
        g_last_maze_left_ok = false;
        g_last_maze_right_ok = false;
        return false;
    }

    auto t1 = std::chrono::steady_clock::now();

    auto t_pre_start = t1;

    // 固定160x120，处理分辨率与采图分辨率一致，无需降采样。
    // 当前先做去畸变（含配置中的 undistort_move_x/y 平移补偿），再进入后续灰度/二值/巡线流程。
    cv::Mat bgr_full(UVC_HEIGHT, UVC_WIDTH, CV_8UC3, g_image_bgr_full);
    cv::Mat bgr(kProcHeight, kProcWidth, CV_8UC3, g_image_bgr);
    if (g_undistort_enabled.load() && init_undistort_remap_table())
    {
        cv::remap(bgr_full, bgr, g_undistort_map_x, g_undistort_map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    }
    else
    {
        std::memcpy(g_image_bgr, g_image_bgr_full, sizeof(g_image_bgr));
    }
    cv::Mat gray(kProcHeight, kProcWidth, CV_8UC1, g_image_gray);
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    // RGB565（当前处理分辨率）
    for (int y = 0; y < kProcHeight; ++y)
    {
        const cv::Vec3b *row = bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < kProcWidth; ++x)
        {
            uint8 bb = row[x][0];
            uint8 gg = row[x][1];
            uint8 rr = row[x][2];
            uint16 v = static_cast<uint16>(((rr >> 3) << 11) | ((gg >> 2) << 5) | (bb >> 3));
            int idx = (y * kProcWidth + x) * 2;
            g_image_rgb565[idx] = static_cast<uint8>(v >> 8);
            g_image_rgb565[idx + 1] = static_cast<uint8>(v & 0xFF);
        }
    }

    auto t_pre_end = std::chrono::steady_clock::now();

    auto t_otsu_start = t_pre_end;
    uint8 otsu_threshold = 127;
    if (g_vision_processor_config.demand_otsu_enable)
    {
        otsu_threshold = compute_global_otsu_threshold_u8(g_image_gray);
        if (g_vision_processor_config.demand_otsu_keep_full_binary_cache)
        {
            build_binary_image_from_gray_threshold(g_image_gray, otsu_threshold);
        }
        else
        {
            std::fill_n(g_image_binary_u8, kProcWidth * kProcHeight, static_cast<uint8>(0));
        }
    }
    else
    {
        cv::Mat gray_ipm(kProcHeight, kProcWidth, CV_8UC1, g_image_gray);
        cv::Mat binary(kProcHeight, kProcWidth, CV_8UC1, g_image_binary_u8);
        const double otsu_value = cv::threshold(gray_ipm, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        otsu_threshold = static_cast<uint8>(std::clamp(static_cast<int>(std::lround(otsu_value)), 0, 255));
    }
    g_last_otsu_threshold = otsu_threshold;
    auto t_otsu_end = std::chrono::steady_clock::now();

    auto t_maze_start = t_otsu_end;
    const int y_min = std::max(1, kProcHeight - (kProcHeight * g_vision_processor_config.maze_lower_region_percent) / 100);
    const uint8 *classify_img = nullptr;
    uint8 classify_white_threshold = 127;
    if (g_vision_processor_config.demand_otsu_enable)
    {
        classify_img = g_image_gray;
        classify_white_threshold = otsu_threshold;
    }
    else
    {
        classify_img = g_image_binary_u8;
        classify_white_threshold = 127;
    }

    int left_start_x = 0;
    int left_start_y = 0;
    int right_start_x = 0;
    int right_start_y = 0;
    int maze_trace_x_min = 1;
    int maze_trace_x_max = kProcWidth - 2;
    bool left_wall_is_white = false;
    bool right_wall_is_white = false;

    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_pts{};

    int left_num = 0;
    int right_num = 0;
    auto t_maze_setup_end = std::chrono::steady_clock::now();

    get_maze_trace_x_range_clamped(&maze_trace_x_min, &maze_trace_x_max);
    const int fallback_center_x = std::clamp(previous_src_centerline_first_x(), maze_trace_x_min, maze_trace_x_max);
    // 起点搜索改为“基于同侧历史起点向对侧偏移10像素”：
    // - 左边界：上一帧左起点 + 10（向右）；
    // - 右边界：上一帧右起点 - 10（向左）。
    // 无历史时退回到中心起搜。
    const int left_search_center_x =
        (g_last_maze_left_start_x >= maze_trace_x_min && g_last_maze_left_start_x <= maze_trace_x_max)
            ? std::clamp(g_last_maze_left_start_x + 10, maze_trace_x_min, maze_trace_x_max)
            : fallback_center_x;
    const int right_search_center_x =
        (g_last_maze_right_start_x >= maze_trace_x_min && g_last_maze_right_start_x <= maze_trace_x_max)
            ? std::clamp(g_last_maze_right_start_x - 10, maze_trace_x_min, maze_trace_x_max)
            : fallback_center_x;
    const int maze_start_row = std::clamp(g_maze_start_row.load(), 1, kProcHeight - 2);
    const int left_search_row = maze_start_row;
    const int right_search_row = maze_start_row;
    bool left_ok = find_maze_start_from_row(classify_img,
                                            classify_white_threshold,
                                            true,
                                            left_search_row,
                                            maze_trace_x_min,
                                            maze_trace_x_max,
                                            &left_start_x,
                                            &left_start_y,
                                            &left_wall_is_white,
                                            left_search_center_x);
    bool right_ok = find_maze_start_from_row(classify_img,
                                             classify_white_threshold,
                                             false,
                                             right_search_row,
                                             maze_trace_x_min,
                                             maze_trace_x_max,
                                             &right_start_x,
                                             &right_start_y,
                                             &right_wall_is_white,
                                             right_search_center_x);
    validate_maze_start_pair(classify_img,
                             classify_white_threshold,
                             maze_trace_x_min,
                             maze_trace_x_max,
                             &left_ok,
                             left_start_x,
                             left_start_y,
                             &right_ok,
                             right_start_x,
                             right_start_y);
    // 单边识别修正：
    // - 仅右边存在时，检查起点左侧5点白黑票；
    // - 仅左边存在时，检查起点右侧5点白黑票。
    // 白点多/相等认为当前边界身份正确；黑点多则改判到对侧。
    if (right_ok && !left_ok)
    {
        const bool right_is_right = single_boundary_side_vote_is_current_side(classify_img,
                                                                               classify_white_threshold,
                                                                               right_start_x,
                                                                               right_start_y,
                                                                               true);
        if (!right_is_right)
        {
            left_ok = true;
            left_start_x = right_start_x;
            left_start_y = right_start_y;
            left_wall_is_white = right_wall_is_white;
            right_ok = false;
        }
    }
    else if (left_ok && !right_ok)
    {
        const bool left_is_left = single_boundary_side_vote_is_current_side(classify_img,
                                                                             classify_white_threshold,
                                                                             left_start_x,
                                                                             left_start_y,
                                                                             false);
        if (!left_is_left)
        {
            right_ok = true;
            right_start_x = left_start_x;
            right_start_y = left_start_y;
            right_wall_is_white = left_wall_is_white;
            left_ok = false;
        }
    }
    auto t_maze_start_search_end = std::chrono::steady_clock::now();

    if (left_ok)
    {
        const int left_max_pts = std::min(static_cast<int>(left_pts.size()), g_vision_processor_config.maze_trace_max_points);
        left_num = maze_trace_left_hand(classify_img,
                                        classify_white_threshold,
                                        left_wall_is_white,
                                        left_start_x,
                                        left_start_y,
                                        y_min,
                                        maze_trace_x_min,
                                        maze_trace_x_max,
                                        left_pts.data(),
                                        left_max_pts);
    }
    auto t_maze_left_trace_end = std::chrono::steady_clock::now();
    if (right_ok)
    {
        const int right_max_pts = std::min(static_cast<int>(right_pts.size()), g_vision_processor_config.maze_trace_max_points);
        right_num = maze_trace_right_hand(classify_img,
                                          classify_white_threshold,
                                          right_wall_is_white,
                                          right_start_x,
                                          right_start_y,
                                          y_min,
                                          maze_trace_x_min,
                                          maze_trace_x_max,
                                          right_pts.data(),
                                          right_max_pts);
    }

    if (left_ok && right_ok)
    {
        const int start_gap = right_start_x - left_start_x;
        if (start_gap > 0 && start_gap < kMazeStartMinBoundaryGapPx)
        {
            if (left_num <= right_num)
            {
                left_ok = false;
                left_num = 0;
            }
            else
            {
                right_ok = false;
                right_num = 0;
            }
        }
    }
    auto t_maze_trace_end = std::chrono::steady_clock::now();

    // 主输出边界使用原图坐标系：左右边线 + 均值中线（无丢线补偿）。
    fill_boundary_arrays_from_maze(left_pts.data(), left_num, right_pts.data(), right_num);
    line_error = 0;

    if (g_vision_processor_config.enable_inverse_perspective)
    {
        render_ipm_boundary_image_and_update_boundaries(left_pts.data(),
                                                        left_num,
                                                        right_pts.data(),
                                                        right_num,
                                                        classify_img,
                                                        classify_white_threshold,
                                                        y_min,
                                                        maze_trace_x_min,
                                                        maze_trace_x_max,
                                                        g_ipm_line_error_preferred_source.load());
        const bool selected_is_right =
            (vision_line_error_layer_source() == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT));
        const uint16 *sel_ipm_x = selected_is_right ? g_ipm_shift_right_center_x : g_ipm_shift_left_center_x;
        const uint16 *sel_ipm_y = selected_is_right ? g_ipm_shift_right_center_y : g_ipm_shift_left_center_y;
        const int sel_ipm_count = selected_is_right ? g_ipm_shift_right_center_count : g_ipm_shift_left_center_count;
        const uint16 *sel_src_x = selected_is_right ? g_src_shift_right_center_x : g_src_shift_left_center_x;
        const uint16 *sel_src_y = selected_is_right ? g_src_shift_right_center_y : g_src_shift_left_center_y;
        const int sel_src_count = selected_is_right ? g_src_shift_right_center_count : g_src_shift_left_center_count;

        line_error = vision_line_error_layer_compute_from_ipm_shifted_centerline(sel_ipm_x,
                                                                                  sel_ipm_y,
                                                                                  sel_ipm_count,
                                                                                  sel_src_x,
                                                                                  sel_src_y,
                                                                                  sel_src_count,
                                                                                  kIpmOutputWidth / 2);
    }
    else
    {
        std::memset(g_image_ipm_bgr, 0, kIpmOutputWidth * kIpmOutputHeight * 3);
        clear_ipm_saved_arrays();
    }
    auto t_maze_end = std::chrono::steady_clock::now();

    g_last_preprocess_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_pre_end - t_pre_start).count());
    g_last_otsu_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_otsu_end - t_otsu_start).count());
    g_last_maze_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_maze_end - t_maze_start).count());
    g_last_maze_setup_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_maze_setup_end - t_maze_start).count());
    g_last_maze_start_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_maze_start_search_end - t_maze_setup_end).count());
    g_last_maze_trace_left_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_maze_left_trace_end - t_maze_start_search_end).count());
    g_last_maze_trace_right_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_maze_trace_end - t_maze_left_trace_end).count());
    g_last_maze_post_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_maze_end - t_maze_trace_end).count());
    g_last_maze_left_points = static_cast<uint16>(std::max(left_num, 0));
    g_last_maze_right_points = static_cast<uint16>(std::max(right_num, 0));
    g_last_maze_left_ok = left_ok;
    g_last_maze_right_ok = right_ok;
    // 规则：哪一边丢线，就把哪一边历史起点归为中心。
    // 当前搜索策略为 left=hist+10, right=hist-10，因此这里用反推值写入 hist。
    const int center_x = std::clamp((maze_trace_x_min + maze_trace_x_max) / 2, maze_trace_x_min, maze_trace_x_max);
    if (left_ok)
    {
        g_last_maze_left_start_x = left_start_x;
    }
    else
    {
        g_last_maze_left_start_x = std::clamp(center_x - 10, maze_trace_x_min, maze_trace_x_max);
    }
    if (right_ok)
    {
        g_last_maze_right_start_x = right_start_x;
    }
    else
    {
        g_last_maze_right_start_x = std::clamp(center_x + 10, maze_trace_x_min, maze_trace_x_max);
    }
    g_last_total_us = static_cast<uint32>(std::chrono::duration_cast<std::chrono::microseconds>(t_maze_end - t0).count());

    return true;
}

void vision_image_processor_set_maze_start_row(int row)
{
    // 如何修改：建议保持在 [1, kProcHeight-2] 内。
    g_maze_start_row.store(std::clamp(row, 1, kProcHeight - 2));
}

int vision_image_processor_get_maze_start_row()
{
    return g_maze_start_row.load();
}

void vision_image_processor_set_maze_trace_x_range(int x_min, int x_max)
{
    const int min_limit = 1;
    const int max_limit = kProcWidth - 2;
    int left = std::clamp(x_min, min_limit, max_limit);
    int right = std::clamp(x_max, min_limit, max_limit);
    if (left > right)
    {
        std::swap(left, right);
    }

    g_maze_trace_x_min.store(left);
    g_maze_trace_x_max.store(right);
}

void vision_image_processor_get_maze_trace_x_range(int *x_min, int *x_max)
{
    get_maze_trace_x_range_clamped(x_min, x_max);
}

void vision_image_processor_set_undistort_enabled(bool enabled)
{
    g_undistort_enabled.store(enabled);
}

bool vision_image_processor_undistort_enabled()
{
    return g_undistort_enabled.load();
}

void vision_image_processor_set_ipm_triangle_filter_enabled(bool enabled)
{
    g_ipm_triangle_filter_enabled.store(enabled);
}

bool vision_image_processor_ipm_triangle_filter_enabled()
{
    return g_ipm_triangle_filter_enabled.load();
}

void vision_image_processor_set_ipm_resample_enabled(bool enabled)
{
    g_ipm_resample_enabled.store(enabled);
}

bool vision_image_processor_ipm_resample_enabled()
{
    return g_ipm_resample_enabled.load();
}

void vision_image_processor_set_ipm_resample_step_px(float step_px)
{
    // 保护下限，避免 0 或负值导致异常行为。
    g_ipm_resample_step_px.store(std::max(0.1f, step_px));
}

float vision_image_processor_ipm_resample_step_px()
{
    return g_ipm_resample_step_px.load();
}

void vision_image_processor_set_ipm_boundary_kappa_sample_spacing_cm(float spacing_cm)
{
    g_ipm_boundary_kappa_sample_spacing_cm.store(std::max(1e-3f, spacing_cm));
}

float vision_image_processor_ipm_boundary_kappa_sample_spacing_cm()
{
    return g_ipm_boundary_kappa_sample_spacing_cm.load();
}

void vision_image_processor_set_ipm_boundary_angle_step(int step)
{
    g_ipm_boundary_angle_step.store(std::max(1, step));
}

int vision_image_processor_ipm_boundary_angle_step()
{
    return g_ipm_boundary_angle_step.load();
}

void vision_image_processor_set_ipm_boundary_shift_distance_px(float dist_px)
{
    g_ipm_boundary_shift_distance_px.store(std::max(0.0f, dist_px));
}

float vision_image_processor_ipm_boundary_shift_distance_px()
{
    return g_ipm_boundary_shift_distance_px.load();
}

void vision_image_processor_set_ipm_centerline_postprocess_enabled(bool enabled)
{
    g_ipm_centerline_postprocess_enabled.store(enabled);
}

bool vision_image_processor_ipm_centerline_postprocess_enabled()
{
    return g_ipm_centerline_postprocess_enabled.load();
}

void vision_image_processor_set_ipm_centerline_triangle_filter_enabled(bool enabled)
{
    g_ipm_centerline_triangle_filter_enabled.store(enabled);
}

bool vision_image_processor_ipm_centerline_triangle_filter_enabled()
{
    return g_ipm_centerline_triangle_filter_enabled.load();
}

void vision_image_processor_set_ipm_centerline_resample_enabled(bool enabled)
{
    g_ipm_centerline_resample_enabled.store(enabled);
}

bool vision_image_processor_ipm_centerline_resample_enabled()
{
    return g_ipm_centerline_resample_enabled.load();
}

void vision_image_processor_set_ipm_centerline_resample_step_px(float step_px)
{
    g_ipm_centerline_resample_step_px.store(std::max(0.1f, step_px));
}

float vision_image_processor_ipm_centerline_resample_step_px()
{
    return g_ipm_centerline_resample_step_px.load();
}

void vision_image_processor_set_ipm_line_error_source(vision_ipm_line_error_source_enum source)
{
    int preferred = static_cast<int>(source);
    if (preferred != static_cast<int>(VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT) &&
        preferred != static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT) &&
        preferred != static_cast<int>(VISION_IPM_LINE_ERROR_FROM_AUTO))
    {
        preferred = static_cast<int>(VISION_IPM_LINE_ERROR_FROM_AUTO);
    }
    g_ipm_line_error_preferred_source.store(preferred);
    if (preferred != static_cast<int>(VISION_IPM_LINE_ERROR_FROM_AUTO))
    {
        vision_line_error_layer_set_source(preferred);
    }
}

vision_ipm_line_error_source_enum vision_image_processor_ipm_line_error_source()
{
    const int source = vision_line_error_layer_source();
    if (source == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT))
    {
        return VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT;
    }
    if (source == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT))
    {
        return VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT;
    }
    return VISION_IPM_LINE_ERROR_FROM_AUTO;
}

void vision_image_processor_set_ipm_line_error_method(vision_ipm_line_error_method_enum method)
{
    vision_line_error_layer_set_method(static_cast<int>(method));
}

vision_ipm_line_error_method_enum vision_image_processor_ipm_line_error_method()
{
    const int m = vision_line_error_layer_method();
    if (m == static_cast<int>(VISION_IPM_LINE_ERROR_FIXED_INDEX))
    {
        return VISION_IPM_LINE_ERROR_FIXED_INDEX;
    }
    if (m == static_cast<int>(VISION_IPM_LINE_ERROR_SPEED_INDEX))
    {
        return VISION_IPM_LINE_ERROR_SPEED_INDEX;
    }
    return VISION_IPM_LINE_ERROR_WEIGHTED_INDEX;
}

void vision_image_processor_set_ipm_line_error_fixed_index(int point_index)
{
    vision_line_error_layer_set_fixed_index(point_index);
}

int vision_image_processor_ipm_line_error_fixed_index()
{
    return vision_line_error_layer_fixed_index();
}

void vision_image_processor_set_ipm_line_error_weighted_points(const int *point_indices,
                                                              const float *weights,
                                                              size_t count)
{
    vision_line_error_layer_set_weighted_points(point_indices, weights, count);
}

size_t vision_image_processor_ipm_line_error_weighted_point_count()
{
    return vision_line_error_layer_weighted_point_count();
}

void vision_image_processor_set_ipm_line_error_speed_formula(float speed_k, float speed_b)
{
    vision_line_error_layer_set_speed_formula(speed_k, speed_b);
}

void vision_image_processor_get_ipm_line_error_speed_formula(float *speed_k, float *speed_b)
{
    vision_line_error_layer_get_speed_formula(speed_k, speed_b);
}

void vision_image_processor_set_ipm_line_error_index_range(int index_min, int index_max)
{
    vision_line_error_layer_set_index_range(index_min, index_max);
}

void vision_image_processor_get_ipm_line_error_index_range(int *index_min, int *index_max)
{
    vision_line_error_layer_get_index_range(index_min, index_max);
}

void vision_image_processor_get_ipm_line_error_track_point(bool *valid, int *x, int *y)
{
    vision_line_error_layer_get_track_point(valid, x, y);
}

void vision_image_processor_set_ipm_centerline_curvature_step(int step)
{
    vision_line_error_layer_set_curvature_step(step);
}

int vision_image_processor_ipm_centerline_curvature_step()
{
    return vision_line_error_layer_curvature_step();
}

void vision_image_processor_get_ipm_selected_centerline_curvature(const float **curvature, int *count)
{
    // 当前返回 line_error 实际选中的那条 IPM 中线曲率序列，
    // 供控制层用来生成“来自视觉的目标横摆角速度”。
    vision_line_error_layer_get_selected_centerline_curvature(curvature, count);
}

void vision_image_processor_get_ipm_left_boundary_curvature(const float **curvature, int *count)
{
    if (curvature) *curvature = g_ipm_left_boundary_sg_curvature;
    if (count) *count = g_ipm_left_boundary_sg_curvature_count;
}

void vision_image_processor_get_ipm_right_boundary_curvature(const float **curvature, int *count)
{
    if (curvature) *curvature = g_ipm_right_boundary_sg_curvature;
    if (count) *count = g_ipm_right_boundary_sg_curvature_count;
}

void vision_image_processor_get_ipm_left_boundary_angle_cos(const float **angle_cos, int *count)
{
    if (angle_cos) *angle_cos = g_ipm_left_boundary_angle_cos;
    if (count) *count = g_ipm_left_boundary_angle_cos_count;
}

void vision_image_processor_get_ipm_right_boundary_angle_cos(const float **angle_cos, int *count)
{
    if (angle_cos) *angle_cos = g_ipm_right_boundary_angle_cos;
    if (count) *count = g_ipm_right_boundary_angle_cos_count;
}

float vision_image_processor_ipm_mean_abs_offset_error()
{
    return vision_line_error_layer_mean_abs_offset();
}

int vision_image_processor_ipm_weighted_first_point_error()
{
    return vision_line_error_layer_weighted_first_point_error();
}

void vision_image_processor_get_ipm_weighted_decision_point(bool *valid, int *x, int *y)
{
    vision_line_error_layer_get_ipm_weighted_decision_point(valid, x, y);
}

void vision_image_processor_get_src_weighted_decision_point(bool *valid, int *x, int *y)
{
    vision_line_error_layer_get_src_weighted_decision_point(valid, x, y);
}

int vision_image_processor_ipm_line_error_track_index()
{
    return vision_line_error_layer_track_index();
}

void vision_image_processor_get_last_perf_us(uint32 *capture_wait_us,
                                             uint32 *preprocess_us,
                                             uint32 *otsu_us,
                                             uint32 *maze_us,
                                             uint32 *total_us)
{
    // 参数说明：均为输出参数指针，可传 nullptr 跳过对应字段。
    if (capture_wait_us) *capture_wait_us = g_last_capture_wait_us;
    if (preprocess_us) *preprocess_us = g_last_preprocess_us;
    if (otsu_us) *otsu_us = g_last_otsu_us;
    if (maze_us) *maze_us = g_last_maze_us;
    if (total_us) *total_us = g_last_total_us;
}

uint8 vision_image_processor_get_last_otsu_threshold()
{
    return g_last_otsu_threshold;
}

void vision_image_processor_get_last_red_detect_us(uint32 *red_detect_us)
{
    // 注意：该值由 infer 模块更新，不在本文件内直接计算红框。
    if (red_detect_us)
    {
        std::lock_guard<std::mutex> lk(g_detect_result_mutex);
        *red_detect_us = g_last_red_detect_us;
    }
}

void vision_image_processor_get_last_maze_detail_us(uint32 *maze_setup_us,
                                                    uint32 *maze_start_us,
                                                    uint32 *maze_trace_left_us,
                                                    uint32 *maze_trace_right_us,
                                                    uint32 *maze_post_us,
                                                    uint16 *left_points,
                                                    uint16 *right_points,
                                                    bool *left_ok,
                                                    bool *right_ok)
{
    if (maze_setup_us) *maze_setup_us = g_last_maze_setup_us;
    if (maze_start_us) *maze_start_us = g_last_maze_start_us;
    if (maze_trace_left_us) *maze_trace_left_us = g_last_maze_trace_left_us;
    if (maze_trace_right_us) *maze_trace_right_us = g_last_maze_trace_right_us;
    if (maze_post_us) *maze_post_us = g_last_maze_post_us;
    if (left_points) *left_points = g_last_maze_left_points;
    if (right_points) *right_points = g_last_maze_right_points;
    if (left_ok) *left_ok = g_last_maze_left_ok;
    if (right_ok) *right_ok = g_last_maze_right_ok;
}

const uint8 *vision_image_processor_gray_image()
{
    return g_image_gray;
}

const uint8 *vision_image_processor_binary_u8_image()
{
    return g_image_binary_u8;
}

const uint8 *vision_image_processor_bgr_image()
{
    return g_image_bgr;
}

const uint8 *vision_image_processor_bgr_full_image()
{
    return g_image_bgr_full;
}

const uint8 *vision_image_processor_rgb565_image()
{
    return g_image_rgb565;
}

const uint8 *vision_image_processor_gray_downsampled_image()
{
    return g_image_gray;
}

const uint8 *vision_image_processor_binary_downsampled_u8_image()
{
    return g_image_binary_u8;
}

const uint8 *vision_image_processor_bgr_downsampled_image()
{
    return g_image_bgr;
}

const uint8 *vision_image_processor_rgb565_downsampled_image()
{
    return g_image_rgb565;
}

const uint8 *vision_image_processor_ipm_bgr_downsampled_image()
{
    return g_image_ipm_bgr;
}

void vision_image_processor_get_boundaries(uint16 **x1, uint16 **x2, uint16 **x3,
                                           uint16 **y1, uint16 **y2, uint16 **y3,
                                           uint16 *dot_num)
{
    // 返回内部静态缓存指针，调用方禁止修改内容。
    if (x1) *x1 = g_xy_x1_boundary;
    if (x2) *x2 = g_xy_x2_boundary;
    if (x3) *x3 = g_xy_x3_boundary;
    if (y1) *y1 = g_xy_y1_boundary;
    if (y2) *y2 = g_xy_y2_boundary;
    if (y3) *y3 = g_xy_y3_boundary;
    if (dot_num) *dot_num = static_cast<uint16>(g_boundary_count);
}

void vision_image_processor_get_boundary_side_counts(uint16 *left_dot_num, uint16 *right_dot_num)
{
    if (left_dot_num) *left_dot_num = static_cast<uint16>(g_boundary_left_count);
    if (right_dot_num) *right_dot_num = static_cast<uint16>(g_boundary_right_count);
}

void vision_image_processor_get_src_auxiliary_lines(uint16 **left_x, uint16 **left_y, uint16 *left_num,
                                                    uint16 **right_x, uint16 **right_y, uint16 *right_num)
{
    if (left_x) *left_x = g_xy_aux_left_x;
    if (left_y) *left_y = g_xy_aux_left_y;
    if (left_num) *left_num = static_cast<uint16>(g_xy_aux_left_count);
    if (right_x) *right_x = g_xy_aux_right_x;
    if (right_y) *right_y = g_xy_aux_right_y;
    if (right_num) *right_num = static_cast<uint16>(g_xy_aux_right_count);
}

void vision_image_processor_get_src_auxiliary_seed_points(uint16 **left_x, uint16 **left_y, uint16 *left_num,
                                                          uint16 **right_x, uint16 **right_y, uint16 *right_num)
{
    if (left_x) *left_x = g_xy_aux_left_seed_x;
    if (left_y) *left_y = g_xy_aux_left_seed_y;
    if (left_num) *left_num = static_cast<uint16>(g_xy_aux_left_seed_count);
    if (right_x) *right_x = g_xy_aux_right_seed_x;
    if (right_y) *right_y = g_xy_aux_right_seed_y;
    if (right_num) *right_num = static_cast<uint16>(g_xy_aux_right_seed_count);
}

void vision_image_processor_get_ipm_boundaries(uint16 **x1, uint16 **x2, uint16 **x3,
                                               uint16 **y1, uint16 **y2, uint16 **y3,
                                               uint16 *dot_num)
{
    if (x1) *x1 = g_ipm_xy_x1_boundary;
    if (x2) *x2 = g_ipm_xy_x2_boundary;
    if (x3) *x3 = g_ipm_xy_x3_boundary;
    if (y1) *y1 = g_ipm_xy_y1_boundary;
    if (y2) *y2 = g_ipm_xy_y2_boundary;
    if (y3) *y3 = g_ipm_xy_y3_boundary;
    if (dot_num) *dot_num = static_cast<uint16>(g_ipm_boundary_count);
}

void vision_image_processor_get_ipm_boundary_side_counts(uint16 *left_dot_num, uint16 *right_dot_num)
{
    if (left_dot_num) *left_dot_num = static_cast<uint16>(g_ipm_boundary_left_count);
    if (right_dot_num) *right_dot_num = static_cast<uint16>(g_ipm_boundary_right_count);
}

void vision_image_processor_get_src_boundary_corners(uint16 **left_x, uint16 **left_y, uint16 *left_num,
                                                     uint16 **right_x, uint16 **right_y, uint16 *right_num)
{
    if (left_x) *left_x = g_src_left_boundary_corner_x;
    if (left_y) *left_y = g_src_left_boundary_corner_y;
    if (left_num) *left_num = static_cast<uint16>(g_src_left_boundary_corner_point_count);
    if (right_x) *right_x = g_src_right_boundary_corner_x;
    if (right_y) *right_y = g_src_right_boundary_corner_y;
    if (right_num) *right_num = static_cast<uint16>(g_src_right_boundary_corner_point_count);
}

void vision_image_processor_get_ipm_boundary_corners(uint16 **left_x, uint16 **left_y, uint16 *left_num,
                                                     uint16 **right_x, uint16 **right_y, uint16 *right_num)
{
    if (left_x) *left_x = g_ipm_left_boundary_corner_x;
    if (left_y) *left_y = g_ipm_left_boundary_corner_y;
    if (left_num) *left_num = static_cast<uint16>(g_ipm_left_boundary_corner_point_count);
    if (right_x) *right_x = g_ipm_right_boundary_corner_x;
    if (right_y) *right_y = g_ipm_right_boundary_corner_y;
    if (right_num) *right_num = static_cast<uint16>(g_ipm_right_boundary_corner_point_count);
}

void vision_image_processor_get_ipm_shifted_centerline_from_left(uint16 **x, uint16 **y, uint16 *dot_num)
{
    if (x) *x = g_ipm_shift_left_center_x;
    if (y) *y = g_ipm_shift_left_center_y;
    if (dot_num) *dot_num = static_cast<uint16>(g_ipm_shift_left_center_count);
}

void vision_image_processor_get_ipm_shifted_centerline_from_right(uint16 **x, uint16 **y, uint16 *dot_num)
{
    if (x) *x = g_ipm_shift_right_center_x;
    if (y) *y = g_ipm_shift_right_center_y;
    if (dot_num) *dot_num = static_cast<uint16>(g_ipm_shift_right_center_count);
}

void vision_image_processor_get_src_shifted_centerline_from_left(uint16 **x, uint16 **y, uint16 *dot_num)
{
    if (x) *x = g_src_shift_left_center_x;
    if (y) *y = g_src_shift_left_center_y;
    if (dot_num) *dot_num = static_cast<uint16>(g_src_shift_left_center_count);
}

void vision_image_processor_get_src_shifted_centerline_from_right(uint16 **x, uint16 **y, uint16 *dot_num)
{
    if (x) *x = g_src_shift_right_center_x;
    if (y) *y = g_src_shift_right_center_y;
    if (dot_num) *dot_num = static_cast<uint16>(g_src_shift_right_center_count);
}

void vision_image_processor_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy)
{
    std::lock_guard<std::mutex> lk(g_detect_result_mutex);
    if (found) *found = g_red_rect_found;
    if (x) *x = g_red_rect_x;
    if (y) *y = g_red_rect_y;
    if (w) *w = g_red_rect_w;
    if (h) *h = g_red_rect_h;
    if (cx) *cx = g_red_rect_cx;
    if (cy) *cy = g_red_rect_cy;
}

int vision_image_processor_get_red_rect_area()
{
    std::lock_guard<std::mutex> lk(g_detect_result_mutex);
    return g_red_rect_area;
}

void vision_image_processor_set_red_rect(bool found, int x, int y, int w, int h, int cx, int cy, int area)
{
    std::lock_guard<std::mutex> lk(g_detect_result_mutex);
    g_red_rect_found = found;
    g_red_rect_x = std::clamp(x, 0, kProcWidth - 1);
    g_red_rect_y = std::clamp(y, 0, kProcHeight - 1);
    g_red_rect_w = std::clamp(w, 0, kProcWidth);
    g_red_rect_h = std::clamp(h, 0, kProcHeight);
    g_red_rect_cx = std::clamp(cx, 0, kProcWidth - 1);
    g_red_rect_cy = std::clamp(cy, 0, kProcHeight - 1);
    g_red_rect_area = std::max(0, area);
}

void vision_image_processor_set_last_red_detect_us(uint32 red_detect_us)
{
    std::lock_guard<std::mutex> lk(g_detect_result_mutex);
    g_last_red_detect_us = red_detect_us;
}

void vision_image_processor_set_ncnn_roi(bool valid, int x, int y, int w, int h)
{
    std::lock_guard<std::mutex> lk(g_detect_result_mutex);
    g_ncnn_roi_valid = valid;
    g_ncnn_roi_x = x;
    g_ncnn_roi_y = y;
    g_ncnn_roi_w = w;
    g_ncnn_roi_h = h;
}

void vision_image_processor_get_ncnn_roi(bool *valid, int *x, int *y, int *w, int *h)
{
    std::lock_guard<std::mutex> lk(g_detect_result_mutex);
    if (valid) *valid = g_ncnn_roi_valid;
    if (x) *x = g_ncnn_roi_x;
    if (y) *y = g_ncnn_roi_y;
    if (w) *w = g_ncnn_roi_w;
    if (h) *h = g_ncnn_roi_h;
}
