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
static constexpr int kCornerDebugBlurKernel = 3;
static constexpr float kCornerDebugResampleStepPx = 2.0f;
static constexpr int kCornerDebugAngleDist = 3;
static constexpr int kCornerDebugNmsKernel = 2;
static constexpr int kBoundaryTailFitPoints = 5;
static constexpr float kBoundaryTailExtendPx = 20.0f;
static constexpr float kCornerEnterAbsThreshold = 1.0f;
static constexpr float kIntersectionCornerDistanceMaxPx = 40.0f;
static constexpr int kIntersectionCornerYMin = 185;
static constexpr int kIntersectionEntryStartRow = 35;
static constexpr int kIntersectionProbeRowOffset = 5;
static constexpr int kIntersectionExitStopRow = 100;
static constexpr int kMazeTraceYFallbackStopDelta = 8;
static constexpr int kMazeStartMinBoundaryGapPx = 5;
static constexpr int kRoundaboutOutSearchRow = 80;

typedef enum
{
    ROUNDABOUT_MODE_NONE = 0,
    ROUNDABOUT_MODE_LEFT_APPROACH = 1,
    ROUNDABOUT_MODE_LEFT_LOOP = 2,
    ROUNDABOUT_MODE_LEFT_ENTRY = 3,
    ROUNDABOUT_MODE_LEFT_RUNNING = 4,
    ROUNDABOUT_MODE_LEFT_EXIT = 5,
    ROUNDABOUT_MODE_LEFT_OUT = 6,
    ROUNDABOUT_MODE_RIGHT_APPROACH = 7,
    ROUNDABOUT_MODE_RIGHT_LOOP = 8,
    ROUNDABOUT_MODE_RIGHT_ENTRY = 9,
    ROUNDABOUT_MODE_RIGHT_RUNNING = 10,
    ROUNDABOUT_MODE_RIGHT_EXIT = 11,
    ROUNDABOUT_MODE_RIGHT_OUT = 12
} roundabout_mode_enum;

// 原图坐标系边线缓存：x1/x2/x3 对应 左/中/右边线，y1/y2/y3 为对应 y。
static uint16 g_xy_x1_boundary[VISION_BOUNDARY_NUM];
static uint16 g_xy_x2_boundary[VISION_BOUNDARY_NUM];
static uint16 g_xy_x3_boundary[VISION_BOUNDARY_NUM];
static uint16 g_xy_y1_boundary[VISION_BOUNDARY_NUM];
static uint16 g_xy_y2_boundary[VISION_BOUNDARY_NUM];
static uint16 g_xy_y3_boundary[VISION_BOUNDARY_NUM];
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
// 逆透视后“原始拷贝”边界数组（未三角滤波）。
static uint16 g_ipm_raw_xy_x1_boundary[VISION_BOUNDARY_NUM];
static uint16 g_ipm_raw_xy_x2_boundary[VISION_BOUNDARY_NUM];
static uint16 g_ipm_raw_xy_x3_boundary[VISION_BOUNDARY_NUM];
static uint16 g_ipm_raw_xy_y1_boundary[VISION_BOUNDARY_NUM];
static uint16 g_ipm_raw_xy_y2_boundary[VISION_BOUNDARY_NUM];
static uint16 g_ipm_raw_xy_y3_boundary[VISION_BOUNDARY_NUM];
static int g_ipm_raw_boundary_count = 0;
static int g_ipm_raw_boundary_left_count = 0;
static int g_ipm_raw_boundary_right_count = 0;
// 角点识别专用边界：原始逆透视边界经过平滑、等距采样与 NMS 角度响应处理后用于网页显示。
static uint16 g_ipm_corner_left_x[VISION_BOUNDARY_NUM];
static uint16 g_ipm_corner_left_y[VISION_BOUNDARY_NUM];
static float g_ipm_corner_left_raw_value[VISION_BOUNDARY_NUM];
static float g_ipm_corner_left_value[VISION_BOUNDARY_NUM];
static int g_ipm_corner_left_count = 0;
static uint16 g_ipm_corner_right_x[VISION_BOUNDARY_NUM];
static uint16 g_ipm_corner_right_y[VISION_BOUNDARY_NUM];
static float g_ipm_corner_right_raw_value[VISION_BOUNDARY_NUM];
static float g_ipm_corner_right_value[VISION_BOUNDARY_NUM];
static int g_ipm_corner_right_count = 0;
static bool g_last_left_corner_valid = false;
static int g_last_left_corner_x = 0;
static int g_last_left_corner_y = 0;
static bool g_last_right_corner_valid = false;
static int g_last_right_corner_x = 0;
static int g_last_right_corner_y = 0;
static bool g_intersection_mode = false;
static int g_intersection_probe_row = kIntersectionEntryStartRow;
static int g_intersection_last_stop_row = 0;
static int g_intersection_current_start_row = -1;
static int g_intersection_probe_x = kProcWidth / 2;
static std::atomic<int> g_roundabout_mode(ROUNDABOUT_MODE_NONE);

static bool roundabout_mode_forces_right_centerline(int mode);
static bool roundabout_mode_forces_left_centerline(int mode);
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
// 逆透视后拟合中线（另存）
static uint16 g_ipm_center_fit_x[VISION_BOUNDARY_NUM];
static uint16 g_ipm_center_fit_y[VISION_BOUNDARY_NUM];
static int g_ipm_center_fit_count = 0;

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

// 迷宫法起始搜索行（可运行时配置）。
static std::atomic<int> g_maze_start_row(g_vision_runtime_config.maze_start_row);
static std::atomic<int> g_maze_trace_x_min(g_vision_processor_config.default_maze_trace_x_min);
static std::atomic<int> g_maze_trace_x_max(g_vision_processor_config.default_maze_trace_x_max);
static std::atomic<bool> g_intersection_enabled(g_vision_runtime_config.intersection_enabled);
// 去畸变开关（默认开启）。
static std::atomic<bool> g_undistort_enabled(g_vision_runtime_config.undistort_enabled);
// 逆透视边界三角滤波开关（默认开启）。
static std::atomic<bool> g_ipm_triangle_filter_enabled(g_vision_runtime_config.ipm_triangle_filter_enabled);
// 逆透视边界近重复点过滤开关与阈值。
static std::atomic<bool> g_ipm_min_point_dist_filter_enabled(g_vision_runtime_config.ipm_min_point_dist_filter_enabled);
static std::atomic<float> g_ipm_min_point_dist_px(g_vision_runtime_config.ipm_min_point_dist_px);
// 逆透视边界等距采样开关与参数（默认开启，步长1px）。
static std::atomic<bool> g_ipm_resample_enabled(g_vision_runtime_config.ipm_resample_enabled);
static std::atomic<float> g_ipm_resample_step_px(g_vision_runtime_config.ipm_resample_step_px);
static std::atomic<float> g_ipm_boundary_shift_distance_px(g_vision_runtime_config.ipm_boundary_shift_distance_px);
// 逆透视处理中线独立配置。
static std::atomic<bool> g_ipm_centerline_postprocess_enabled(g_vision_runtime_config.ipm_centerline_postprocess_enabled);
static std::atomic<bool> g_ipm_centerline_triangle_filter_enabled(g_vision_runtime_config.ipm_centerline_triangle_filter_enabled);
static std::atomic<bool> g_ipm_centerline_resample_enabled(g_vision_runtime_config.ipm_centerline_resample_enabled);
static std::atomic<float> g_ipm_centerline_resample_step_px(g_vision_runtime_config.ipm_centerline_resample_step_px);
static std::atomic<float> g_ipm_centerline_min_point_dist_px(g_vision_runtime_config.ipm_centerline_min_point_dist_px);
static std::atomic<bool> g_keep_last_centerline_on_double_loss(g_vision_runtime_config.keep_last_centerline_on_double_loss);

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
static int previous_src_centerline_first_x();
static inline bool pixel_is_white(const uint8 *img, int x, int y, uint8 white_threshold);
static void resample_boundary_points_equal_spacing_inplace(maze_point_t *pts, int *num, int width, int height, float step_px);
static bool init_undistort_remap_table();
static bool init_ipm_forward_matrix();
static bool ipm_point_to_src_point(int ipm_x, int ipm_y, int *src_x, int *src_y);
static void get_maze_trace_x_range_clamped(int *x_min, int *x_max);
static void trim_last_point_and_extend_tail_inplace(maze_point_t *pts, int *num, int width, int height);
static void update_corner_state_from_current_frame(int left_first_corner_idx, int right_first_corner_idx);
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
static int compute_intersection_probe_x_from_corners();
static int scan_intersection_stop_row(const uint8 *classify_img,
                                      uint8 white_threshold,
                                      int start_row,
                                      int *probe_x,
                                      int x_min,
                                      int x_max);
static void reset_intersection_mode_state();
static void enter_intersection_mode_if_needed();
static void maybe_exit_intersection_mode(int stop_row);

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

static inline int clip_int_local(int x, int low, int up)
{
    if (x < low) return low;
    if (x > up) return up;
    return x;
}

static void blur_points_local(const float pts_in[][2], int num, float pts_out[][2], int kernel)
{
    const int half = kernel / 2;
    const float denom = static_cast<float>((2 * half + 2) * (half + 1)) * 0.5f;
    for (int i = 0; i < num; ++i)
    {
        float sx = 0.0f;
        float sy = 0.0f;
        for (int j = -half; j <= half; ++j)
        {
            const int idx = clip_int_local(i + j, 0, num - 1);
            const float w = static_cast<float>(half + 1 - std::abs(j));
            sx += pts_in[idx][0] * w;
            sy += pts_in[idx][1] * w;
        }
        pts_out[i][0] = sx / denom;
        pts_out[i][1] = sy / denom;
    }
}

static void resample_points_local(const float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist)
{
    if (!num2 || num1 <= 0 || dist <= 0.0f)
    {
        return;
    }

    float remain = 0.0f;
    int len = 0;
    for (int i = 0; i < num1 - 1 && len < *num2; ++i)
    {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i + 1][0] - x0;
        float dy = pts_in[i + 1][1] - y0;
        const float dn = std::sqrt(dx * dx + dy * dy);
        if (dn <= 1e-6f)
        {
            continue;
        }
        dx /= dn;
        dy /= dn;

        while (remain < dn && len < *num2)
        {
            x0 += dx * remain;
            y0 += dy * remain;
            pts_out[len][0] = x0;
            pts_out[len][1] = y0;
            ++len;
            remain += dist;
        }
        remain -= dn;
    }
    *num2 = len;
}

static void local_angle_points_local(const float pts_in[][2], int num, float angle_out[], int dist)
{
    for (int i = 0; i < num; ++i)
    {
        if (i <= dist || i >= num - dist)
        {
            angle_out[i] = 0.0f;
            continue;
        }

        float c1 = pts_in[i][0] - pts_in[i - dist][0];
        float s1 = pts_in[i][1] - pts_in[i - dist][1];
        float c2 = pts_in[i + dist][0] - pts_in[i][0];
        float s2 = pts_in[i + dist][1] - pts_in[i][1];
        const float n1 = std::sqrt(c1 * c1 + s1 * s1);
        const float n2 = std::sqrt(c2 * c2 + s2 * s2);
        if (n1 <= 1e-6f || n2 <= 1e-6f)
        {
            angle_out[i] = 0.0f;
            continue;
        }

        c1 /= n1;
        s1 /= n1;
        c2 /= n2;
        s2 /= n2;
        angle_out[i] = std::atan2(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

static void nms_angle_local(const float angle_in[], int num, float angle_out[], int kernel)
{
    for (int i = 0; i < num; ++i)
    {
        angle_out[i] = angle_in[i];
        for (int j = -kernel; j <= kernel; ++j)
        {
            const int idx = clip_int_local(i + j, 0, num - 1);
            if (std::fabs(angle_in[idx]) > std::fabs(angle_out[i]))
            {
                angle_out[i] = 0.0f;
                break;
            }
        }
    }
}

static void build_corner_debug_boundary(const maze_point_t *src_pts,
                                        int src_num,
                                        uint16 out_x[],
                                        uint16 out_y[],
                                        float out_raw_value[],
                                        float out_value[],
                                        int *out_num,
                                        maze_point_t proc_pts[],
                                        int *proc_num,
                                        int *first_corner_idx)
{
    if (!out_num)
    {
        return;
    }

    *out_num = 0;
    if (proc_num) *proc_num = 0;
    if (first_corner_idx) *first_corner_idx = -1;
    if (!src_pts || src_num <= 0)
    {
        return;
    }

    const int clipped_num = std::min(src_num, VISION_BOUNDARY_NUM);
    float pts_in[VISION_BOUNDARY_NUM][2] = {};
    float pts_blur[VISION_BOUNDARY_NUM][2] = {};
    float pts_resample[VISION_BOUNDARY_NUM][2] = {};
    float angle_raw[VISION_BOUNDARY_NUM] = {};
    float angle_nms[VISION_BOUNDARY_NUM] = {};

    for (int i = 0; i < clipped_num; ++i)
    {
        pts_in[i][0] = static_cast<float>(src_pts[i].x);
        pts_in[i][1] = static_cast<float>(src_pts[i].y);
    }

    blur_points_local(pts_in, clipped_num, pts_blur, kCornerDebugBlurKernel);

    int resample_num = VISION_BOUNDARY_NUM;
    resample_points_local(pts_blur, clipped_num, pts_resample, &resample_num, kCornerDebugResampleStepPx);
    if (resample_num <= 0)
    {
        return;
    }

    local_angle_points_local(pts_resample, resample_num, angle_raw, kCornerDebugAngleDist);
    nms_angle_local(angle_raw, resample_num, angle_nms, kCornerDebugNmsKernel);

    for (int i = 0; i < resample_num; ++i)
    {
        const int px = std::clamp(static_cast<int>(std::lround(pts_resample[i][0])), 0, kIpmOutputWidth - 1);
        const int py = std::clamp(static_cast<int>(std::lround(pts_resample[i][1])), 0, kIpmOutputHeight - 1);
        out_x[i] = static_cast<uint16>(px);
        out_y[i] = static_cast<uint16>(py);
        if (out_raw_value)
        {
            out_raw_value[i] = angle_raw[i];
        }
        out_value[i] = angle_nms[i];
        if (proc_pts)
        {
            proc_pts[i] = {px, py};
        }
        if (first_corner_idx && *first_corner_idx < 0 && std::fabs(angle_nms[i]) >= 1.0f)
        {
            *first_corner_idx = i;
        }
    }
    *out_num = resample_num;
    if (proc_num)
    {
        *proc_num = resample_num;
    }
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
    g_boundary_count = 0;
    g_boundary_left_count = 0;
    g_boundary_right_count = 0;
}

static void clear_ipm_saved_arrays()
{
    // 作用：清空逆透视边线和拟合中线缓存。
    std::fill_n(g_ipm_xy_x1_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_xy_x2_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_xy_x3_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_xy_y1_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_xy_y2_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_xy_y3_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_raw_xy_x1_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_raw_xy_x2_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_raw_xy_x3_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_raw_xy_y1_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_raw_xy_y2_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_raw_xy_y3_boundary, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_corner_left_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_corner_left_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_corner_left_value, VISION_BOUNDARY_NUM, 0.0f);
    std::fill_n(g_ipm_corner_right_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_corner_right_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_corner_right_value, VISION_BOUNDARY_NUM, 0.0f);
    std::fill_n(g_ipm_shift_left_center_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_shift_left_center_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_shift_right_center_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_shift_right_center_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_center_fit_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_center_fit_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    g_ipm_boundary_count = 0;
    g_ipm_raw_boundary_count = 0;
    g_ipm_boundary_left_count = 0;
    g_ipm_boundary_right_count = 0;
    g_ipm_raw_boundary_left_count = 0;
    g_ipm_raw_boundary_right_count = 0;
    g_ipm_corner_left_count = 0;
    g_ipm_corner_right_count = 0;
    g_ipm_shift_left_center_count = 0;
    g_ipm_shift_right_center_count = 0;
    g_ipm_center_fit_count = 0;
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

static void trim_last_point_and_extend_tail_inplace(maze_point_t *pts, int *num, int width, int height)
{
    if (pts == nullptr || num == nullptr || *num <= 1 || width <= 0 || height <= 0)
    {
        return;
    }

    int count = std::clamp(*num, 0, VISION_BOUNDARY_NUM);
    if (count <= 1)
    {
        *num = count;
        return;
    }

    // 去重后先丢弃最后一个像素点。
    --count;
    if (count <= 1)
    {
        *num = count;
        return;
    }

    const int fit_count = std::min(count, kBoundaryTailFitPoints);
    const int start_idx = count - fit_count;
    const float start_x = static_cast<float>(pts[start_idx].x);
    const float start_y = static_cast<float>(pts[start_idx].y);
    const float end_x = static_cast<float>(pts[count - 1].x);
    const float end_y = static_cast<float>(pts[count - 1].y);
    float dir_x = end_x - start_x;
    float dir_y = end_y - start_y;
    const float dir_len = std::sqrt(dir_x * dir_x + dir_y * dir_y);
    if (dir_len <= 1e-6f)
    {
        *num = count;
        return;
    }

    dir_x /= dir_len;
    dir_y /= dir_len;
    const maze_point_t anchor = pts[count - 1];
    const int max_x = width - 1;
    const int max_y = height - 1;
    int out = count;
    const int extend_steps = std::max(0, static_cast<int>(std::lround(kBoundaryTailExtendPx)));
    for (int step = 1; step <= extend_steps && out < VISION_BOUNDARY_NUM; ++step)
    {
        const int px = std::clamp(static_cast<int>(std::lround(static_cast<float>(anchor.x) + dir_x * static_cast<float>(step))), 0, max_x);
        const int py = std::clamp(static_cast<int>(std::lround(static_cast<float>(anchor.y) + dir_y * static_cast<float>(step))), 0, max_y);
        if (px == pts[out - 1].x && py == pts[out - 1].y)
        {
            continue;
        }
        pts[out++] = {px, py};
    }
    *num = out;
}

static void update_corner_state_from_current_frame(int left_first_corner_idx, int right_first_corner_idx)
{
    g_last_left_corner_valid = (left_first_corner_idx >= 0 && left_first_corner_idx < g_ipm_corner_left_count);
    if (g_last_left_corner_valid)
    {
        g_last_left_corner_x = static_cast<int>(g_ipm_corner_left_x[left_first_corner_idx]);
        g_last_left_corner_y = static_cast<int>(g_ipm_corner_left_y[left_first_corner_idx]);
    }

    g_last_right_corner_valid = (right_first_corner_idx >= 0 && right_first_corner_idx < g_ipm_corner_right_count);
    if (g_last_right_corner_valid)
    {
        g_last_right_corner_x = static_cast<int>(g_ipm_corner_right_x[right_first_corner_idx]);
        g_last_right_corner_y = static_cast<int>(g_ipm_corner_right_y[right_first_corner_idx]);
    }
}

static bool prefix_boundary_is_straightish(const uint16 *xs,
                                           const uint16 *ys,
                                           const float *nms,
                                           int count,
                                           float span_px,
                                           float nms_abs_max)
{
    if (xs == nullptr || ys == nullptr || nms == nullptr || count <= 0)
    {
        return false;
    }

    float traveled = 0.0f;
    bool all_low_nms = true;
    bool has_sample = false;
    for (int i = 0; i < count; ++i)
    {
        if (i > 0)
        {
            const float dx = static_cast<float>(xs[i] - xs[i - 1]);
            const float dy = static_cast<float>(ys[i] - ys[i - 1]);
            traveled += std::sqrt(dx * dx + dy * dy);
        }
        has_sample = true;
        all_low_nms = all_low_nms && (std::fabs(nms[i]) < nms_abs_max);
        if (traveled >= span_px)
        {
            break;
        }
    }

    if (!has_sample)
    {
        return false;
    }
    return all_low_nms;
}

static bool boundary_ring_points_are_straightish(const uint16 *xs,
                                                 const uint16 *ys,
                                                 const float *nms,
                                                 int count,
                                                 int ref_x,
                                                 int ref_y,
                                                 float target_dist_px,
                                                 float dist_tolerance_px,
                                                 float nms_abs_max)
{
    if (xs == nullptr || ys == nullptr || nms == nullptr || count <= 0)
    {
        return false;
    }

    bool all_low_nms = true;
    bool has_sample = false;
    for (int i = 0; i < count; ++i)
    {
        const float dx = static_cast<float>(xs[i] - ref_x);
        const float dy = static_cast<float>(ys[i] - ref_y);
        const float dist = std::sqrt(dx * dx + dy * dy);
        if (std::fabs(dist - target_dist_px) > dist_tolerance_px)
        {
            continue;
        }
        has_sample = true;
        all_low_nms = all_low_nms && (std::fabs(nms[i]) < nms_abs_max);
    }

    if (!has_sample)
    {
        return false;
    }
    return all_low_nms;
}

static bool detect_left_roundabout_candidate()
{
    if (!g_last_left_corner_valid)
    {
        return false;
    }
    return boundary_ring_points_are_straightish(g_ipm_corner_right_x,
                                                g_ipm_corner_right_y,
                                                g_ipm_corner_right_value,
                                                g_ipm_corner_right_count,
                                                g_last_left_corner_x,
                                                g_last_left_corner_y,
                                                g_vision_runtime_config.roundabout_corner_match_distance_px,
                                                g_vision_runtime_config.roundabout_corner_match_tolerance_px,
                                                g_vision_runtime_config.roundabout_straight_nms_abs_max);
}

static bool detect_right_roundabout_candidate()
{
    if (!g_last_right_corner_valid)
    {
        return false;
    }
    return boundary_ring_points_are_straightish(g_ipm_corner_left_x,
                                                g_ipm_corner_left_y,
                                                g_ipm_corner_left_value,
                                                g_ipm_corner_left_count,
                                                g_last_right_corner_x,
                                                g_last_right_corner_y,
                                                g_vision_runtime_config.roundabout_corner_match_distance_px,
                                                g_vision_runtime_config.roundabout_corner_match_tolerance_px,
                                                g_vision_runtime_config.roundabout_straight_nms_abs_max);
}

static bool roundabout_mode_forces_right_centerline(int mode)
{
    return mode == ROUNDABOUT_MODE_LEFT_APPROACH ||
           mode == ROUNDABOUT_MODE_LEFT_LOOP ||
           mode == ROUNDABOUT_MODE_LEFT_OUT ||
           mode == ROUNDABOUT_MODE_RIGHT_ENTRY ||
           mode == ROUNDABOUT_MODE_RIGHT_RUNNING ||
           mode == ROUNDABOUT_MODE_RIGHT_EXIT;
}

static bool roundabout_mode_forces_left_centerline(int mode)
{
    return mode == ROUNDABOUT_MODE_RIGHT_APPROACH ||
           mode == ROUNDABOUT_MODE_RIGHT_LOOP ||
           mode == ROUNDABOUT_MODE_RIGHT_OUT ||
           mode == ROUNDABOUT_MODE_LEFT_ENTRY ||
           mode == ROUNDABOUT_MODE_LEFT_RUNNING ||
           mode == ROUNDABOUT_MODE_LEFT_EXIT;
}

static void apply_roundabout_shift_distance_override()
{
    const int mode = g_roundabout_mode.load();
    const bool use_special_shift = (mode == ROUNDABOUT_MODE_LEFT_ENTRY ||
                                    mode == ROUNDABOUT_MODE_LEFT_RUNNING ||
                                    mode == ROUNDABOUT_MODE_RIGHT_ENTRY ||
                                    mode == ROUNDABOUT_MODE_RIGHT_RUNNING);
    g_ipm_boundary_shift_distance_px.store(use_special_shift ? g_vision_runtime_config.roundabout_running_shift_distance_px
                                                            : g_vision_runtime_config.roundabout_normal_shift_distance_px);
}

static void enter_roundabout_mode_if_needed()
{
    // 环岛识别流程当前按需求停用：保持一般模式，不进入任何环岛状态。
    g_roundabout_mode.store(ROUNDABOUT_MODE_NONE);
}

static void update_roundabout_mode_after_current_frame(bool left_found, bool right_found)
{
    (void)left_found;
    (void)right_found;
    // 环岛识别流程当前按需求停用：保持一般模式，不做状态推进。
    g_roundabout_mode.store(ROUNDABOUT_MODE_NONE);
}

static int compute_intersection_probe_x_from_corners()
{
    int left_src_x = 0;
    int left_src_y = 0;
    int right_src_x = 0;
    int right_src_y = 0;
    if (!g_last_left_corner_valid || !g_last_right_corner_valid ||
        !ipm_point_to_src_point(g_last_left_corner_x, g_last_left_corner_y, &left_src_x, &left_src_y) ||
        !ipm_point_to_src_point(g_last_right_corner_x, g_last_right_corner_y, &right_src_x, &right_src_y))
    {
        return kProcWidth / 2;
    }

    const float mid_x = 0.5f * static_cast<float>(left_src_x + right_src_x);
    const float mid_y = 0.5f * static_cast<float>(left_src_y + right_src_y);
    const float seg_dx = static_cast<float>(right_src_x - left_src_x);
    const float seg_dy = static_cast<float>(right_src_y - left_src_y);
    const float dir_x = -seg_dy;
    const float dir_y = seg_dx;
    if (std::fabs(dir_y) < 1e-6f)
    {
        return std::clamp(static_cast<int>(std::lround(mid_x)), 1, kProcWidth - 2);
    }

    const float t = (static_cast<float>(kIntersectionEntryStartRow) - mid_y) / dir_y;
    return std::clamp(static_cast<int>(std::lround(mid_x + t * dir_x)), 1, kProcWidth - 2);
}

static int scan_intersection_stop_row(const uint8 *classify_img,
                                      uint8 white_threshold,
                                      int start_row,
                                      int *probe_x,
                                      int x_min,
                                      int x_max)
{
    int center_x = (probe_x != nullptr) ? std::clamp(*probe_x, x_min, x_max) : (kProcWidth / 2);
    int stop_row = std::clamp(start_row, 1, kProcHeight - 2);
    for (int row = stop_row; row <= kProcHeight - 2; ++row)
    {
        int left_start_x = 0;
        int left_start_y = 0;
        int right_start_x = 0;
        int right_start_y = 0;
        bool left_wall_is_white = false;
        bool right_wall_is_white = false;
        const bool left_ok = find_maze_start_from_row(classify_img,
                                                      white_threshold,
                                                      true,
                                                      row,
                                                      x_min,
                                                      x_max,
                                                      &left_start_x,
                                                      &left_start_y,
                                                      &left_wall_is_white,
                                                      center_x);
        const bool right_ok = find_maze_start_from_row(classify_img,
                                                       white_threshold,
                                                       false,
                                                       row,
                                                       x_min,
                                                       x_max,
                                                       &right_start_x,
                                                       &right_start_y,
                                                       &right_wall_is_white,
                                                       center_x);
        if (!(left_ok && right_ok))
        {
            stop_row = row;
            return stop_row;
        }
        center_x = std::clamp((left_start_x + right_start_x) / 2, x_min, x_max);
        if (probe_x != nullptr)
        {
            *probe_x = center_x;
        }
        stop_row = row;
    }
    return stop_row;
}

static void reset_intersection_mode_state()
{
    g_intersection_mode = false;
    g_intersection_probe_row = kIntersectionEntryStartRow;
    g_intersection_last_stop_row = 0;
    g_intersection_current_start_row = -1;
    g_intersection_probe_x = kProcWidth / 2;
}

static void enter_intersection_mode_if_needed()
{
    if (!g_intersection_enabled.load())
    {
        reset_intersection_mode_state();
        return;
    }
    if (g_intersection_mode || g_roundabout_mode.load() != ROUNDABOUT_MODE_NONE ||
        !g_last_left_corner_valid || !g_last_right_corner_valid)
    {
        return;
    }

    const int dx = g_last_left_corner_x - g_last_right_corner_x;
    const int dy = g_last_left_corner_y - g_last_right_corner_y;
    const float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy));
    if (dist >= kIntersectionCornerDistanceMaxPx)
    {
        return;
    }
    if (std::max(g_last_left_corner_y, g_last_right_corner_y) <= kIntersectionCornerYMin)
    {
        return;
    }

    g_intersection_mode = true;
    g_intersection_probe_row = kIntersectionEntryStartRow;
    g_intersection_last_stop_row = kIntersectionEntryStartRow;
    g_intersection_current_start_row = kIntersectionEntryStartRow;
    g_intersection_probe_x = compute_intersection_probe_x_from_corners();
}

static void maybe_exit_intersection_mode(int stop_row)
{
    g_intersection_last_stop_row = stop_row;
    g_intersection_probe_row = std::clamp(stop_row, 1, kProcHeight - 2);
    if (!g_intersection_mode || stop_row < kIntersectionExitStopRow)
    {
        return;
    }

    reset_intersection_mode_state();
}

static void postprocess_shifted_centerline_inplace(maze_point_t *pts, int *num, int width, int height)
{
    if (pts == nullptr || num == nullptr)
    {
        return;
    }

    remove_near_duplicate_boundary_points_inplace(pts, num, g_ipm_centerline_min_point_dist_px.load());
    if (!g_ipm_centerline_postprocess_enabled.load())
    {
        return;
    }

    if (g_ipm_centerline_triangle_filter_enabled.load())
    {
        triangle_filter_boundary_points_inplace(pts, *num, width, height);
        remove_near_duplicate_boundary_points_inplace(pts, num, g_ipm_centerline_min_point_dist_px.load());
    }

    if (g_ipm_centerline_resample_enabled.load())
    {
        resample_boundary_points_equal_spacing_inplace(pts,
                                                       num,
                                                       width,
                                                       height,
                                                       g_ipm_centerline_resample_step_px.load());
        remove_near_duplicate_boundary_points_inplace(pts, num, g_ipm_centerline_min_point_dist_px.load());
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
                                                           int right_num)
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

    // 第一份：原始逆透视边界（用于角点识别等保形任务）。
    fill_boundary_arrays_from_points_to_target(left_ipm.data(),
                                               left_ipm_num,
                                               right_ipm.data(),
                                               right_ipm_num,
                                               g_ipm_raw_xy_x1_boundary,
                                               g_ipm_raw_xy_x2_boundary,
                                               g_ipm_raw_xy_x3_boundary,
                                               g_ipm_raw_xy_y1_boundary,
                                               g_ipm_raw_xy_y2_boundary,
                                               g_ipm_raw_xy_y3_boundary,
                                               &g_ipm_raw_boundary_count,
                                               kIpmOutputWidth,
                                               kIpmOutputHeight);
    g_ipm_raw_boundary_left_count = std::clamp(left_ipm_num, 0, VISION_BOUNDARY_NUM);
    g_ipm_raw_boundary_right_count = std::clamp(right_ipm_num, 0, VISION_BOUNDARY_NUM);
    // 第二份：角点识别专用处理链。
    // 先对原始逆透视边界做平滑、等距采样、局部角度计算与 NMS，
    // 再记录从头开始第一个 |nms|>=1 的点作为直角点，并在该点处截断。
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_proc{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_proc{};
    int left_proc_num = 0;
    int right_proc_num = 0;
    int left_first_corner_idx = -1;
    int right_first_corner_idx = -1;
    build_corner_debug_boundary(left_ipm.data(),
                                left_ipm_num,
                                g_ipm_corner_left_x,
                                g_ipm_corner_left_y,
                                g_ipm_corner_left_raw_value,
                                g_ipm_corner_left_value,
                                &g_ipm_corner_left_count,
                                left_proc.data(),
                                &left_proc_num,
                                &left_first_corner_idx);
    build_corner_debug_boundary(right_ipm.data(),
                                right_ipm_num,
                                g_ipm_corner_right_x,
                                g_ipm_corner_right_y,
                                g_ipm_corner_right_raw_value,
                                g_ipm_corner_right_value,
                                &g_ipm_corner_right_count,
                                right_proc.data(),
                                &right_proc_num,
                                &right_first_corner_idx);
    if (left_first_corner_idx >= 0)
    {
        left_proc_num = std::min(left_proc_num, left_first_corner_idx + 1);
    }
    if (right_first_corner_idx >= 0)
    {
        right_proc_num = std::min(right_proc_num, right_first_corner_idx + 1);
    }
    if (left_proc_num <= 0)
    {
        left_proc = left_ipm;
        left_proc_num = left_ipm_num;
    }
    if (right_proc_num <= 0)
    {
        right_proc = right_ipm;
        right_proc_num = right_ipm_num;
    }
    if (g_ipm_min_point_dist_filter_enabled.load())
    {
        remove_near_duplicate_boundary_points_inplace(left_proc.data(), &left_proc_num, g_ipm_min_point_dist_px.load());
        remove_near_duplicate_boundary_points_inplace(right_proc.data(), &right_proc_num, g_ipm_min_point_dist_px.load());
    }
    trim_last_point_and_extend_tail_inplace(left_proc.data(), &left_proc_num, kIpmOutputWidth, kIpmOutputHeight);
    trim_last_point_and_extend_tail_inplace(right_proc.data(), &right_proc_num, kIpmOutputWidth, kIpmOutputHeight);

    // 处理链边界法向平移中线：
    // - 左边界向右平移；
    // - 右边界向左平移。
    std::array<maze_point_t, VISION_BOUNDARY_NUM> center_from_left{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> center_from_right{};
    int center_from_left_num = 0;
    int center_from_right_num = 0;
    const float shift_dist_px = g_ipm_boundary_shift_distance_px.load();
    shift_boundary_along_normal(left_proc.data(),
                                left_proc_num,
                                right_proc.data(),
                                right_proc_num,
                                center_from_left.data(),
                                &center_from_left_num,
                                kIpmOutputWidth,
                                kIpmOutputHeight,
                                shift_dist_px,
                                true);
    shift_boundary_along_normal(right_proc.data(),
                                right_proc_num,
                                left_proc.data(),
                                left_proc_num,
                                center_from_right.data(),
                                &center_from_right_num,
                                kIpmOutputWidth,
                                kIpmOutputHeight,
                                shift_dist_px,
                                false);

    // 中线独立后处理：去近重复点、可选三角滤波、可选等距采样。
    postprocess_shifted_centerline_inplace(center_from_left.data(),
                                           &center_from_left_num,
                                           kIpmOutputWidth,
                                           kIpmOutputHeight);
    postprocess_shifted_centerline_inplace(center_from_right.data(),
                                           &center_from_right_num,
                                           kIpmOutputWidth,
                                           kIpmOutputHeight);

    fill_single_line_arrays_from_points(center_from_left.data(),
                                        center_from_left_num,
                                        g_ipm_shift_left_center_x,
                                        g_ipm_shift_left_center_y,
                                        &g_ipm_shift_left_center_count,
                                        kIpmOutputWidth,
                                        kIpmOutputHeight);
    fill_single_line_arrays_from_points(center_from_right.data(),
                                        center_from_right_num,
                                        g_ipm_shift_right_center_x,
                                        g_ipm_shift_right_center_y,
                                        &g_ipm_shift_right_center_count,
                                        kIpmOutputWidth,
                                        kIpmOutputHeight);

    std::array<maze_point_t, VISION_BOUNDARY_NUM> src_center_from_left{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> src_center_from_right{};
    const int src_center_from_left_num = transform_boundary_points_from_ipm_to_src(center_from_left.data(),
                                                                                   center_from_left_num,
                                                                                   src_center_from_left.data(),
                                                                                   static_cast<int>(src_center_from_left.size()));
    const int src_center_from_right_num = transform_boundary_points_from_ipm_to_src(center_from_right.data(),
                                                                                    center_from_right_num,
                                                                                    src_center_from_right.data(),
                                                                                    static_cast<int>(src_center_from_right.size()));
    fill_single_line_arrays_from_points(src_center_from_left.data(),
                                        src_center_from_left_num,
                                        g_src_shift_left_center_x,
                                        g_src_shift_left_center_y,
                                        &g_src_shift_left_center_count,
                                        kProcWidth,
                                        kProcHeight);
    fill_single_line_arrays_from_points(src_center_from_right.data(),
                                        src_center_from_right_num,
                                        g_src_shift_right_center_x,
                                        g_src_shift_right_center_y,
                                        &g_src_shift_right_center_count,
                                        kProcWidth,
                                        kProcHeight);

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
    update_corner_state_from_current_frame(left_first_corner_idx, right_first_corner_idx);
    // 根据需求移除“拟合中线”流程：该数组保持清空状态。
    g_ipm_center_fit_count = 0;
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
            if (y > min_y_seen + kMazeTraceYFallbackStopDelta)
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
            if (y > min_y_seen + kMazeTraceYFallbackStopDelta)
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
            if (y > min_y_seen + kMazeTraceYFallbackStopDelta)
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
            if (y > min_y_seen + kMazeTraceYFallbackStopDelta)
            {
                break;
            }
            pts[step++] = {x, y};
            turn = 0;
        }
    }

    return step;
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
    g_last_left_corner_valid = false;
    g_last_right_corner_valid = false;
    g_intersection_mode = false;
    g_intersection_probe_row = kIntersectionEntryStartRow;
    g_intersection_last_stop_row = 0;
    g_intersection_current_start_row = -1;
    g_intersection_probe_x = kProcWidth / 2;
    g_roundabout_mode.store(ROUNDABOUT_MODE_NONE);
    vision_line_error_layer_reset();
    line_error = 0;
    g_last_otsu_threshold = 127;
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
    const int search_center_x = std::clamp(previous_src_centerline_first_x(), maze_trace_x_min, maze_trace_x_max);
    int maze_start_row = std::clamp(g_maze_start_row.load(), 1, kProcHeight - 2);
    if (!g_intersection_enabled.load())
    {
        reset_intersection_mode_state();
        g_intersection_current_start_row = maze_start_row;
    }
    else
    {
        enter_intersection_mode_if_needed();
    }
    if (g_intersection_mode)
    {
        const int probe_start_row = std::clamp(g_intersection_probe_row, 1, kProcHeight - 2);
        const int stop_row = scan_intersection_stop_row(classify_img,
                                                        classify_white_threshold,
                                                        probe_start_row,
                                                        &g_intersection_probe_x,
                                                        maze_trace_x_min,
                                                        maze_trace_x_max);
        maze_start_row = std::clamp(stop_row - kIntersectionProbeRowOffset, 1, kProcHeight - 2);
        g_intersection_current_start_row = maze_start_row;
        maybe_exit_intersection_mode(stop_row);
    }
    else
    {
        g_intersection_current_start_row = maze_start_row;
    }
    int left_search_row = maze_start_row;
    int right_search_row = maze_start_row;
    const int roundabout_mode = g_roundabout_mode.load();
    if (roundabout_mode == ROUNDABOUT_MODE_LEFT_OUT)
    {
        right_search_row = std::clamp(kRoundaboutOutSearchRow, 1, kProcHeight - 2);
    }
    else if (roundabout_mode == ROUNDABOUT_MODE_RIGHT_OUT)
    {
        left_search_row = std::clamp(kRoundaboutOutSearchRow, 1, kProcHeight - 2);
    }
    bool left_ok = find_maze_start_from_row(classify_img,
                                            classify_white_threshold,
                                            true,
                                            left_search_row,
                                            maze_trace_x_min,
                                            maze_trace_x_max,
                                            &left_start_x,
                                            &left_start_y,
                                            &left_wall_is_white,
                                            search_center_x);
    bool right_ok = find_maze_start_from_row(classify_img,
                                             classify_white_threshold,
                                             false,
                                             right_search_row,
                                             maze_trace_x_min,
                                             maze_trace_x_max,
                                             &right_start_x,
                                             &right_start_y,
                                             &right_wall_is_white,
                                             search_center_x);
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
        apply_roundabout_shift_distance_override();
        render_ipm_boundary_image_and_update_boundaries(left_pts.data(), left_num, right_pts.data(), right_num);
        enter_roundabout_mode_if_needed();
        update_roundabout_mode_after_current_frame(left_ok && left_num > 0, right_ok && right_num > 0);

        const int roundabout_mode = g_roundabout_mode.load();
        const bool force_use_right = roundabout_mode_forces_right_centerline(roundabout_mode);
        const bool force_use_left = roundabout_mode_forces_left_centerline(roundabout_mode);
        const int left_count = g_ipm_shift_left_center_count;
        const int right_count = g_ipm_shift_right_center_count;
        bool use_right = false;
        if (left_count <= 0 && right_count <= 0)
        {
            use_right = false;
        }
        else if (force_use_right)
        {
            use_right = (right_count > 0) || (left_count <= 0);
        }
        else if (force_use_left)
        {
            use_right = !(left_count > 0) && (right_count > 0);
        }
        else if (left_count <= 0)
        {
            use_right = true;
        }
        else if (right_count <= 0)
        {
            use_right = false;
        }
        else
        {
            use_right = (right_count > left_count);
        }

        const uint16 *sel_ipm_x = use_right ? g_ipm_shift_right_center_x : g_ipm_shift_left_center_x;
        const uint16 *sel_ipm_y = use_right ? g_ipm_shift_right_center_y : g_ipm_shift_left_center_y;
        const int sel_ipm_count = use_right ? right_count : left_count;
        const uint16 *sel_src_x = use_right ? g_src_shift_right_center_x : g_src_shift_left_center_x;
        const uint16 *sel_src_y = use_right ? g_src_shift_right_center_y : g_src_shift_left_center_y;
        const int sel_src_count = use_right ? g_src_shift_right_center_count : g_src_shift_left_center_count;

        vision_line_error_layer_set_source(use_right ? static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT)
                                                     : static_cast<int>(VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT));
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

void vision_image_processor_set_ipm_min_point_dist_filter_enabled(bool enabled)
{
    g_ipm_min_point_dist_filter_enabled.store(enabled);
}

bool vision_image_processor_ipm_min_point_dist_filter_enabled()
{
    return g_ipm_min_point_dist_filter_enabled.load();
}

void vision_image_processor_set_ipm_min_point_dist_px(float dist_px)
{
    g_ipm_min_point_dist_px.store(std::max(0.0f, dist_px));
}

float vision_image_processor_ipm_min_point_dist_px()
{
    return g_ipm_min_point_dist_px.load();
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

void vision_image_processor_set_ipm_centerline_min_point_dist_px(float dist_px)
{
    g_ipm_centerline_min_point_dist_px.store(std::max(0.0f, dist_px));
}

float vision_image_processor_ipm_centerline_min_point_dist_px()
{
    return g_ipm_centerline_min_point_dist_px.load();
}

void vision_image_processor_set_ipm_line_error_source(vision_ipm_line_error_source_enum source)
{
    vision_line_error_layer_set_source(static_cast<int>(source));
}

vision_ipm_line_error_source_enum vision_image_processor_ipm_line_error_source()
{
    return (vision_line_error_layer_source() == static_cast<int>(VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT))
               ? VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT
               : VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT;
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
    vision_line_error_layer_get_selected_centerline_curvature(curvature, count);
}

void vision_image_processor_get_ipm_curvature_lookahead_debug(float *speed_v,
                                                              float *k_eff,
                                                              float *eta,
                                                              int *lookahead_index)
{
    vision_line_error_layer_get_curvature_lookahead_debug(speed_v, k_eff, eta, lookahead_index);
}

void vision_image_processor_get_ipm_curvature_weighted_error_debug(float *weighted_error,
                                                                   bool *lookahead_point_valid,
                                                                   int *lookahead_point_x,
                                                                   int *lookahead_point_y)
{
    vision_line_error_layer_get_curvature_weighted_error_debug(weighted_error,
                                                               lookahead_point_valid,
                                                               lookahead_point_x,
                                                               lookahead_point_y);
}

void vision_image_processor_get_ipm_curvature_speed_limit_debug(float *kappa_max,
                                                                float *delta_kappa_max,
                                                                float *curve_base_speed,
                                                                float *v_curve_raw,
                                                                float *v_curve_after_dkappa,
                                                                float *v_error_limit,
                                                                float *v_target)
{
    vision_line_error_layer_get_curvature_speed_limit_debug(kappa_max,
                                                            delta_kappa_max,
                                                            curve_base_speed,
                                                            v_curve_raw,
                                                            v_curve_after_dkappa,
                                                            v_error_limit,
                                                            v_target);
}

int vision_image_processor_ipm_weighted_first_point_error()
{
    return vision_line_error_layer_weighted_first_point_error();
}

int vision_image_processor_ipm_weighted_current_spacing()
{
    return vision_line_error_layer_weighted_current_spacing();
}

void vision_image_processor_get_ipm_weighted_decision_point(bool *valid, int *x, int *y)
{
    vision_line_error_layer_get_ipm_weighted_decision_point(valid, x, y);
}

void vision_image_processor_get_src_weighted_decision_point(bool *valid, int *x, int *y)
{
    vision_line_error_layer_get_src_weighted_decision_point(valid, x, y);
}

void vision_image_processor_get_intersection_mode_state(bool *enabled, int *stop_row, int *current_start_row)
{
    if (enabled) *enabled = g_intersection_mode;
    if (stop_row) *stop_row = g_intersection_last_stop_row;
    if (current_start_row) *current_start_row = g_intersection_current_start_row;
}

int vision_image_processor_roundabout_mode()
{
    return g_roundabout_mode.load();
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

void vision_image_processor_get_ipm_boundaries_raw(uint16 **x1, uint16 **x2, uint16 **x3,
                                                   uint16 **y1, uint16 **y2, uint16 **y3,
                                                   uint16 *dot_num)
{
    if (x1) *x1 = g_ipm_raw_xy_x1_boundary;
    if (x2) *x2 = g_ipm_raw_xy_x2_boundary;
    if (x3) *x3 = g_ipm_raw_xy_x3_boundary;
    if (y1) *y1 = g_ipm_raw_xy_y1_boundary;
    if (y2) *y2 = g_ipm_raw_xy_y2_boundary;
    if (y3) *y3 = g_ipm_raw_xy_y3_boundary;
    if (dot_num) *dot_num = static_cast<uint16>(g_ipm_raw_boundary_count);
}

void vision_image_processor_get_ipm_raw_boundary_side_counts(uint16 *left_dot_num, uint16 *right_dot_num)
{
    if (left_dot_num) *left_dot_num = static_cast<uint16>(g_ipm_raw_boundary_left_count);
    if (right_dot_num) *right_dot_num = static_cast<uint16>(g_ipm_raw_boundary_right_count);
}

void vision_image_processor_get_ipm_corner_debug_left(uint16 **x, uint16 **y, float **raw_value, float **nms_value, uint16 *dot_num)
{
    if (x) *x = g_ipm_corner_left_x;
    if (y) *y = g_ipm_corner_left_y;
    if (raw_value) *raw_value = g_ipm_corner_left_raw_value;
    if (nms_value) *nms_value = g_ipm_corner_left_value;
    if (dot_num) *dot_num = static_cast<uint16>(g_ipm_corner_left_count);
}

void vision_image_processor_get_ipm_corner_debug_right(uint16 **x, uint16 **y, float **raw_value, float **nms_value, uint16 *dot_num)
{
    if (x) *x = g_ipm_corner_right_x;
    if (y) *y = g_ipm_corner_right_y;
    if (raw_value) *raw_value = g_ipm_corner_right_raw_value;
    if (nms_value) *nms_value = g_ipm_corner_right_value;
    if (dot_num) *dot_num = static_cast<uint16>(g_ipm_corner_right_count);
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

void vision_image_processor_get_ipm_fitted_centerline(uint16 **x, uint16 **y, uint16 *dot_num)
{
    if (x) *x = g_ipm_center_fit_x;
    if (y) *y = g_ipm_center_fit_y;
    if (dot_num) *dot_num = static_cast<uint16>(g_ipm_center_fit_count);
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
