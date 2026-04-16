#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_frame_capture.h"
#include "driver/vision/vision_line_error_layer.h"
#include "driver/vision/vision_route_state_machine.h"

#include "app/motor_thread/motor_thread.h"

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
static constexpr int kBinaryMorphWhiteHigh = 255 * 5;
static constexpr int kBinaryMorphWhiteLow = 255 * 2;
static constexpr int kInitialFrameWallKeepMaxYSpan = 15;
static constexpr int kCrossLowerFitPointCount = 10;
static constexpr int kCrossLowerFitMinPointCount = 2;

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
// 八邻域原始轮廓点与每一步生长方向：dir 与同索引轮廓点对齐，0..7 为 seeds 索引，255 表示无后继方向。
static uint16 g_eight_left_trace_x[VISION_BOUNDARY_NUM];
static uint16 g_eight_left_trace_y[VISION_BOUNDARY_NUM];
static uint8 g_eight_left_trace_dir[VISION_BOUNDARY_NUM];
static uint16 g_eight_right_trace_x[VISION_BOUNDARY_NUM];
static uint16 g_eight_right_trace_y[VISION_BOUNDARY_NUM];
static uint8 g_eight_right_trace_dir[VISION_BOUNDARY_NUM];
static int g_eight_left_trace_count = 0;
static int g_eight_right_trace_count = 0;
static int g_eight_left_first_frame_touch_index = -1;
static int g_eight_right_first_frame_touch_index = -1;
// 八邻域 dir 十字下角点检测结果：角点取在 4/5 -> 2 跳变过程附近。
static std::atomic<bool> g_cross_lower_left_corner_found(false);
static std::atomic<bool> g_cross_lower_right_corner_found(false);
static std::atomic<bool> g_cross_lower_corner_pair_valid(false);
static std::atomic<int> g_cross_lower_left_corner_index(-1);
static std::atomic<int> g_cross_lower_right_corner_index(-1);
static std::atomic<int> g_cross_lower_left_corner_x(0);
static std::atomic<int> g_cross_lower_left_corner_y(0);
static std::atomic<int> g_cross_lower_right_corner_x(0);
static std::atomic<int> g_cross_lower_right_corner_y(0);
static std::atomic<int> g_cross_left_corner_post_frame_wall_rows(0);
static std::atomic<int> g_cross_right_corner_post_frame_wall_rows(0);
static std::atomic<int> g_cross_start_boundary_gap_x(0);
static std::atomic<bool> g_cross_left_aux_found(false);
static std::atomic<bool> g_cross_right_aux_found(false);
static std::atomic<int> g_cross_left_aux_transition_x(0);
static std::atomic<int> g_cross_left_aux_transition_y(0);
static std::atomic<int> g_cross_right_aux_transition_x(0);
static std::atomic<int> g_cross_right_aux_transition_y(0);
static uint16 g_cross_left_aux_trace_x[VISION_BOUNDARY_NUM];
static uint16 g_cross_left_aux_trace_y[VISION_BOUNDARY_NUM];
static uint8 g_cross_left_aux_trace_dir[VISION_BOUNDARY_NUM];
static uint16 g_cross_right_aux_trace_x[VISION_BOUNDARY_NUM];
static uint16 g_cross_right_aux_trace_y[VISION_BOUNDARY_NUM];
static uint8 g_cross_right_aux_trace_dir[VISION_BOUNDARY_NUM];
static uint16 g_cross_left_aux_regular_x[VISION_BOUNDARY_NUM];
static uint16 g_cross_left_aux_regular_y[VISION_BOUNDARY_NUM];
static uint16 g_cross_right_aux_regular_x[VISION_BOUNDARY_NUM];
static uint16 g_cross_right_aux_regular_y[VISION_BOUNDARY_NUM];
static uint16 g_cross_left_aux_trace_count = 0;
static uint16 g_cross_right_aux_trace_count = 0;
static uint16 g_cross_left_aux_regular_count = 0;
static uint16 g_cross_right_aux_regular_count = 0;
static std::atomic<bool> g_cross_left_upper_corner_found(false);
static std::atomic<bool> g_cross_right_upper_corner_found(false);
static std::atomic<int> g_cross_left_upper_corner_index(-1);
static std::atomic<int> g_cross_right_upper_corner_index(-1);
static std::atomic<int> g_cross_left_upper_corner_x(0);
static std::atomic<int> g_cross_left_upper_corner_y(0);
static std::atomic<int> g_cross_right_upper_corner_x(0);
static std::atomic<int> g_cross_right_upper_corner_y(0);
static std::atomic<bool> g_src_left_trace_has_frame_wall(false);
static std::atomic<bool> g_src_right_trace_has_frame_wall(false);
static std::atomic<int> g_src_left_start_frame_wall_rows(0);
static std::atomic<int> g_src_right_start_frame_wall_rows(0);
static std::atomic<bool> g_src_left_boundary_straight_detected(false);
static std::atomic<bool> g_src_right_boundary_straight_detected(false);
static uint16 g_src_left_circle_guide_x[VISION_BOUNDARY_NUM];
static uint16 g_src_left_circle_guide_y[VISION_BOUNDARY_NUM];
static uint16 g_src_right_circle_guide_x[VISION_BOUNDARY_NUM];
static uint16 g_src_right_circle_guide_y[VISION_BOUNDARY_NUM];
static uint16 g_src_left_circle_guide_count = 0;
static uint16 g_src_right_circle_guide_count = 0;
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
// - g_image_bgr/g_image_gray/g_image_binary_u8: 处理分辨率图像。
static uint8 g_image_bgr_full[UVC_HEIGHT * UVC_WIDTH * 3];
static uint8 g_image_bgr[kProcHeight * kProcWidth * 3];
static uint8 g_image_gray[kProcHeight * kProcWidth];
static uint8 g_image_binary_u8[kProcHeight * kProcWidth];
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
static std::atomic<int> g_zebra_cross_count(0);

static int count_gray_row_threshold_jumps(const uint8 *gray, int width, int height, int y, int step, int abs_diff_threshold)
{
    if (gray == nullptr || width <= 0 || height <= 0 || step <= 0)
    {
        return 0;
    }
    const int row = std::clamp(y, 0, height - 1);
    const int row_offset = row * width;
    int prev_x = 0;
    int jump_count = 0;
    for (int x = step; x < width; x += step)
    {
        const int prev_val = static_cast<int>(gray[row_offset + prev_x]);
        const int cur_val = static_cast<int>(gray[row_offset + x]);
        if (std::abs(cur_val - prev_val) > abs_diff_threshold)
        {
            jump_count += 1;
        }
        prev_x = x;
    }
    return jump_count;
}
// 成功处理完一帧视觉结果后递增，供控制层做“新样本驱动”的滤波推进。
static std::atomic<uint32> g_processed_frame_seq(0);
// 记录“最近一次成功识别”的左右起点 x，用于下一帧起点搜索偏移。
static int g_last_maze_left_start_x = -1;
static int g_last_maze_right_start_x = -1;

// 迷宫法起始搜索行（可运行时配置）。
static std::atomic<int> g_maze_start_row(g_vision_runtime_config.maze_start_row);
static std::atomic<int> g_maze_trace_method(g_vision_runtime_config.maze_trace_method);
static std::atomic<int> g_maze_trace_y_fallback_stop_delta(g_vision_runtime_config.maze_trace_y_fallback_stop_delta);
static std::atomic<bool> g_cross_lower_corner_dir_enabled(g_vision_runtime_config.cross_lower_corner_dir_enabled);
static std::atomic<int> g_cross_lower_corner_pre_window(g_vision_runtime_config.cross_lower_corner_pre_window);
static std::atomic<int> g_cross_lower_corner_post_window(g_vision_runtime_config.cross_lower_corner_post_window);
static std::atomic<int> g_cross_lower_corner_pre_min_votes(g_vision_runtime_config.cross_lower_corner_pre_min_votes);
static std::atomic<int> g_cross_lower_corner_post_min_votes(g_vision_runtime_config.cross_lower_corner_post_min_votes);
static std::atomic<int> g_cross_lower_corner_transition_max_len(g_vision_runtime_config.cross_lower_corner_transition_max_len);
static std::atomic<int> g_cross_lower_corner_pair_y_diff_max(g_vision_runtime_config.cross_lower_corner_pair_y_diff_max);
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
static std::atomic<int> g_ipm_boundary_angle_step(g_vision_runtime_config.ipm_boundary_angle_step);
static std::atomic<float> g_ipm_boundary_corner_cos_threshold(g_vision_runtime_config.ipm_boundary_corner_cos_threshold);
static std::atomic<int> g_ipm_boundary_corner_nms_radius(g_vision_runtime_config.ipm_boundary_corner_nms_radius);
static std::atomic<bool> g_ipm_boundary_truncate_at_first_corner_enabled(
    g_vision_runtime_config.ipm_boundary_truncate_at_first_corner_enabled);
static std::atomic<int> g_ipm_boundary_straight_min_points(g_vision_runtime_config.ipm_boundary_straight_min_points);
static std::atomic<int> g_ipm_boundary_straight_check_count(g_vision_runtime_config.ipm_boundary_straight_check_count);
static std::atomic<float> g_ipm_boundary_straight_min_cos(g_vision_runtime_config.ipm_boundary_straight_min_cos);
static std::atomic<float> g_ipm_track_width_px(g_vision_runtime_config.ipm_track_width_px);
static std::atomic<float> g_ipm_center_target_offset_from_left_px(g_vision_runtime_config.ipm_center_target_offset_from_left_px);
// 逆透视处理中线独立配置。
static std::atomic<bool> g_ipm_centerline_postprocess_enabled(g_vision_runtime_config.ipm_centerline_postprocess_enabled);
static std::atomic<bool> g_ipm_centerline_triangle_filter_enabled(g_vision_runtime_config.ipm_centerline_triangle_filter_enabled);
static std::atomic<bool> g_ipm_centerline_resample_enabled(g_vision_runtime_config.ipm_centerline_resample_enabled);
static std::atomic<float> g_ipm_centerline_resample_step_px(g_vision_runtime_config.ipm_centerline_resample_step_px);
static std::atomic<bool> g_ipm_centerline_curvature_enabled(g_vision_runtime_config.ipm_centerline_curvature_enabled);
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

struct cross_lower_corner_detection_t
{
    bool found = false;
    int index = -1;
    maze_point_t point{0, 0};
};

static void fill_boundary_arrays_from_maze(const maze_point_t *left_pts,
                                           int left_num,
                                           const maze_point_t *right_pts,
                                           int right_num);
static int copy_boundary_points(const maze_point_t *src, int src_num, maze_point_t *dst, int max_dst);
static int extract_one_point_per_row_from_contour(const maze_point_t *raw_pts,
                                                  int raw_num,
                                                  bool is_left,
                                                  bool clip_artificial_frame,
                                                  maze_point_t *regular_pts,
                                                  int max_regular_pts);
static void prepend_ipm_bottom_midpoint_bridge_inplace(maze_point_t *pts,
                                                       int *num,
                                                       int width,
                                                       int height,
                                                       float step_px);
static int previous_src_centerline_first_x();
static inline bool pixel_is_white(const uint8 *img, int x, int y, uint8 white_threshold);
static void filter_binary_image_inplace(uint8 *binary_img);
static void draw_binary_black_frame(uint8 *binary_img);
static void resample_boundary_points_equal_spacing_inplace(maze_point_t *pts, int *num, int width, int height, float step_px);
static void clear_eight_neighbor_trace_cache();
static void save_eight_neighbor_trace_cache(bool is_left, const maze_point_t *pts, const uint8 *dirs, int count, int first_frame_touch_index);
static void clear_cross_lower_corner_detection_cache();
static void clear_cross_aux_cache();
static cross_lower_corner_detection_t detect_cross_lower_corner_from_dirs(const maze_point_t *pts, const uint8 *dirs, int count);
static void update_cross_lower_corner_detection_cache(const maze_point_t *left_pts, const uint8 *left_dirs, int left_count,
                                                      const maze_point_t *right_pts, const uint8 *right_dirs, int right_count);
static void update_src_trace_frame_wall_cache(const maze_point_t *left_trace_pts,
                                              int left_trace_num,
                                              const maze_point_t *right_trace_pts,
                                              int right_trace_num);
static void clear_src_circle_guide_cache();
static void fill_single_line_arrays_from_points(const maze_point_t *pts,
                                                int num,
                                                uint16 *xs,
                                                uint16 *ys,
                                                int *count,
                                                int width,
                                                int height);
static bool detect_src_straight_boundary_from_dirs(const uint8 *dirs, int count);
static int count_side_frame_wall_rows_after_index(const maze_point_t *pts, int count, int start_index, bool is_left);
static bool find_vertical_white_to_black_transition(const uint8 *classify_img,
                                                    uint8 white_threshold,
                                                    int x,
                                                    int start_y,
                                                    int max_scan_rows,
                                                    int *transition_x,
                                                    int *transition_y);
static void save_cross_aux_trace_cache(bool is_left, const maze_point_t *pts, const uint8 *dirs, int count);
static void save_cross_aux_regular_cache(bool is_left, const maze_point_t *pts, int count);
static bool build_cross_aux_boundary(const uint8 *classify_img,
                                     uint8 white_threshold,
                                     bool wall_is_white,
                                     bool is_left,
                                     int corner_x,
                                     int corner_y,
                                     int x_min,
                                     int x_max,
                                     maze_point_t *raw_pts,
                                     uint8 *raw_dirs,
                                     int *raw_count,
                                     maze_point_t *regular_pts,
                                     int *regular_count,
                                     maze_point_t *transition_point);
static int find_cross_upper_record_index_from_dirs(const uint8 *dirs, int count);
static bool find_cross_upper_corner_from_trace(const maze_point_t *trace_pts,
                                               const uint8 *trace_dirs,
                                               int trace_count,
                                               bool is_left,
                                               maze_point_t *corner_point,
                                               int *corner_index);
static bool find_nth_dir_point_on_trace(const maze_point_t *trace_pts,
                                        const uint8 *trace_dirs,
                                        int trace_count,
                                        uint8 target_dir,
                                        int nth_match,
                                        maze_point_t *corner_point,
                                        int *corner_index);
static int truncate_regular_boundary_before_point_inplace(maze_point_t *pts,
                                                          int count,
                                                          const maze_point_t &corner_point);
static int concatenate_boundary_segments(const maze_point_t *segment_a,
                                         int count_a,
                                         const maze_point_t *segment_b,
                                         int count_b,
                                         const maze_point_t *segment_c,
                                         int count_c,
                                         maze_point_t *out_pts,
                                         int max_out_pts);
static bool point_touches_artificial_frame(int x, int y);
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
static int trace_boundary_eight_neighbor(const uint8 *classify_img,
                                         uint8 white_threshold,
                                         bool wall_is_white,
                                         int start_x,
                                         int start_y,
                                         int y_min,
                                         int x_min,
                                         int x_max,
                                         bool prefer_left_bias,
                                         maze_point_t *pts,
                                         uint8 *dirs,
                                         int max_pts,
                                         int *first_frame_touch_index);
static int trace_left_boundary_selected_method(const uint8 *classify_img,
                                               uint8 white_threshold,
                                               bool wall_is_white,
                                               int start_x,
                                               int start_y,
                                               int y_min,
                                               int x_min,
                                               int x_max,
                                               maze_point_t *pts,
                                               uint8 *dirs,
                                               int max_pts,
                                               int *first_frame_touch_index);
static int trace_right_boundary_selected_method(const uint8 *classify_img,
                                                uint8 white_threshold,
                                                bool wall_is_white,
                                                int start_x,
                                                int start_y,
                                                int y_min,
                                                int x_min,
                                                int x_max,
                                                maze_point_t *pts,
                                                uint8 *dirs,
                                                int max_pts,
                                                int *first_frame_touch_index);
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
static int truncate_boundary_at_cross_lower_corner_inplace(maze_point_t *pts,
                                                           int num,
                                                           int corner_x,
                                                           int corner_y,
                                                           int max_pts);
static bool find_raw_boundary_x_at_row(const maze_point_t *pts, int count, int target_y, int *x_out);
static bool circle_entry_raw_boundary_gap_ready(const maze_point_t *left_raw_pts,
                                                int left_raw_num,
                                                const maze_point_t *right_raw_pts,
                                                int right_raw_num,
                                                int corner_y);
static bool find_circle_stage3_target_on_regular_boundary(const maze_point_t *pts,
                                                          int count,
                                                          bool is_left,
                                                          int target_offset,
                                                          maze_point_t *target_out);

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
static constexpr int kEightNeighborLeftSeeds[8][2] = {
    {0, 1},
    {-1, 1},
    {-1, 0},
    {-1, -1},
    {0, -1},
    {1, -1},
    {1, 0},
    {1, 1}
};
static constexpr int kEightNeighborRightSeeds[8][2] = {
    {0, 1},
    {1, 1},
    {1, 0},
    {1, -1},
    {0, -1},
    {-1, -1},
    {-1, 0},
    {-1, 1}
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
    std::fill_n(g_ipm_shift_left_center_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_shift_left_center_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_shift_right_center_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_ipm_shift_right_center_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    g_ipm_boundary_count = 0;
    g_ipm_boundary_left_count = 0;
    g_ipm_boundary_right_count = 0;
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

static void filter_binary_image_inplace(uint8 *binary_img)
{
    if (binary_img == nullptr)
    {
        return;
    }

    for (int y = 1; y < kProcHeight - 1; ++y)
    {
        for (int x = 1; x < kProcWidth - 1; ++x)
        {
            const int idx = y * kProcWidth + x;
            const int neighbor_sum =
                binary_img[(y - 1) * kProcWidth + (x - 1)] +
                binary_img[(y - 1) * kProcWidth + x] +
                binary_img[(y - 1) * kProcWidth + (x + 1)] +
                binary_img[y * kProcWidth + (x - 1)] +
                binary_img[y * kProcWidth + (x + 1)] +
                binary_img[(y + 1) * kProcWidth + (x - 1)] +
                binary_img[(y + 1) * kProcWidth + x] +
                binary_img[(y + 1) * kProcWidth + (x + 1)];

            if (neighbor_sum >= kBinaryMorphWhiteHigh && binary_img[idx] == 0)
            {
                binary_img[idx] = 255;
            }
            else if (neighbor_sum <= kBinaryMorphWhiteLow && binary_img[idx] == 255)
            {
                binary_img[idx] = 0;
            }
        }
    }
}

static void draw_binary_black_frame(uint8 *binary_img)
{
    if (binary_img == nullptr)
    {
        return;
    }

    for (int y = 0; y < kProcHeight; ++y)
    {
        binary_img[y * kProcWidth + 0] = 0;
        binary_img[y * kProcWidth + 1] = 0;
        binary_img[y * kProcWidth + (kProcWidth - 2)] = 0;
        binary_img[y * kProcWidth + (kProcWidth - 1)] = 0;
    }
    for (int x = 0; x < kProcWidth; ++x)
    {
        binary_img[x] = 0;
        binary_img[kProcWidth + x] = 0;
    }
}

static void clear_eight_neighbor_trace_cache()
{
    g_eight_left_trace_count = 0;
    g_eight_right_trace_count = 0;
    g_eight_left_first_frame_touch_index = -1;
    g_eight_right_first_frame_touch_index = -1;
    std::fill_n(g_eight_left_trace_dir, VISION_BOUNDARY_NUM, static_cast<uint8>(255));
    std::fill_n(g_eight_right_trace_dir, VISION_BOUNDARY_NUM, static_cast<uint8>(255));
}

static void save_eight_neighbor_trace_cache(bool is_left,
                                            const maze_point_t *pts,
                                            const uint8 *dirs,
                                            int count,
                                            int first_frame_touch_index)
{
    uint16 *xs = is_left ? g_eight_left_trace_x : g_eight_right_trace_x;
    uint16 *ys = is_left ? g_eight_left_trace_y : g_eight_right_trace_y;
    uint8 *out_dirs = is_left ? g_eight_left_trace_dir : g_eight_right_trace_dir;
    int *out_count = is_left ? &g_eight_left_trace_count : &g_eight_right_trace_count;
    int *out_touch = is_left ? &g_eight_left_first_frame_touch_index : &g_eight_right_first_frame_touch_index;

    const int safe_count = (pts && dirs) ? std::clamp(count, 0, VISION_BOUNDARY_NUM) : 0;
    for (int i = 0; i < safe_count; ++i)
    {
        xs[i] = static_cast<uint16>(std::clamp(pts[i].x, 0, kProcWidth - 1));
        ys[i] = static_cast<uint16>(std::clamp(pts[i].y, 0, kProcHeight - 1));
        out_dirs[i] = dirs[i];
    }
    if (safe_count < VISION_BOUNDARY_NUM)
    {
        std::fill_n(out_dirs + safe_count, VISION_BOUNDARY_NUM - safe_count, static_cast<uint8>(255));
    }

    *out_count = safe_count;
    *out_touch = first_frame_touch_index;
}

static void clear_cross_lower_corner_detection_cache()
{
    g_cross_lower_left_corner_found.store(false);
    g_cross_lower_right_corner_found.store(false);
    g_cross_lower_corner_pair_valid.store(false);
    g_cross_lower_left_corner_index.store(-1);
    g_cross_lower_right_corner_index.store(-1);
    g_cross_lower_left_corner_x.store(0);
    g_cross_lower_left_corner_y.store(0);
    g_cross_lower_right_corner_x.store(0);
    g_cross_lower_right_corner_y.store(0);
    g_cross_left_corner_post_frame_wall_rows.store(0);
    g_cross_right_corner_post_frame_wall_rows.store(0);
    g_cross_start_boundary_gap_x.store(0);
}

static void clear_cross_aux_cache()
{
    g_cross_left_aux_found.store(false);
    g_cross_right_aux_found.store(false);
    g_cross_left_aux_transition_x.store(0);
    g_cross_left_aux_transition_y.store(0);
    g_cross_right_aux_transition_x.store(0);
    g_cross_right_aux_transition_y.store(0);
    g_cross_left_upper_corner_found.store(false);
    g_cross_right_upper_corner_found.store(false);
    g_cross_left_upper_corner_index.store(-1);
    g_cross_right_upper_corner_index.store(-1);
    g_cross_left_upper_corner_x.store(0);
    g_cross_left_upper_corner_y.store(0);
    g_cross_right_upper_corner_x.store(0);
    g_cross_right_upper_corner_y.store(0);
    std::fill_n(g_cross_left_aux_trace_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_cross_left_aux_trace_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_cross_left_aux_trace_dir, VISION_BOUNDARY_NUM, static_cast<uint8>(255));
    std::fill_n(g_cross_right_aux_trace_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_cross_right_aux_trace_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_cross_right_aux_trace_dir, VISION_BOUNDARY_NUM, static_cast<uint8>(255));
    std::fill_n(g_cross_left_aux_regular_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_cross_left_aux_regular_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_cross_right_aux_regular_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_cross_right_aux_regular_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    g_cross_left_aux_trace_count = 0;
    g_cross_right_aux_trace_count = 0;
    g_cross_left_aux_regular_count = 0;
    g_cross_right_aux_regular_count = 0;
}

static void clear_src_circle_guide_cache()
{
    std::fill_n(g_src_left_circle_guide_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_src_left_circle_guide_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_src_right_circle_guide_x, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    std::fill_n(g_src_right_circle_guide_y, VISION_BOUNDARY_NUM, static_cast<uint16>(0));
    g_src_left_circle_guide_count = 0;
    g_src_right_circle_guide_count = 0;
}

static void save_cross_aux_trace_cache(bool is_left, const maze_point_t *pts, const uint8 *dirs, int count)
{
    uint16 *xs = is_left ? g_cross_left_aux_trace_x : g_cross_right_aux_trace_x;
    uint16 *ys = is_left ? g_cross_left_aux_trace_y : g_cross_right_aux_trace_y;
    uint8 *out_dirs = is_left ? g_cross_left_aux_trace_dir : g_cross_right_aux_trace_dir;
    uint16 *out_count = is_left ? &g_cross_left_aux_trace_count : &g_cross_right_aux_trace_count;

    const int safe_count = (pts && dirs) ? std::clamp(count, 0, VISION_BOUNDARY_NUM) : 0;
    for (int i = 0; i < safe_count; ++i)
    {
        xs[i] = static_cast<uint16>(std::clamp(pts[i].x, 0, kProcWidth - 1));
        ys[i] = static_cast<uint16>(std::clamp(pts[i].y, 0, kProcHeight - 1));
        out_dirs[i] = dirs[i];
    }
    if (safe_count < VISION_BOUNDARY_NUM)
    {
        std::fill_n(out_dirs + safe_count, VISION_BOUNDARY_NUM - safe_count, static_cast<uint8>(255));
    }
    *out_count = static_cast<uint16>(safe_count);
}

static void save_cross_aux_regular_cache(bool is_left, const maze_point_t *pts, int count)
{
    uint16 *xs = is_left ? g_cross_left_aux_regular_x : g_cross_right_aux_regular_x;
    uint16 *ys = is_left ? g_cross_left_aux_regular_y : g_cross_right_aux_regular_y;
    uint16 *out_count = is_left ? &g_cross_left_aux_regular_count : &g_cross_right_aux_regular_count;
    int cached_count = 0;
    fill_single_line_arrays_from_points(pts, count, xs, ys, &cached_count, kProcWidth, kProcHeight);
    *out_count = static_cast<uint16>(std::clamp(cached_count, 0, VISION_BOUNDARY_NUM));
}

static void build_trace_frame_wall_row_mask(const maze_point_t *pts,
                                            int count,
                                            std::array<uint8, kProcHeight> *row_mask,
                                            bool *has_frame_wall)
{
    if (row_mask == nullptr)
    {
        return;
    }

    row_mask->fill(0);
    bool any = false;
    if (pts != nullptr && count > 0)
    {
        const int safe_count = std::clamp(count, 0, VISION_BOUNDARY_NUM);
        for (int i = 0; i < safe_count; ++i)
        {
            if (!point_touches_artificial_frame(pts[i].x, pts[i].y))
            {
                continue;
            }

            const int y = std::clamp(pts[i].y, 0, kProcHeight - 1);
            (*row_mask)[y] = 1;
            any = true;
        }
    }

    if (has_frame_wall != nullptr)
    {
        *has_frame_wall = any;
    }
}

static void update_src_trace_frame_wall_cache(const maze_point_t *left_trace_pts,
                                              int left_trace_num,
                                              const maze_point_t *right_trace_pts,
                                              int right_trace_num)
{
    std::array<uint8, kProcHeight> left_row_mask{};
    std::array<uint8, kProcHeight> right_row_mask{};
    bool left_has_frame_wall = false;
    bool right_has_frame_wall = false;
    build_trace_frame_wall_row_mask(left_trace_pts, left_trace_num, &left_row_mask, &left_has_frame_wall);
    build_trace_frame_wall_row_mask(right_trace_pts, right_trace_num, &right_row_mask, &right_has_frame_wall);

    g_src_left_trace_has_frame_wall.store(left_has_frame_wall);
    g_src_right_trace_has_frame_wall.store(right_has_frame_wall);
}

static inline bool cross_lower_pre_dir(uint8 dir)
{
    return dir == 4 || dir == 5 || dir == 6;
}

static inline bool cross_lower_transition_dir(uint8 dir)
{
    return dir >= 1 && dir <= 6;
}

static inline bool cross_lower_post_dir(uint8 dir)
{
    return dir == 1 || dir == 2 || dir == 3;
}

static bool cross_lower_match_adjacent_platform(const uint8 *dirs,
                                                int start,
                                                int end,
                                                uint8 pair1_low,
                                                uint8 pair1_high,
                                                uint8 pair2_low,
                                                uint8 pair2_high,
                                                int need_votes)
{
    if (dirs == nullptr || start < 0 || end <= start)
    {
        return false;
    }

    int votes = 0;
    uint8 min_dir = 255;
    uint8 max_dir = 0;
    for (int i = start; i < end; ++i)
    {
        const uint8 dir = dirs[i];
        const bool valid =
            ((dir >= pair1_low && dir <= pair1_high) ||
             (dir >= pair2_low && dir <= pair2_high));
        if (!valid)
        {
            continue;
        }

        ++votes;
        min_dir = std::min(min_dir, dir);
        max_dir = std::max(max_dir, dir);
    }

    if (votes < need_votes || min_dir == 255)
    {
        return false;
    }

    if ((max_dir - min_dir) > 1)
    {
        return false;
    }

    return ((min_dir >= pair1_low && max_dir <= pair1_high) ||
            (min_dir >= pair2_low && max_dir <= pair2_high));
}

static bool cross_lower_compute_adjacent_platform_mean(const uint8 *dirs,
                                                       int start,
                                                       int end,
                                                       uint8 pair1_low,
                                                       uint8 pair1_high,
                                                       uint8 pair2_low,
                                                       uint8 pair2_high,
                                                       float *mean_out)
{
    if (dirs == nullptr || mean_out == nullptr || start < 0 || end <= start)
    {
        return false;
    }

    int valid_count = 0;
    int sum = 0;
    for (int i = start; i < end; ++i)
    {
        const uint8 dir = dirs[i];
        const bool valid =
            ((dir >= pair1_low && dir <= pair1_high) ||
             (dir >= pair2_low && dir <= pair2_high));
        if (!valid)
        {
            continue;
        }
        sum += static_cast<int>(dir);
        ++valid_count;
    }

    if (valid_count <= 0)
    {
        return false;
    }

    *mean_out = static_cast<float>(sum) / static_cast<float>(valid_count);
    return true;
}

static int count_leading_side_frame_wall_rows(const maze_point_t *pts, int count, bool is_left)
{
    if (pts == nullptr || count <= 0)
    {
        return 0;
    }

    int rows = 0;
    int prev_y = std::numeric_limits<int>::max();
    for (int i = 0; i < count; ++i)
    {
        const int x = pts[i].x;
        const int y = pts[i].y;
        const bool on_side_frame = is_left ? (x <= 1) : (x >= (kProcWidth - 2));
        if (!on_side_frame)
        {
            break;
        }
        if (y == prev_y)
        {
            continue;
        }
        ++rows;
        prev_y = y;
    }

    return rows;
}

static int count_side_frame_wall_rows_after_index(const maze_point_t *pts, int count, int start_index, bool is_left)
{
    if (pts == nullptr || count <= 0 || start_index < 0 || start_index >= count)
    {
        return 0;
    }

    int current_rows = 0;
    int max_rows = 0;
    int prev_y = std::numeric_limits<int>::max();
    for (int i = start_index + 1; i < count; ++i)
    {
        const int x = pts[i].x;
        const int y = pts[i].y;
        const bool on_side_frame = is_left ? (x <= 1) : (x >= (kProcWidth - 2));
        if (!on_side_frame)
        {
            max_rows = std::max(max_rows, current_rows);
            current_rows = 0;
            prev_y = std::numeric_limits<int>::max();
            continue;
        }
        if (y == prev_y)
        {
            continue;
        }
        ++current_rows;
        prev_y = y;
    }

    return std::max(max_rows, current_rows);
}

static bool find_vertical_white_to_black_transition(const uint8 *classify_img,
                                                    uint8 white_threshold,
                                                    int x,
                                                    int start_y,
                                                    int max_scan_rows,
                                                    int *transition_x,
                                                    int *transition_y)
{
    if (classify_img == nullptr)
    {
        return false;
    }

    const int clamped_x = std::clamp(x, 1, kProcWidth - 2);
    const int begin_y = std::clamp(start_y, 1, kProcHeight - 2);
    const int scan_rows = std::max(1, max_scan_rows);
    bool prev_is_white = pixel_is_white(classify_img, clamped_x, begin_y, white_threshold);
    for (int step = 1; step <= scan_rows; ++step)
    {
        const int y = begin_y - step;
        if (y <= 0)
        {
            break;
        }
        const bool curr_is_white = pixel_is_white(classify_img, clamped_x, y, white_threshold);
        if (prev_is_white && !curr_is_white)
        {
            if (transition_x) *transition_x = clamped_x;
            if (transition_y) *transition_y = y;
            return true;
        }
        prev_is_white = curr_is_white;
    }

    return false;
}

static bool find_regular_boundary_jump_cut_point(const maze_point_t *pts,
                                                 int count,
                                                 int x_diff_threshold,
                                                 int cut_forward_points,
                                                 maze_point_t *cut_point,
                                                 int *cut_index)
{
    if (cut_point) *cut_point = maze_point_t{0, 0};
    if (cut_index) *cut_index = -1;
    if (pts == nullptr || count < 2)
    {
        return false;
    }

    const int safe_count = std::clamp(count, 0, VISION_BOUNDARY_NUM);
    for (int i = 1; i < safe_count; ++i)
    {
        const int dx = std::abs(pts[i].x - pts[i - 1].x);
        if (dx <= x_diff_threshold)
        {
            continue;
        }

        const int index = std::clamp(i + std::max(0, cut_forward_points), 0, safe_count - 1);
        if (cut_point) *cut_point = pts[index];
        if (cut_index) *cut_index = index;
        return true;
    }

    return false;
}

static bool find_raw_boundary_x_at_row(const maze_point_t *pts, int count, int target_y, int *x_out)
{
    if (pts == nullptr || count <= 0 || x_out == nullptr)
    {
        return false;
    }

    const int safe_count = std::clamp(count, 0, VISION_BOUNDARY_NUM);
    for (int i = 0; i < safe_count; ++i)
    {
        if (pts[i].y != target_y)
        {
            continue;
        }

        *x_out = pts[i].x;
        return true;
    }

    return false;
}

static bool circle_entry_raw_boundary_gap_ready(const maze_point_t *left_raw_pts,
                                                int left_raw_num,
                                                const maze_point_t *right_raw_pts,
                                                int right_raw_num,
                                                int corner_y)
{
    const int row_offset = std::max(g_vision_runtime_config.route_circle_entry_corner_row_offset, 0);
    const int rows_to_check = std::max(g_vision_runtime_config.route_circle_entry_gap_check_rows, 1);
    const int min_gap = g_vision_runtime_config.route_circle_entry_min_raw_boundary_gap;
    const int start_y = corner_y - row_offset;

    for (int i = 0; i < rows_to_check; ++i)
    {
        const int target_y = start_y - i;
        if (target_y <= 0 || target_y >= (kProcHeight - 1))
        {
            return false;
        }

        int left_x = 0;
        int right_x = 0;
        if (!find_raw_boundary_x_at_row(left_raw_pts, left_raw_num, target_y, &left_x) ||
            !find_raw_boundary_x_at_row(right_raw_pts, right_raw_num, target_y, &right_x))
        {
            return false;
        }

        if ((right_x - left_x) <= min_gap)
        {
            return false;
        }
    }

    return true;
}

static bool find_circle_stage3_target_on_regular_boundary(const maze_point_t *pts,
                                                          int count,
                                                          bool is_left,
                                                          int target_offset,
                                                          maze_point_t *target_out)
{
    if (pts == nullptr || count <= 0 || target_out == nullptr)
    {
        return false;
    }

    const int safe_count = std::clamp(count, 0, VISION_BOUNDARY_NUM);
    const int min_run_len = std::max(g_vision_runtime_config.circle_guide_min_frame_wall_segment_len, 6);
    bool frame_seg_found = false;
    int frame_seg_end_index = -1;
    int run_start = -1;
    for (int i = 0; i < safe_count; ++i)
    {
        const bool on_frame = is_left ? (pts[i].x <= 1) : (pts[i].x >= (kProcWidth - 2));
        if (on_frame)
        {
            if (run_start < 0)
            {
                run_start = i;
            }
            continue;
        }

        if (run_start >= 0)
        {
            const int run_len = i - run_start;
            if (run_len > g_vision_runtime_config.circle_guide_min_frame_wall_segment_len)
            {
                frame_seg_found = true;
                frame_seg_end_index = i - 1;
                break;
            }
            run_start = -1;
        }
    }
    if (!frame_seg_found && run_start >= 0)
    {
        const int run_len = safe_count - run_start;
        if (run_len > g_vision_runtime_config.circle_guide_min_frame_wall_segment_len)
        {
            frame_seg_found = true;
            frame_seg_end_index = safe_count - 1;
        }
    }

    const int search_begin = frame_seg_found ? (frame_seg_end_index + 1) : 0;

    for (int start = search_begin; start + min_run_len <= safe_count; ++start)
    {
        const bool start_on_frame = is_left ? (pts[start].x <= 1) : (pts[start].x >= (kProcWidth - 2));
        if (start_on_frame)
        {
            continue;
        }

        const int end = start + min_run_len - 1;
        const int total_dx = pts[end].x - pts[start].x;
        if ((is_left && total_dx < 4) || (!is_left && total_dx > -4))
        {
            continue;
        }

        int backward_steps = 0;
        int max_abs_residual = 0;
        bool y_monotonic = true;
        for (int i = start + 1; i <= end; ++i)
        {
            const int dx = pts[i].x - pts[i - 1].x;
            const int dy = pts[i - 1].y - pts[i].y;
            if (dy < 0 || dy > 2)
            {
                y_monotonic = false;
                break;
            }

            if ((is_left && dx < -1) || (!is_left && dx > 1))
            {
                ++backward_steps;
            }

            const float t = (pts[end].y != pts[start].y)
                                ? static_cast<float>(pts[i].y - pts[start].y) / static_cast<float>(pts[end].y - pts[start].y)
                                : 0.0f;
            const float predicted_x = static_cast<float>(pts[start].x) + t * static_cast<float>(pts[end].x - pts[start].x);
            max_abs_residual = std::max(max_abs_residual,
                                        std::abs(static_cast<int>(std::lround(predicted_x)) - pts[i].x));
        }

        if (!y_monotonic || backward_steps > 1 || max_abs_residual > 2)
        {
            continue;
        }

        *target_out = pts[start];
        return true;
    }

    int fallback_index = frame_seg_found ? (frame_seg_end_index + std::max(0, target_offset)) : 0;
    fallback_index = std::clamp(fallback_index, 0, safe_count - 1);
    while (fallback_index < safe_count)
    {
        const bool on_frame = is_left ? (pts[fallback_index].x <= 1) : (pts[fallback_index].x >= (kProcWidth - 2));
        if (!on_frame)
        {
            *target_out = pts[fallback_index];
            return true;
        }
        ++fallback_index;
    }

    *target_out = pts[safe_count - 1];
    return true;
}

typedef struct
{
    bool found;
    int start_index;
    int end_index;
} side_frame_wall_segment_t;

static side_frame_wall_segment_t find_first_side_frame_wall_segment(const maze_point_t *pts,
                                                                    int count,
                                                                    bool is_left,
                                                                    int min_len)
{
    side_frame_wall_segment_t result{};
    if (pts == nullptr || count <= 0)
    {
        return result;
    }

    int run_start = -1;
    for (int i = 0; i < count; ++i)
    {
        const bool on_side_frame = is_left ? (pts[i].x <= 1) : (pts[i].x >= (kProcWidth - 2));
        if (on_side_frame)
        {
            if (run_start < 0)
            {
                run_start = i;
            }
            continue;
        }

        if (run_start >= 0)
        {
            const int run_len = i - run_start;
            if (run_len > min_len)
            {
                result.found = true;
                result.start_index = run_start;
                result.end_index = i - 1;
                return result;
            }
            run_start = -1;
        }
    }

    if (run_start >= 0)
    {
        const int run_len = count - run_start;
        if (run_len > min_len)
        {
            result.found = true;
            result.start_index = run_start;
            result.end_index = count - 1;
        }
    }

    return result;
}

static bool find_circle_stage5_target_on_regular_boundary(const maze_point_t *pts,
                                                          int count,
                                                          bool is_left,
                                                          maze_point_t *target_out)
{
    if (pts == nullptr || count <= 0 || target_out == nullptr)
    {
        return false;
    }

    const int safe_count = std::clamp(count, 0, VISION_BOUNDARY_NUM);
    const side_frame_wall_segment_t seg = find_first_side_frame_wall_segment(pts,
                                                                             safe_count,
                                                                             is_left,
                                                                             g_vision_runtime_config.circle_guide_min_frame_wall_segment_len);
    if (seg.found && seg.end_index >= 0 && seg.end_index < safe_count)
    {
        *target_out = pts[seg.end_index];
        return true;
    }

    int last_frame_index = -1;
    for (int i = 0; i < safe_count; ++i)
    {
        const bool on_side_frame = is_left ? (pts[i].x <= 1) : (pts[i].x >= (kProcWidth - 2));
        if (on_side_frame)
        {
            last_frame_index = i;
        }
    }

    if (last_frame_index >= 0)
    {
        *target_out = pts[last_frame_index];
        return true;
    }

    return false;
}

static int build_line_points_between(const maze_point_t &a,
                                     const maze_point_t &b,
                                     maze_point_t *out_pts,
                                     int max_out_pts)
{
    if (out_pts == nullptr || max_out_pts <= 0)
    {
        return 0;
    }

    const int min_y = std::min(a.y, b.y);
    const int max_y = std::max(a.y, b.y);
    int out_num = 0;
    for (int y = max_y; y >= min_y && out_num < max_out_pts; --y)
    {
        float t = 0.0f;
        if (a.y != b.y)
        {
            t = static_cast<float>(y - a.y) / static_cast<float>(b.y - a.y);
        }
        const float xf = static_cast<float>(a.x) + t * static_cast<float>(b.x - a.x);
        out_pts[out_num++] = {std::clamp(static_cast<int>(std::lround(xf)), 0, kProcWidth - 1), y};
    }

    return out_num;
}

static int build_line_points_between_with_y_step(const maze_point_t &a,
                                                 const maze_point_t &b,
                                                 int y_step,
                                                 maze_point_t *out_pts,
                                                 int max_out_pts)
{
    if (out_pts == nullptr || max_out_pts <= 0)
    {
        return 0;
    }

    const int safe_step = std::max(1, y_step);
    const int min_y = std::min(a.y, b.y);
    const int max_y = std::max(a.y, b.y);
    int out_num = 0;
    for (int y = max_y; y >= min_y && out_num < max_out_pts; y -= safe_step)
    {
        float t = 0.0f;
        if (a.y != b.y)
        {
            t = static_cast<float>(y - a.y) / static_cast<float>(b.y - a.y);
        }
        const float xf = static_cast<float>(a.x) + t * static_cast<float>(b.x - a.x);
        out_pts[out_num++] = {std::clamp(static_cast<int>(std::lround(xf)), 0, kProcWidth - 1), y};
    }

    if (out_num < max_out_pts)
    {
        const maze_point_t end_point{std::clamp(b.x, 0, kProcWidth - 1), std::clamp(b.y, 0, kProcHeight - 1)};
        if (out_num <= 0 || out_pts[out_num - 1].x != end_point.x || out_pts[out_num - 1].y != end_point.y)
        {
            out_pts[out_num++] = end_point;
        }
    }

    return out_num;
}

static void overwrite_boundary_rows_with_points(maze_point_t *pts,
                                                int *count,
                                                const maze_point_t *overlay_pts,
                                                int overlay_num)
{
    if (pts == nullptr || count == nullptr || overlay_pts == nullptr || overlay_num <= 0)
    {
        return;
    }

    std::array<int, kProcHeight> row_x{};
    row_x.fill(-1);
    const int safe_count = std::clamp(*count, 0, VISION_BOUNDARY_NUM);
    for (int i = 0; i < safe_count; ++i)
    {
        const int y = std::clamp(pts[i].y, 0, kProcHeight - 1);
        row_x[y] = std::clamp(pts[i].x, 0, kProcWidth - 1);
    }
    for (int i = 0; i < overlay_num; ++i)
    {
        const int y = std::clamp(overlay_pts[i].y, 0, kProcHeight - 1);
        row_x[y] = std::clamp(overlay_pts[i].x, 0, kProcWidth - 1);
    }

    int out_num = 0;
    for (int y = kProcHeight - 2; y >= 1 && out_num < VISION_BOUNDARY_NUM; --y)
    {
        if (row_x[y] < 0)
        {
            continue;
        }
        pts[out_num++] = {row_x[y], y};
    }
    *count = out_num;
}

static bool apply_circle_guide_line_to_side(const maze_point_t *source_boundary_pts,
                                            int source_boundary_num,
                                            maze_point_t *dst_pts,
                                            int *dst_num,
                                            maze_point_t *dst_ipm_pts,
                                            int *dst_ipm_num,
                                            const maze_point_t &target,
                                            bool source_is_left,
                                            int anchor_extra_offset,
                                            bool replace_dst_boundary,
                                            uint16 *cache_x,
                                            uint16 *cache_y,
                                            uint16 *cache_num)
{
    if (cache_num != nullptr)
    {
        *cache_num = 0;
    }
    if (source_boundary_pts == nullptr || source_boundary_num <= 0 ||
        dst_pts == nullptr || dst_num == nullptr || dst_ipm_pts == nullptr || dst_ipm_num == nullptr)
    {
        return false;
    }

    const int safe_source_num = std::clamp(source_boundary_num, 0, VISION_BOUNDARY_NUM);
    const side_frame_wall_segment_t seg = find_first_side_frame_wall_segment(source_boundary_pts,
                                                                             safe_source_num,
                                                                             source_is_left,
                                                                             g_vision_runtime_config.circle_guide_min_frame_wall_segment_len);
    if (!seg.found)
    {
        return false;
    }

    const int anchor_index = std::clamp(seg.end_index + std::max(0, anchor_extra_offset), 0, safe_source_num - 1);
    const maze_point_t anchor = source_boundary_pts[anchor_index];
    std::array<maze_point_t, VISION_BOUNDARY_NUM> guide_pts{};
    const int guide_num = build_line_points_between(anchor, target, guide_pts.data(), static_cast<int>(guide_pts.size()));
    if (guide_num <= 0)
    {
        return false;
    }

    if (replace_dst_boundary)
    {
        copy_boundary_points(guide_pts.data(), guide_num, dst_pts, VISION_BOUNDARY_NUM);
        *dst_num = std::clamp(guide_num, 0, VISION_BOUNDARY_NUM);
        copy_boundary_points(guide_pts.data(), guide_num, dst_ipm_pts, VISION_BOUNDARY_NUM);
        *dst_ipm_num = std::clamp(guide_num, 0, VISION_BOUNDARY_NUM);
    }
    else
    {
        overwrite_boundary_rows_with_points(dst_pts, dst_num, guide_pts.data(), guide_num);
        overwrite_boundary_rows_with_points(dst_ipm_pts, dst_ipm_num, guide_pts.data(), guide_num);
    }
    int cached_count = 0;
    fill_single_line_arrays_from_points(guide_pts.data(),
                                        guide_num,
                                        cache_x,
                                        cache_y,
                                        &cached_count,
                                        kProcWidth,
                                        kProcHeight);
    if (cache_num != nullptr)
    {
        *cache_num = static_cast<uint16>(std::clamp(cached_count, 0, VISION_BOUNDARY_NUM));
    }
    return true;
}

static bool detect_src_straight_boundary_from_dirs(const uint8 *dirs, int count)
{
    if (dirs == nullptr || count <= 0)
    {
        return false;
    }

    const int cfg_check_count = std::max(1, g_vision_runtime_config.src_boundary_straight_check_count);
    const float cfg_ratio_min = std::clamp(g_vision_runtime_config.src_boundary_straight_dir45_ratio_min, 0.0f, 1.0f);
    const int sample_count = std::min(count, cfg_check_count);
    if (sample_count <= 0)
    {
        return false;
    }

    int dir45_count = 0;
    for (int i = 0; i < sample_count; ++i)
    {
        if (cross_lower_pre_dir(dirs[i]))
        {
            ++dir45_count;
        }
    }

    const float ratio = static_cast<float>(dir45_count) / static_cast<float>(sample_count);
    return ratio >= cfg_ratio_min;
}

static int pick_cross_lower_corner_index_near_transition(const uint8 *dirs,
                                                         int transition_start,
                                                         int transition_end,
                                                         int post_start,
                                                         int post_end)
{
    // 角点偏向平台跳变开始处：优先取过渡区中的第一个点；
    // 若无过渡区，再回退到后平台中的 2/3，最后回退到 1。
    if (transition_start < transition_end)
    {
        return transition_start;
    }
    for (int i = post_start; i < post_end; ++i)
    {
        if (dirs[i] == 2 || dirs[i] == 3)
        {
            return i;
        }
    }
    for (int i = post_start; i < post_end; ++i)
    {
        if (dirs[i] == 1)
        {
            return i;
        }
    }
    return post_start;
}

static cross_lower_corner_detection_t detect_cross_lower_corner_from_dirs(const maze_point_t *pts,
                                                                          const uint8 *dirs,
                                                                          int count)
{
    cross_lower_corner_detection_t result{};
    if (!g_cross_lower_corner_dir_enabled.load() || pts == nullptr || dirs == nullptr || count <= 0)
    {
        return result;
    }

    // 软模板：
    //   1) 前平台窗口为相邻平台：4/5 或 5/6；
    //   2) 中间过渡区长度受限，允许短暂扫过 1..6 任意层级；
    //   3) 后平台窗口为相邻平台：1/2 或 2/3；
    //   4) 前平台均值 - 后平台均值必须大于 2；
    //   5) 命中后角点优先落在过渡区起点；若无过渡区，则回退到后平台中的 2/3/1。
    const int pre_win = std::clamp(g_cross_lower_corner_pre_window.load(), 1, VISION_BOUNDARY_NUM);
    const int post_win = std::clamp(g_cross_lower_corner_post_window.load(), 1, VISION_BOUNDARY_NUM);
    const int need_pre = std::clamp(g_cross_lower_corner_pre_min_votes.load(), 1, pre_win);
    const int need_post = std::clamp(g_cross_lower_corner_post_min_votes.load(), 1, post_win);
    const int transition_max = std::clamp(g_cross_lower_corner_transition_max_len.load(), 0, VISION_BOUNDARY_NUM);

    for (int i = pre_win; i + post_win < count; ++i)
    {
        const int pre_start = i - pre_win;
        if (!cross_lower_match_adjacent_platform(dirs,
                                                 pre_start,
                                                 i,
                                                 4,
                                                 5,
                                                 5,
                                                 6,
                                                 need_pre))
        {
            continue;
        }

        for (int transition_len = 0; transition_len <= transition_max && i + transition_len + post_win <= count; ++transition_len)
        {
            bool transition_ok = true;
            for (int k = i; k < i + transition_len; ++k)
            {
                if (!cross_lower_transition_dir(dirs[k]))
                {
                    transition_ok = false;
                    break;
                }
            }
            if (!transition_ok)
            {
                continue;
            }

            int post_votes = 0;
            const int post_start = i + transition_len;
            const int post_end = post_start + post_win;
            for (int k = post_start; k < post_start + post_win; ++k)
            {
                if (cross_lower_post_dir(dirs[k]))
                {
                    ++post_votes;
                }
            }
            if (post_votes < need_post ||
                !cross_lower_match_adjacent_platform(dirs,
                                                     post_start,
                                                     post_end,
                                                     1,
                                                     2,
                                                     2,
                                                     3,
                                                     need_post))
            {
                continue;
            }

            float pre_mean = 0.0f;
            float post_mean = 0.0f;
            if (!cross_lower_compute_adjacent_platform_mean(dirs, pre_start, i, 4, 5, 5, 6, &pre_mean) ||
                !cross_lower_compute_adjacent_platform_mean(dirs, post_start, post_end, 1, 2, 2, 3, &post_mean) ||
                (pre_mean - post_mean) <= 2.0f)
            {
                continue;
            }

            const int corner_idx = pick_cross_lower_corner_index_near_transition(dirs,
                                                                                 i,
                                                                                 post_start,
                                                                                 post_start,
                                                                                 post_end);
            result.found = true;
            result.index = corner_idx;
            result.point = pts[corner_idx];
            return result;
        }
    }

    return result;
}

static void update_cross_lower_corner_detection_cache(const maze_point_t *left_pts,
                                                      const uint8 *left_dirs,
                                                      int left_count,
                                                      const maze_point_t *right_pts,
                                                      const uint8 *right_dirs,
                                                      int right_count)
{
    clear_cross_lower_corner_detection_cache();

    const cross_lower_corner_detection_t left =
        detect_cross_lower_corner_from_dirs(left_pts, left_dirs, std::clamp(left_count, 0, VISION_BOUNDARY_NUM));
    const cross_lower_corner_detection_t right =
        detect_cross_lower_corner_from_dirs(right_pts, right_dirs, std::clamp(right_count, 0, VISION_BOUNDARY_NUM));
    const int y_diff_max = std::max(0, g_cross_lower_corner_pair_y_diff_max.load());
    const bool pair_valid = left.found && right.found && std::abs(left.point.y - right.point.y) <= y_diff_max;

    g_cross_lower_left_corner_found.store(left.found);
    g_cross_lower_right_corner_found.store(right.found);
    g_cross_lower_corner_pair_valid.store(pair_valid);
    g_cross_lower_left_corner_index.store(left.index);
    g_cross_lower_right_corner_index.store(right.index);
    g_cross_lower_left_corner_x.store(left.point.x);
    g_cross_lower_left_corner_y.store(left.point.y);
    g_cross_lower_right_corner_x.store(right.point.x);
    g_cross_lower_right_corner_y.store(right.point.y);
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

    int first_idx = std::clamp(corner_indices[0] - 1, 0, n - 1);
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

static bool detect_straight_boundary_from_ipm_features(int boundary_count,
                                                       const float *angle_cos,
                                                       int angle_cos_count,
                                                       bool has_corner,
                                                       int min_points,
                                                       int check_count,
                                                       float min_cos)
{
    if (has_corner)
    {
        return false;
    }

    const int required_points = std::max(1, min_points);
    const int required_check_count = std::max(1, check_count);
    if (boundary_count <= required_points || angle_cos == nullptr || angle_cos_count < required_check_count)
    {
        return false;
    }

    const float cos_threshold = std::clamp(min_cos, -1.0f, 1.0f);
    for (int i = 0; i < required_check_count; ++i)
    {
        const float value = angle_cos[i];
        if (!std::isfinite(value) || value < cos_threshold)
        {
            return false;
        }
    }
    return true;
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

static void prepend_ipm_bottom_midpoint_bridge_inplace(maze_point_t *pts,
                                                       int *num,
                                                       int width,
                                                       int height,
                                                       float step_px)
{
    if (pts == nullptr || num == nullptr || *num <= 0 || width <= 0 || height <= 0)
    {
        return;
    }

    const int in_num = std::clamp(*num, 0, VISION_BOUNDARY_NUM);
    if (in_num <= 0)
    {
        *num = 0;
        return;
    }

    const int anchor_x = width / 2;
    const int anchor_y = height - 1;
    const int first_x = std::clamp(pts[0].x, 0, width - 1);
    const int first_y = std::clamp(pts[0].y, 0, height - 1);

    std::array<maze_point_t, VISION_BOUNDARY_NUM> out{};
    int out_num = 0;
    out[out_num++] = {anchor_x, anchor_y};

    const float dx = static_cast<float>(first_x - anchor_x);
    const float dy = static_cast<float>(first_y - anchor_y);
    const float seg_len = std::sqrt(dx * dx + dy * dy);
    const float step = std::max(0.5f, step_px);
    if (seg_len > 1e-6f)
    {
        const int bridge_points = std::max(0, static_cast<int>(std::floor(seg_len / step)));
        for (int i = 1; i <= bridge_points && out_num < VISION_BOUNDARY_NUM; ++i)
        {
            const float travel = std::min(seg_len, step * static_cast<float>(i));
            const float t = travel / seg_len;
            const int px = std::clamp(static_cast<int>(std::lround(static_cast<float>(anchor_x) + dx * t)), 0, width - 1);
            const int py = std::clamp(static_cast<int>(std::lround(static_cast<float>(anchor_y) + dy * t)), 0, height - 1);
            if (px == out[out_num - 1].x && py == out[out_num - 1].y)
            {
                continue;
            }
            out[out_num++] = {px, py};
        }
    }

    for (int i = 0; i < in_num && out_num < VISION_BOUNDARY_NUM; ++i)
    {
        const int px = std::clamp(pts[i].x, 0, width - 1);
        const int py = std::clamp(pts[i].y, 0, height - 1);
        if (px == out[out_num - 1].x && py == out[out_num - 1].y)
        {
            continue;
        }
        out[out_num++] = {px, py};
    }

    for (int i = 0; i < out_num; ++i)
    {
        pts[i] = out[i];
    }
    *num = out_num;
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

    // IPM 角点支路当前停用：
    // 1) 不再拷贝 left_angle/right_angle；
    // 2) 不再做 triangle_filter_boundary_points_inplace；
    // 3) 不再计算三点夹角 cos；
    // 4) 不再做角点检测/NMS；
    // 5) 不再按首个角点截断边界；
    // 6) 不再基于该支路回填 IPM/source corner 缓存；
    // 7) 不再做 IPM 直边判定。
    //
    // 保留下面这整段原始代码作为恢复参考，后续若要恢复可直接取消注释。
#if 0
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

    if (g_ipm_boundary_truncate_at_first_corner_enabled.load())
    {
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
    }

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

    g_ipm_left_boundary_corner_found.store(g_ipm_left_boundary_corner_point_count > 0);
    g_ipm_right_boundary_corner_found.store(g_ipm_right_boundary_corner_point_count > 0);
    g_src_left_boundary_corner_found.store(g_src_left_boundary_corner_point_count > 0);
    g_src_right_boundary_corner_found.store(g_src_right_boundary_corner_point_count > 0);
    g_ipm_left_boundary_corner_first_x.store(g_ipm_left_boundary_corner_point_count > 0 ? static_cast<int>(g_ipm_left_boundary_corner_x[0]) : 0);
    g_ipm_left_boundary_corner_first_y.store(g_ipm_left_boundary_corner_point_count > 0 ? static_cast<int>(g_ipm_left_boundary_corner_y[0]) : 0);
    g_ipm_right_boundary_corner_first_x.store(g_ipm_right_boundary_corner_point_count > 0 ? static_cast<int>(g_ipm_right_boundary_corner_x[0]) : 0);
    g_ipm_right_boundary_corner_first_y.store(g_ipm_right_boundary_corner_point_count > 0 ? static_cast<int>(g_ipm_right_boundary_corner_y[0]) : 0);
    g_ipm_left_boundary_corner_first_index.store(g_ipm_left_boundary_corner_count > 0 ? g_ipm_left_boundary_corner_indices[0] : -1);
    g_ipm_right_boundary_corner_first_index.store(g_ipm_right_boundary_corner_count > 0 ? g_ipm_right_boundary_corner_indices[0] : -1);
    g_src_left_boundary_corner_first_x.store(g_src_left_boundary_corner_point_count > 0 ? static_cast<int>(g_src_left_boundary_corner_x[0]) : 0);
    g_src_left_boundary_corner_first_y.store(g_src_left_boundary_corner_point_count > 0 ? static_cast<int>(g_src_left_boundary_corner_y[0]) : 0);
    g_src_right_boundary_corner_first_x.store(g_src_right_boundary_corner_point_count > 0 ? static_cast<int>(g_src_right_boundary_corner_x[0]) : 0);
    g_src_right_boundary_corner_first_y.store(g_src_right_boundary_corner_point_count > 0 ? static_cast<int>(g_src_right_boundary_corner_y[0]) : 0);
    g_ipm_left_boundary_straight_detected.store(
        detect_straight_boundary_from_ipm_features(left_proc_num,
                                                   g_ipm_left_boundary_angle_cos,
                                                   g_ipm_left_boundary_angle_cos_count,
                                                   g_ipm_left_boundary_corner_count > 0,
                                                   g_ipm_boundary_straight_min_points.load(),
                                                   g_ipm_boundary_straight_check_count.load(),
                                                   g_ipm_boundary_straight_min_cos.load()));
    g_ipm_right_boundary_straight_detected.store(
        detect_straight_boundary_from_ipm_features(right_proc_num,
                                                   g_ipm_right_boundary_angle_cos,
                                                   g_ipm_right_boundary_angle_cos_count,
                                                   g_ipm_right_boundary_corner_count > 0,
                                                   g_ipm_boundary_straight_min_points.load(),
                                                   g_ipm_boundary_straight_check_count.load(),
                                                   g_ipm_boundary_straight_min_cos.load()));
#endif

    const vision_route_state_snapshot_t route_snapshot = vision_route_state_machine_snapshot();

    std::array<maze_point_t, VISION_BOUNDARY_NUM> cross_left_proc = left_proc;
    std::array<maze_point_t, VISION_BOUNDARY_NUM> cross_right_proc = right_proc;
    int cross_left_proc_num = left_proc_num;
    int cross_right_proc_num = right_proc_num;

    // 处理链边界法向平移中线：
    // 先看偏好源，再按左右边界点数择优，只计算并保留一条平移中线。
    const bool selected_is_right = select_shift_source_from_preference_and_counts(route_snapshot.preferred_source,
                                                                                   cross_left_proc_num,
                                                                                   cross_right_proc_num);
    const float track_width_px = std::max(0.0f, g_ipm_track_width_px.load());
    float target_offset_from_left_px = std::clamp(g_ipm_center_target_offset_from_left_px.load(),
                                                  0.0f,
                                                  track_width_px);
    const bool left_circle_offset_active =
        (route_snapshot.main_state == VISION_ROUTE_MAIN_CIRCLE_LEFT) &&
        (route_snapshot.sub_state == VISION_ROUTE_SUB_CIRCLE_LEFT_4 ||
         route_snapshot.sub_state == VISION_ROUTE_SUB_CIRCLE_LEFT_5);
    const bool right_circle_offset_active =
        (route_snapshot.main_state == VISION_ROUTE_MAIN_CIRCLE_RIGHT) &&
        (route_snapshot.sub_state == VISION_ROUTE_SUB_CIRCLE_RIGHT_4 ||
         route_snapshot.sub_state == VISION_ROUTE_SUB_CIRCLE_RIGHT_5);
    if (left_circle_offset_active)
    {
        target_offset_from_left_px = std::clamp(g_vision_runtime_config.route_circle_center_target_offset_from_left_px,
                                                0.0f,
                                                track_width_px);
    }
    else if (right_circle_offset_active)
    {
        const float left_circle_offset = std::clamp(g_vision_runtime_config.route_circle_center_target_offset_from_left_px,
                                                    0.0f,
                                                    track_width_px);
        target_offset_from_left_px = std::max(0.0f, track_width_px - left_circle_offset);
    }
    const float shift_dist_from_left_px = target_offset_from_left_px;
    const float shift_dist_from_right_px = std::max(0.0f, track_width_px - target_offset_from_left_px);
    std::array<maze_point_t, VISION_BOUNDARY_NUM> center_selected{};
    int center_selected_num = 0;
    if (selected_is_right)
    {
        shift_boundary_along_normal(cross_right_proc.data(),
                                    cross_right_proc_num,
                                    cross_left_proc.data(),
                                    cross_left_proc_num,
                                    center_selected.data(),
                                    &center_selected_num,
                                    kIpmOutputWidth,
                                    kIpmOutputHeight,
                                    shift_dist_from_right_px,
                                    false);
    }
    else
    {
        shift_boundary_along_normal(cross_left_proc.data(),
                                    cross_left_proc_num,
                                    cross_right_proc.data(),
                                    cross_right_proc_num,
                                    center_selected.data(),
                                    &center_selected_num,
                                    kIpmOutputWidth,
                                    kIpmOutputHeight,
                                    shift_dist_from_left_px,
                                    true);
    }

    // 平移中线生成后，先补一段“IPM 底部中点 -> 当前中线起点”的连接线，再统一做中线后处理。
    if (route_snapshot.main_state != VISION_ROUTE_MAIN_CROSS)
    {
        prepend_ipm_bottom_midpoint_bridge_inplace(center_selected.data(),
                                                   &center_selected_num,
                                                   kIpmOutputWidth,
                                                   kIpmOutputHeight,
                                                   g_ipm_centerline_resample_step_px.load());
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

static inline bool pixel_is_path(const uint8 *img, int x, int y, uint8 white_threshold, bool wall_is_white)
{
    return !pixel_is_wall(img, x, y, white_threshold, wall_is_white);
}

static bool binary_point_in_trace_range(int x, int y, int y_min, int x_min, int x_max)
{
    return x > 0 && x < (kProcWidth - 1) &&
           y > 0 && y < (kProcHeight - 1) &&
           x >= x_min && x <= x_max &&
           y >= y_min;
}

static bool point_touches_artificial_frame(int x, int y)
{
    return x <= 1 || x >= (kProcWidth - 2) || y <= 1;
}

static int trace_boundary_eight_neighbor(const uint8 *classify_img,
                                         uint8 white_threshold,
                                         bool wall_is_white,
                                         int start_x,
                                         int start_y,
                                         int y_min,
                                         int x_min,
                                         int x_max,
                                         bool prefer_left_bias,
                                         maze_point_t *pts,
                                         uint8 *dirs,
                                         int max_pts,
                                         int *first_frame_touch_index)
{
    if (classify_img == nullptr || pts == nullptr || dirs == nullptr || max_pts <= 0 || start_x < x_min || start_x > x_max)
    {
        if (first_frame_touch_index) *first_frame_touch_index = -1;
        return 0;
    }

    std::fill_n(dirs, max_pts, static_cast<uint8>(255));
    const int (*seeds)[2] = prefer_left_bias ? kEightNeighborLeftSeeds : kEightNeighborRightSeeds;
    int x = start_x;
    int y = start_y;
    const int y_fallback_stop_delta = std::max(0, g_maze_trace_y_fallback_stop_delta.load());
    int min_y_seen = start_y;
    int first_touch = -1;
    int step = 0;

    while (step < max_pts &&
           binary_point_in_trace_range(x, y, y_min, x_min, x_max))
    {
        pts[step] = {x, y};
        if (first_touch < 0 && point_touches_artificial_frame(x, y))
        {
            first_touch = step;
        }
        ++step;

        maze_point_t candidates[8]{};
        int candidate_dirs[8]{};
        int candidate_count = 0;
        for (int i = 0; i < 8; ++i)
        {
            const int nx0 = x + seeds[i][0];
            const int ny0 = y + seeds[i][1];
            const int next = (i + 1) & 7;
            const int nx1 = x + seeds[next][0];
            const int ny1 = y + seeds[next][1];
            if (!binary_point_in_trace_range(nx0, ny0, y_min, x_min, x_max) ||
                !binary_point_in_trace_range(nx1, ny1, y_min, x_min, x_max))
            {
                continue;
            }
            if (pixel_is_wall(classify_img, nx0, ny0, white_threshold, wall_is_white) &&
                pixel_is_path(classify_img, nx1, ny1, white_threshold, wall_is_white))
            {
                candidates[candidate_count] = {nx0, ny0};
                candidate_dirs[candidate_count] = i;
                ++candidate_count;
            }
        }

        if (candidate_count <= 0)
        {
            break;
        }

        int best = 0;
        for (int i = 1; i < candidate_count; ++i)
        {
            if (candidates[i].y < candidates[best].y ||
                (candidates[i].y == candidates[best].y && candidate_dirs[i] < candidate_dirs[best]))
            {
                best = i;
            }
        }

        if (step >= 3 &&
            pts[step - 1].x == pts[step - 2].x &&
            pts[step - 1].x == pts[step - 3].x &&
            pts[step - 1].y == pts[step - 2].y &&
            pts[step - 1].y == pts[step - 3].y)
        {
            break;
        }

        x = candidates[best].x;
        y = candidates[best].y;
        dirs[step - 1] = static_cast<uint8>(candidate_dirs[best]);
        min_y_seen = std::min(min_y_seen, y);
        if (y > min_y_seen + y_fallback_stop_delta)
        {
            break;
        }
    }

    if (first_frame_touch_index) *first_frame_touch_index = first_touch;
    return step;
}

static int trace_left_boundary_selected_method(const uint8 *classify_img,
                                               uint8 white_threshold,
                                               bool wall_is_white,
                                               int start_x,
                                               int start_y,
                                               int y_min,
                                               int x_min,
                                               int x_max,
                                               maze_point_t *pts,
                                               uint8 *dirs,
                                               int max_pts,
                                               int *first_frame_touch_index)
{
    const vision_trace_method_enum method =
        static_cast<vision_trace_method_enum>(g_maze_trace_method.load());
    if (method == VISION_TRACE_METHOD_EIGHT_NEIGHBOR)
    {
        return trace_boundary_eight_neighbor(classify_img,
                                             white_threshold,
                                             wall_is_white,
                                             start_x,
                                             start_y,
                                             y_min,
                                             x_min,
                                             x_max,
                                             true,
                                             pts,
                                             dirs,
                                             max_pts,
                                             first_frame_touch_index);
    }

    if (first_frame_touch_index) *first_frame_touch_index = -1;
    return maze_trace_left_hand(classify_img,
                                white_threshold,
                                wall_is_white,
                                start_x,
                                start_y,
                                y_min,
                                x_min,
                                x_max,
                                pts,
                                max_pts);
}

static bool build_cross_aux_boundary(const uint8 *classify_img,
                                     uint8 white_threshold,
                                     bool wall_is_white,
                                     bool is_left,
                                     int corner_x,
                                     int corner_y,
                                     int x_min,
                                     int x_max,
                                     maze_point_t *raw_pts,
                                     uint8 *raw_dirs,
                                     int *raw_count,
                                     maze_point_t *regular_pts,
                                     int *regular_count,
                                     maze_point_t *transition_point)
{
    if (raw_count) *raw_count = 0;
    if (regular_count) *regular_count = 0;
    if (transition_point) *transition_point = maze_point_t{0, 0};
    if (classify_img == nullptr || raw_pts == nullptr || raw_dirs == nullptr || regular_pts == nullptr)
    {
        return false;
    }

    maze_point_t found_transition{};
    if (!find_vertical_white_to_black_transition(classify_img,
                                                 white_threshold,
                                                 corner_x,
                                                 corner_y,
                                                 g_vision_runtime_config.cross_aux_vertical_scan_max_rows,
                                                 &found_transition.x,
                                                 &found_transition.y))
    {
        return false;
    }

    const int y_min = std::max(1,
                               found_transition.y -
                               std::max(1, g_vision_runtime_config.cross_aux_trace_upward_rows_max));
    int first_frame_touch_index = -1;
    const int max_trace_count =
        std::clamp(g_vision_runtime_config.cross_aux_trace_max_points, 1, VISION_BOUNDARY_NUM);
    const int traced_count = trace_boundary_eight_neighbor(classify_img,
                                                           white_threshold,
                                                           wall_is_white,
                                                           found_transition.x,
                                                           found_transition.y,
                                                           y_min,
                                                           x_min,
                                                           x_max,
                                                           is_left,
                                                           raw_pts,
                                                           raw_dirs,
                                                           max_trace_count,
                                                           &first_frame_touch_index);
    if (traced_count <= 0)
    {
        return false;
    }

    const int extracted_regular_count = extract_one_point_per_row_from_contour(raw_pts,
                                                                               traced_count,
                                                                               is_left,
                                                                               false,
                                                                               regular_pts,
                                                                               VISION_BOUNDARY_NUM);
    if (extracted_regular_count <= 0)
    {
        return false;
    }

    if (raw_count) *raw_count = traced_count;
    if (regular_count) *regular_count = extracted_regular_count;
    if (transition_point) *transition_point = found_transition;
    return true;
}

static int find_cross_upper_record_index_from_dirs(const uint8 *dirs, int count)
{
    if (dirs == nullptr || count <= 0)
    {
        return -1;
    }

    const int pre_len = std::max(1, g_vision_runtime_config.cross_upper_dir4_pre_run_len);
    const int transition_max_len = std::max(0, g_vision_runtime_config.cross_upper_transition_max_len);
    const int post_len = std::max(1, g_vision_runtime_config.cross_upper_dir6_post_run_len);
    for (int pre_start = 0; pre_start < count; ++pre_start)
    {
        const int pre_end = pre_start + pre_len;
        if (pre_end >= count)
        {
            break;
        }

        bool pre_ok = true;
        for (int i = pre_start; i < pre_end; ++i)
        {
            if (dirs[i] != 4)
            {
                pre_ok = false;
                break;
            }
        }
        if (!pre_ok)
        {
            continue;
        }

        for (int transition_len = 0; transition_len <= transition_max_len; ++transition_len)
        {
            const int post_start = pre_end + transition_len;
            const int post_end = post_start + post_len;
            if (post_end > count)
            {
                break;
            }

            bool transition_ok = true;
            for (int i = pre_end; i < post_start; ++i)
            {
                if (!(dirs[i] == 4 || dirs[i] == 5 || dirs[i] == 6))
                {
                    transition_ok = false;
                    break;
                }
            }
            if (!transition_ok)
            {
                continue;
            }

            bool post_ok = true;
            for (int i = post_start; i < post_end; ++i)
            {
                if (dirs[i] != 6)
                {
                    post_ok = false;
                    break;
                }
            }
            if (post_ok)
            {
                return post_end - 1;
            }
        }
    }

    return -1;
}

static bool find_cross_upper_corner_from_trace(const maze_point_t *trace_pts,
                                               const uint8 *trace_dirs,
                                               int trace_count,
                                               bool is_left,
                                               maze_point_t *corner_point,
                                               int *corner_index)
{
    if (corner_point) *corner_point = maze_point_t{0, 0};
    if (corner_index) *corner_index = -1;
    if (trace_pts == nullptr || trace_dirs == nullptr || trace_count <= 0)
    {
        return false;
    }

    const int record_index = find_cross_upper_record_index_from_dirs(trace_dirs, trace_count);
    if (record_index < 0 || record_index >= trace_count)
    {
        return false;
    }

    for (int i = record_index + 1; i < trace_count; ++i)
    {
        if (trace_dirs[i] == 5)
        {
            if (corner_point) *corner_point = trace_pts[i];
            if (corner_index) *corner_index = i;
            return true;
        }
    }

    return false;
}

static int find_leading_side_frame_touch_prefix(const maze_point_t *pts, int count, bool is_left)
{
    if (pts == nullptr || count <= 0)
    {
        return 0;
    }

    const int safe_count = std::clamp(count, 0, VISION_BOUNDARY_NUM);
    int prefix = 0;
    while (prefix < safe_count)
    {
        const bool on_side_frame = is_left ? (pts[prefix].x <= 1) : (pts[prefix].x >= (kProcWidth - 2));
        if (!on_side_frame)
        {
            break;
        }
        ++prefix;
    }
    return prefix;
}

static bool find_cross_upper_corner_from_trace_after_frame_prefix(const maze_point_t *trace_pts,
                                                                  const uint8 *trace_dirs,
                                                                  int trace_count,
                                                                  bool is_left,
                                                                  maze_point_t *corner_point,
                                                                  int *corner_index)
{
    if (corner_point) *corner_point = maze_point_t{0, 0};
    if (corner_index) *corner_index = -1;
    if (trace_pts == nullptr || trace_dirs == nullptr || trace_count <= 0)
    {
        return false;
    }

    const int prefix = find_leading_side_frame_touch_prefix(trace_pts, trace_count, is_left);
    if (prefix >= trace_count)
    {
        return false;
    }

    int local_index = -1;
    maze_point_t local_corner{};
    if (!find_cross_upper_corner_from_trace(trace_pts + prefix,
                                            trace_dirs + prefix,
                                            trace_count - prefix,
                                            is_left,
                                            &local_corner,
                                            &local_index))
    {
        return false;
    }

    if (corner_point) *corner_point = local_corner;
    if (corner_index) *corner_index = prefix + local_index;
    return true;
}

static bool find_nth_dir_point_on_trace(const maze_point_t *trace_pts,
                                        const uint8 *trace_dirs,
                                        int trace_count,
                                        uint8 target_dir,
                                        int nth_match,
                                        maze_point_t *corner_point,
                                        int *corner_index)
{
    if (corner_point) *corner_point = maze_point_t{0, 0};
    if (corner_index) *corner_index = -1;
    if (trace_pts == nullptr || trace_dirs == nullptr || trace_count <= 0 || nth_match <= 0)
    {
        return false;
    }

    int hit_count = 0;
    for (int i = 0; i < trace_count; ++i)
    {
        if (trace_dirs[i] != target_dir)
        {
            continue;
        }
        hit_count += 1;
        if (hit_count == nth_match)
        {
            if (corner_point) *corner_point = trace_pts[i];
            if (corner_index) *corner_index = i;
            return true;
        }
    }

    return false;
}

static int truncate_regular_boundary_before_point_inplace(maze_point_t *pts,
                                                          int count,
                                                          const maze_point_t &corner_point)
{
    if (pts == nullptr || count <= 0)
    {
        return 0;
    }

    int start_index = -1;
    for (int i = 0; i < count; ++i)
    {
        if (pts[i].y <= corner_point.y)
        {
            start_index = i;
            break;
        }
    }
    if (start_index <= 0)
    {
        return count;
    }

    const int remain = count - start_index;
    if (remain <= 0)
    {
        return 0;
    }
    for (int i = 0; i < remain; ++i)
    {
        pts[i] = pts[start_index + i];
    }
    return remain;
}

static int concatenate_boundary_segments(const maze_point_t *segment_a,
                                         int count_a,
                                         const maze_point_t *segment_b,
                                         int count_b,
                                         const maze_point_t *segment_c,
                                         int count_c,
                                         maze_point_t *out_pts,
                                         int max_out_pts)
{
    if (out_pts == nullptr || max_out_pts <= 0)
    {
        return 0;
    }

    int out_num = 0;
    auto append_segment = [&](const maze_point_t *pts, int count) {
        if (pts == nullptr || count <= 0)
        {
            return;
        }
        const int safe_count = std::clamp(count, 0, VISION_BOUNDARY_NUM);
        for (int i = 0; i < safe_count && out_num < max_out_pts; ++i)
        {
            const maze_point_t point = pts[i];
            if (out_num > 0 &&
                out_pts[out_num - 1].x == point.x &&
                out_pts[out_num - 1].y == point.y)
            {
                continue;
            }
            out_pts[out_num++] = point;
        }
    };

    append_segment(segment_a, count_a);
    append_segment(segment_b, count_b);
    append_segment(segment_c, count_c);
    return out_num;
}

static int trace_right_boundary_selected_method(const uint8 *classify_img,
                                                uint8 white_threshold,
                                                bool wall_is_white,
                                                int start_x,
                                                int start_y,
                                                int y_min,
                                                int x_min,
                                                int x_max,
                                                maze_point_t *pts,
                                                uint8 *dirs,
                                                int max_pts,
                                                int *first_frame_touch_index)
{
    const vision_trace_method_enum method =
        static_cast<vision_trace_method_enum>(g_maze_trace_method.load());
    if (method == VISION_TRACE_METHOD_EIGHT_NEIGHBOR)
    {
        return trace_boundary_eight_neighbor(classify_img,
                                             white_threshold,
                                             wall_is_white,
                                             start_x,
                                             start_y,
                                             y_min,
                                             x_min,
                                             x_max,
                                             false,
                                             pts,
                                             dirs,
                                             max_pts,
                                             first_frame_touch_index);
    }

    if (first_frame_touch_index) *first_frame_touch_index = -1;
    return maze_trace_right_hand(classify_img,
                                 white_threshold,
                                 wall_is_white,
                                 start_x,
                                 start_y,
                                 y_min,
                                 x_min,
                                 x_max,
                                 pts,
                                 max_pts);
}

static int extract_one_point_per_row_from_contour(const maze_point_t *raw_pts,
                                                  int raw_num,
                                                  bool is_left,
                                                  bool skip_artificial_frame,
                                                  maze_point_t *out_pts,
                                                  int max_out_pts)
{
    if (raw_pts == nullptr || out_pts == nullptr || raw_num <= 0 || max_out_pts <= 0)
    {
        return 0;
    }

    std::array<int, kProcHeight> border_x{};
    border_x.fill(-1);
    int initial_frame_end = 0;
    bool keep_initial_frame_wall = false;
    if (skip_artificial_frame && point_touches_artificial_frame(raw_pts[0].x, raw_pts[0].y))
    {
        while (initial_frame_end < raw_num &&
               point_touches_artificial_frame(raw_pts[initial_frame_end].x, raw_pts[initial_frame_end].y))
        {
            ++initial_frame_end;
        }
        if (initial_frame_end > 0 && initial_frame_end < raw_num)
        {
            const int initial_frame_y_span = std::abs(raw_pts[0].y - raw_pts[initial_frame_end - 1].y);
            keep_initial_frame_wall = initial_frame_y_span < kInitialFrameWallKeepMaxYSpan;
        }
    }

    for (int i = 0; i < raw_num; ++i)
    {
        const bool is_initial_frame_wall = keep_initial_frame_wall && i < initial_frame_end;
        if (skip_artificial_frame &&
            !is_initial_frame_wall &&
            point_touches_artificial_frame(raw_pts[i].x, raw_pts[i].y))
        {
            continue;
        }

        const int y = std::clamp(raw_pts[i].y, 0, kProcHeight - 1);
        if (y <= 0 || y >= kProcHeight - 1 || border_x[y] >= 0)
        {
            continue;
        }

        const int adjusted_x = raw_pts[i].x + (is_left ? 1 : -1);
        border_x[y] = std::clamp(adjusted_x, 0, kProcWidth - 1);
    }

    int out_num = 0;
    for (int y = kProcHeight - 2; y >= 1 && out_num < max_out_pts; --y)
    {
        if (border_x[y] < 0)
        {
            continue;
        }
        out_pts[out_num++] = {border_x[y], y};
    }

    return out_num;
}

static bool initial_frame_wall_can_be_kept_for_ipm(const maze_point_t *raw_pts, int raw_num)
{
    if (raw_pts == nullptr || raw_num <= 1 || !point_touches_artificial_frame(raw_pts[0].x, raw_pts[0].y))
    {
        return false;
    }

    int initial_frame_end = 0;
    while (initial_frame_end < raw_num &&
           point_touches_artificial_frame(raw_pts[initial_frame_end].x, raw_pts[initial_frame_end].y))
    {
        ++initial_frame_end;
    }

    if (initial_frame_end <= 0 || initial_frame_end >= raw_num)
    {
        return false;
    }

    const int initial_frame_y_span = std::abs(raw_pts[0].y - raw_pts[initial_frame_end - 1].y);
    return initial_frame_y_span < kInitialFrameWallKeepMaxYSpan;
}

static int trace_num_for_ipm_artificial_frame_policy(const maze_point_t *raw_pts,
                                                     int raw_num,
                                                     int first_frame_touch_index)
{
    if (raw_num <= 0)
    {
        return 0;
    }
    if (first_frame_touch_index < 0)
    {
        return raw_num;
    }
    if (first_frame_touch_index == 0 && initial_frame_wall_can_be_kept_for_ipm(raw_pts, raw_num))
    {
        return raw_num;
    }
    return std::min(raw_num, first_frame_touch_index + 1);
}

static int find_first_artificial_frame_touch_index(const maze_point_t *pts, int num)
{
    if (pts == nullptr || num <= 0)
    {
        return -1;
    }

    for (int i = 0; i < num; ++i)
    {
        if (point_touches_artificial_frame(pts[i].x, pts[i].y))
        {
            return i;
        }
    }
    return -1;
}

static int trim_initial_artificial_frame_prefix_inplace(maze_point_t *pts,
                                                        uint8 *dirs,
                                                        int *num)
{
    if (pts == nullptr || num == nullptr || *num <= 0)
    {
        return 0;
    }

    const int in_num = std::clamp(*num, 0, VISION_BOUNDARY_NUM);
    if (in_num <= 0 || !point_touches_artificial_frame(pts[0].x, pts[0].y))
    {
        return 0;
    }

    int prefix_end = 0;
    while (prefix_end < in_num &&
           point_touches_artificial_frame(pts[prefix_end].x, pts[prefix_end].y))
    {
        ++prefix_end;
    }

    // 只有“起始贴着图像边界，随后重新回到真实边界”时，才删除这段前缀。
    if (prefix_end <= 0 || prefix_end >= in_num)
    {
        return 0;
    }

    const int initial_frame_y_span = std::abs(pts[0].y - pts[prefix_end - 1].y);
    if (initial_frame_y_span >= kInitialFrameWallKeepMaxYSpan)
    {
        if (dirs != nullptr)
        {
            std::fill_n(dirs, VISION_BOUNDARY_NUM, static_cast<uint8>(255));
        }
        *num = 0;
        return prefix_end;
    }

    const int remaining = in_num - prefix_end;
    for (int i = 0; i < remaining; ++i)
    {
        pts[i] = pts[i + prefix_end];
        if (dirs != nullptr)
        {
            dirs[i] = dirs[i + prefix_end];
        }
    }
    if (dirs != nullptr)
    {
        std::fill_n(dirs + remaining, VISION_BOUNDARY_NUM - remaining, static_cast<uint8>(255));
    }
    *num = remaining;
    return prefix_end;
}

static int rebuild_boundary_points_from_row_table(const std::array<int, kProcHeight> &border_x,
                                                  maze_point_t *pts,
                                                  int max_pts)
{
    if (pts == nullptr || max_pts <= 0)
    {
        return 0;
    }

    int out_num = 0;
    for (int y = kProcHeight - 2; y >= 1 && out_num < max_pts; --y)
    {
        if (border_x[y] < 0)
        {
            continue;
        }
        pts[out_num++] = {std::clamp(border_x[y], 0, kProcWidth - 1), y};
    }
    return out_num;
}

static bool fit_src_boundary_slope_by_y_below_corner(const maze_point_t *pts,
                                                     int num,
                                                     int corner_y,
                                                     float *slope)
{
    if (pts == nullptr || slope == nullptr || num <= 0)
    {
        return false;
    }

    double sum_y = 0.0;
    double sum_x = 0.0;
    double sum_yx = 0.0;
    double sum_y2 = 0.0;
    int fit_num = 0;
    for (int i = 0; i < num && fit_num < kCrossLowerFitPointCount; ++i)
    {
        // 每行数组按 y 从大到小排列；仅使用角点下方（更靠近车体）的边界点估计方向。
        if (pts[i].y <= corner_y)
        {
            continue;
        }

        const double y = static_cast<double>(pts[i].y);
        const double x = static_cast<double>(pts[i].x);
        sum_y += y;
        sum_x += x;
        sum_yx += y * x;
        sum_y2 += y * y;
        ++fit_num;
    }

    if (fit_num < kCrossLowerFitMinPointCount)
    {
        return false;
    }

    const double denom = static_cast<double>(fit_num) * sum_y2 - sum_y * sum_y;
    if (std::fabs(denom) < 1e-6)
    {
        return false;
    }

    const double k = (static_cast<double>(fit_num) * sum_yx - sum_y * sum_x) / denom;
    *slope = static_cast<float>(k);
    return true;
}

static int extrapolate_cross_lower_boundary_inplace(maze_point_t *pts,
                                                    int num,
                                                    int corner_x,
                                                    int corner_y,
                                                    int max_pts)
{
    if (pts == nullptr || num <= 0 || max_pts <= 0)
    {
        return num;
    }

    float slope = 0.0f;
    if (!fit_src_boundary_slope_by_y_below_corner(pts, num, corner_y, &slope))
    {
        return num;
    }

    const float intercept = static_cast<float>(corner_x) - slope * static_cast<float>(corner_y);

    num = truncate_boundary_at_cross_lower_corner_inplace(pts, num, corner_x, corner_y, max_pts);
    if (num <= 0)
    {
        return num;
    }

    std::array<int, kProcHeight> border_x{};
    border_x.fill(-1);
    for (int i = 0; i < num; ++i)
    {
        const int y = std::clamp(pts[i].y, 0, kProcHeight - 1);
        if (y <= 0 || y >= kProcHeight - 1)
        {
            continue;
        }
        border_x[y] = std::clamp(pts[i].x, 0, kProcWidth - 1);
    }

    const int extrapolate_y_span = std::max(0, g_vision_runtime_config.cross_lower_corner_extrapolate_y_span);
    const int start_y = std::clamp(corner_y, 1, kProcHeight - 2);
    const int end_y = std::clamp(corner_y - extrapolate_y_span, 1, kProcHeight - 2);
    for (int y = start_y - 1; y >= end_y; --y)
    {
        const int x = static_cast<int>(std::lround(slope * static_cast<float>(y) + intercept));
        border_x[y] = std::clamp(x, 0, kProcWidth - 1);
    }

    return rebuild_boundary_points_from_row_table(border_x, pts, std::min(max_pts, VISION_BOUNDARY_NUM));
}

static int truncate_boundary_at_cross_lower_corner_inplace(maze_point_t *pts,
                                                           int num,
                                                           int corner_x,
                                                           int corner_y,
                                                           int max_pts)
{
    if (pts == nullptr || num <= 0 || max_pts <= 0)
    {
        return num;
    }

    std::array<int, kProcHeight> border_x{};
    border_x.fill(-1);
    for (int i = 0; i < num; ++i)
    {
        const int y = std::clamp(pts[i].y, 0, kProcHeight - 1);
        if (y <= 0 || y >= kProcHeight - 1)
        {
            continue;
        }
        // 角点之后的真实边界不再使用，仅保留角点及其下方边界。
        if (y < corner_y)
        {
            continue;
        }
        border_x[y] = std::clamp(pts[i].x, 0, kProcWidth - 1);
    }

    border_x[std::clamp(corner_y, 1, kProcHeight - 2)] = std::clamp(corner_x, 0, kProcWidth - 1);
    return rebuild_boundary_points_from_row_table(border_x, pts, std::min(max_pts, VISION_BOUNDARY_NUM));
}

static int copy_boundary_points(const maze_point_t *src, int src_num, maze_point_t *dst, int max_dst)
{
    if (src == nullptr || dst == nullptr || src_num <= 0 || max_dst <= 0)
    {
        return 0;
    }

    const int count = std::min(src_num, max_dst);
    std::copy_n(src, count, dst);
    return count;
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
    g_zebra_cross_count.store(0);
    g_processed_frame_seq.store(0U);
    return true;
}

void vision_image_processor_cleanup()
{
    vision_frame_capture_cleanup();
    g_undistort_map_x.release();
    g_undistort_map_y.release();
    g_undistort_ready = false;
    g_processed_frame_seq.store(0U);
}

bool vision_image_processor_process_step()
{
    // 主处理步骤：
    // 1) 等待新帧；
    // 2) 生成灰度/二值图；
    // 3) 迷宫法双边线提取；
    // 4) 计算中线误差与逆透视结果；
    // 5) 更新时间统计，并把“成功处理帧序号”向前推进一次。
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

    // 斑马线检测：在灰度图 y=40 行按步长 3 采样，统计相邻采样点灰度跳变次数。
    // 若跳变次数 > 7，则认为命中一次斑马线，累计计数 +1。
    if (g_vision_runtime_config.zebra_cross_detection_enabled)
    {
        const int jump_count = count_gray_row_threshold_jumps(g_image_gray,
                                                              kProcWidth,
                                                              kProcHeight,
                                                              80,
                                                              3,
                                                              50);
        if (jump_count > 7)
        {
            g_zebra_cross_count.fetch_add(1);
        }
    }

    auto t_pre_end = std::chrono::steady_clock::now();

    auto t_otsu_start = t_pre_end;
    uint8 otsu_threshold = 127;
    if (g_vision_processor_config.demand_otsu_enable)
    {
        otsu_threshold = compute_global_otsu_threshold_u8(g_image_gray);
        build_binary_image_from_gray_threshold(g_image_gray, otsu_threshold);
    }
    else
    {
        cv::Mat gray_ipm(kProcHeight, kProcWidth, CV_8UC1, g_image_gray);
        cv::Mat binary(kProcHeight, kProcWidth, CV_8UC1, g_image_binary_u8);
        const double otsu_value = cv::threshold(gray_ipm, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        otsu_threshold = static_cast<uint8>(std::clamp(static_cast<int>(std::lround(otsu_value)), 0, 255));
    }
    filter_binary_image_inplace(g_image_binary_u8);
    draw_binary_black_frame(g_image_binary_u8);
    g_last_otsu_threshold = otsu_threshold;
    auto t_otsu_end = std::chrono::steady_clock::now();

    auto t_maze_start = t_otsu_end;
    const int y_min = std::max(1, kProcHeight - (kProcHeight * g_vision_processor_config.maze_lower_region_percent) / 100);
    const uint8 *classify_img = g_image_binary_u8;
    uint8 classify_white_threshold = 127;

    int left_start_x = 0;
    int left_start_y = 0;
    int right_start_x = 0;
    int right_start_y = 0;
    int maze_trace_x_min = 1;
    int maze_trace_x_max = kProcWidth - 2;
    bool left_wall_is_white = false;
    bool right_wall_is_white = false;

    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_trace_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_trace_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_trace_pts_raw{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_trace_pts_raw{};
    std::array<uint8, VISION_BOUNDARY_NUM> left_trace_dirs_raw{};
    std::array<uint8, VISION_BOUNDARY_NUM> right_trace_dirs_raw{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_regular_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_regular_pts{};
    std::array<uint8, VISION_BOUNDARY_NUM> left_trace_dirs{};
    std::array<uint8, VISION_BOUNDARY_NUM> right_trace_dirs{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_ipm_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_ipm_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_cross_base_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_cross_base_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_cross_base_ipm_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_cross_base_ipm_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_cross_aux_trace_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_cross_aux_trace_pts{};
    std::array<uint8, VISION_BOUNDARY_NUM> left_cross_aux_trace_dirs{};
    std::array<uint8, VISION_BOUNDARY_NUM> right_cross_aux_trace_dirs{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_cross_aux_regular_pts{};
    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_cross_aux_regular_pts{};

    int left_trace_num = 0;
    int right_trace_num = 0;
    int left_trace_raw_num = 0;
    int right_trace_raw_num = 0;
    int left_regular_num = 0;
    int right_regular_num = 0;
    int left_num = 0;
    int right_num = 0;
    int left_ipm_num = 0;
    int right_ipm_num = 0;
    int left_first_frame_touch_index = -1;
    int right_first_frame_touch_index = -1;
    int left_start_frame_wall_rows = 0;
    int right_start_frame_wall_rows = 0;
    int left_corner_post_frame_wall_rows = 0;
    int right_corner_post_frame_wall_rows = 0;
    int start_boundary_gap_x = 0;
    int left_cross_aux_trace_num = 0;
    int right_cross_aux_trace_num = 0;
    int left_cross_aux_regular_num = 0;
    int right_cross_aux_regular_num = 0;
    int left_cross_base_num = 0;
    int right_cross_base_num = 0;
    int left_cross_base_ipm_num = 0;
    int right_cross_base_ipm_num = 0;
    maze_point_t left_cross_aux_transition{0, 0};
    maze_point_t right_cross_aux_transition{0, 0};
    clear_eight_neighbor_trace_cache();
    clear_cross_lower_corner_detection_cache();
    clear_cross_aux_cache();
    g_src_left_trace_has_frame_wall.store(false);
    g_src_right_trace_has_frame_wall.store(false);
    g_src_left_start_frame_wall_rows.store(0);
    g_src_right_start_frame_wall_rows.store(0);
    clear_src_circle_guide_cache();
    left_trace_dirs.fill(255);
    right_trace_dirs.fill(255);
    left_trace_dirs_raw.fill(255);
    right_trace_dirs_raw.fill(255);
    auto t_maze_setup_end = std::chrono::steady_clock::now();

    get_maze_trace_x_range_clamped(&maze_trace_x_min, &maze_trace_x_max);
    const int fallback_center_x = std::clamp(previous_src_centerline_first_x(), maze_trace_x_min, maze_trace_x_max);
    int maze_start_row = std::clamp(g_maze_start_row.load(), 1, kProcHeight - 2);
    const vision_route_state_snapshot_t route_snapshot_before_trace = vision_route_state_machine_snapshot();
    const bool left_circle_stage6_active =
        (route_snapshot_before_trace.main_state == VISION_ROUTE_MAIN_CIRCLE_LEFT) &&
        (route_snapshot_before_trace.sub_state == VISION_ROUTE_SUB_CIRCLE_LEFT_6);
    const bool right_circle_stage6_active =
        (route_snapshot_before_trace.main_state == VISION_ROUTE_MAIN_CIRCLE_RIGHT) &&
        (route_snapshot_before_trace.sub_state == VISION_ROUTE_SUB_CIRCLE_RIGHT_6);
    const bool cross2_state_active_before_trace =
        (route_snapshot_before_trace.main_state == VISION_ROUTE_MAIN_CROSS) &&
        (route_snapshot_before_trace.sub_state == VISION_ROUTE_SUB_CROSS_2);
    if (left_circle_stage6_active || right_circle_stage6_active)
    {
        maze_start_row = std::clamp(std::max(maze_start_row, g_vision_runtime_config.route_circle_stage6_maze_start_row),
                                    1,
                                    kProcHeight - 2);
    }

    // 起点搜索改为“基于同侧历史起点向对侧偏移10像素”：
    // - 左边界：上一帧左起点 + 10（向右）；
    // - 右边界：上一帧右起点 - 10（向左）。
    // 无历史时退回到中心起搜。
    const int left_search_center_x =
        ((g_last_maze_left_start_x >= maze_trace_x_min && g_last_maze_left_start_x <= maze_trace_x_max)
             ? std::clamp(g_last_maze_left_start_x + 10, maze_trace_x_min, maze_trace_x_max)
             : fallback_center_x);
    const int right_search_center_x =
        ((g_last_maze_right_start_x >= maze_trace_x_min && g_last_maze_right_start_x <= maze_trace_x_max)
             ? std::clamp(g_last_maze_right_start_x - 10, maze_trace_x_min, maze_trace_x_max)
             : fallback_center_x);
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
    const vision_trace_method_enum trace_method =
        static_cast<vision_trace_method_enum>(g_maze_trace_method.load());
    const bool trace_with_eight_neighbor = (trace_method == VISION_TRACE_METHOD_EIGHT_NEIGHBOR);
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
    if (trace_with_eight_neighbor)
    {
        if (left_ok && left_start_x > maze_trace_x_min)
        {
            --left_start_x;
        }
        if (right_ok && right_start_x < maze_trace_x_max)
        {
            ++right_start_x;
        }
        left_wall_is_white = false;
        right_wall_is_white = false;
    }
    auto t_maze_start_search_end = std::chrono::steady_clock::now();

    if (left_ok)
    {
        const int left_max_pts = std::min(static_cast<int>(left_trace_pts.size()), g_vision_processor_config.maze_trace_max_points);
        left_trace_num = trace_left_boundary_selected_method(classify_img,
                                                             classify_white_threshold,
                                                             left_wall_is_white,
                                                             left_start_x,
                                                             left_start_y,
                                                             y_min,
                                                             maze_trace_x_min,
                                                             maze_trace_x_max,
                                                             left_trace_pts.data(),
                                                             left_trace_dirs.data(),
                                                             left_max_pts,
                                                             &left_first_frame_touch_index);
    }
    auto t_maze_left_trace_end = std::chrono::steady_clock::now();
    if (right_ok)
    {
        const int right_max_pts = std::min(static_cast<int>(right_trace_pts.size()), g_vision_processor_config.maze_trace_max_points);
        right_trace_num = trace_right_boundary_selected_method(classify_img,
                                                               classify_white_threshold,
                                                               right_wall_is_white,
                                                               right_start_x,
                                                               right_start_y,
                                                               y_min,
                                                               maze_trace_x_min,
                                                               maze_trace_x_max,
                                                               right_trace_pts.data(),
                                                               right_trace_dirs.data(),
                                                               right_max_pts,
                                                               &right_first_frame_touch_index);
    }

    if (trace_with_eight_neighbor)
    {
        save_eight_neighbor_trace_cache(true,
                                        left_trace_pts.data(),
                                        left_trace_dirs.data(),
                                        left_trace_num,
                                        left_first_frame_touch_index);
        save_eight_neighbor_trace_cache(false,
                                        right_trace_pts.data(),
                                        right_trace_dirs.data(),
                                        right_trace_num,
                                        right_first_frame_touch_index);
        left_trace_raw_num = copy_boundary_points(left_trace_pts.data(),
                                                  left_trace_num,
                                                  left_trace_pts_raw.data(),
                                                  static_cast<int>(left_trace_pts_raw.size()));
        right_trace_raw_num = copy_boundary_points(right_trace_pts.data(),
                                                   right_trace_num,
                                                   right_trace_pts_raw.data(),
                                                   static_cast<int>(right_trace_pts_raw.size()));
        std::copy_n(left_trace_dirs.data(),
                    std::clamp(left_trace_num, 0, VISION_BOUNDARY_NUM),
                    left_trace_dirs_raw.data());
        std::copy_n(right_trace_dirs.data(),
                    std::clamp(right_trace_num, 0, VISION_BOUNDARY_NUM),
                    right_trace_dirs_raw.data());

        if (!cross2_state_active_before_trace)
        {
            trim_initial_artificial_frame_prefix_inplace(left_trace_pts.data(),
                                                         left_trace_dirs.data(),
                                                         &left_trace_num);
            trim_initial_artificial_frame_prefix_inplace(right_trace_pts.data(),
                                                         right_trace_dirs.data(),
                                                         &right_trace_num);
        }
        const int left_trimmed_first_frame_touch_index = find_first_artificial_frame_touch_index(left_trace_pts.data(),
                                                                                                 left_trace_num);
        const int right_trimmed_first_frame_touch_index = find_first_artificial_frame_touch_index(right_trace_pts.data(),
                                                                                                   right_trace_num);
        const int left_corner_trace_num = cross2_state_active_before_trace
                                              ? left_trace_num
                                              : trace_num_for_ipm_artificial_frame_policy(left_trace_pts.data(),
                                                                                          left_trace_num,
                                                                                          left_trimmed_first_frame_touch_index);
        const int right_corner_trace_num = cross2_state_active_before_trace
                                               ? right_trace_num
                                               : trace_num_for_ipm_artificial_frame_policy(right_trace_pts.data(),
                                                                                           right_trace_num,
                                                                                           right_trimmed_first_frame_touch_index);
        update_cross_lower_corner_detection_cache(left_trace_pts.data(),
                                                  left_trace_dirs.data(),
                                                  left_corner_trace_num,
                                                  right_trace_pts.data(),
                                                  right_trace_dirs.data(),
                                                  right_corner_trace_num);
        update_src_trace_frame_wall_cache(left_trace_pts.data(),
                                          left_corner_trace_num,
                                          right_trace_pts.data(),
                                          right_corner_trace_num);
        g_src_left_boundary_straight_detected.store(
            detect_src_straight_boundary_from_dirs(left_trace_dirs.data(), left_corner_trace_num));
        g_src_right_boundary_straight_detected.store(
            detect_src_straight_boundary_from_dirs(right_trace_dirs.data(), right_corner_trace_num));

        left_regular_num = extract_one_point_per_row_from_contour(left_trace_pts.data(),
                                                                  left_trace_num,
                                                                  true,
                                                                  false,
                                                                  left_regular_pts.data(),
                                                                  static_cast<int>(left_regular_pts.size()));
        right_regular_num = extract_one_point_per_row_from_contour(right_trace_pts.data(),
                                                                   right_trace_num,
                                                                   false,
                                                                   false,
                                                                   right_regular_pts.data(),
                                                                   static_cast<int>(right_regular_pts.size()));
        left_num = copy_boundary_points(left_regular_pts.data(),
                                        left_regular_num,
                                        left_pts.data(),
                                        static_cast<int>(left_pts.size()));
        right_num = copy_boundary_points(right_regular_pts.data(),
                                         right_regular_num,
                                         right_pts.data(),
                                         static_cast<int>(right_pts.size()));
        left_start_frame_wall_rows = count_leading_side_frame_wall_rows(left_trace_pts_raw.data(), left_trace_raw_num, true);
        right_start_frame_wall_rows = count_leading_side_frame_wall_rows(right_trace_pts_raw.data(), right_trace_raw_num, false);
        g_src_left_start_frame_wall_rows.store(left_start_frame_wall_rows);
        g_src_right_start_frame_wall_rows.store(right_start_frame_wall_rows);
        start_boundary_gap_x = (left_num > 0 && right_num > 0) ? (right_pts[0].x - left_pts[0].x) : 0;
        left_corner_post_frame_wall_rows = count_side_frame_wall_rows_after_index(left_trace_pts.data(),
                                                                                  left_trace_num,
                                                                                  g_cross_lower_left_corner_index.load(),
                                                                                  true);
        right_corner_post_frame_wall_rows = count_side_frame_wall_rows_after_index(right_trace_pts.data(),
                                                                                   right_trace_num,
                                                                                   g_cross_lower_right_corner_index.load(),
                                                                                   false);
        g_cross_left_corner_post_frame_wall_rows.store(left_corner_post_frame_wall_rows);
        g_cross_right_corner_post_frame_wall_rows.store(right_corner_post_frame_wall_rows);
        g_cross_start_boundary_gap_x.store(start_boundary_gap_x);

        const int left_ipm_trace_num = cross2_state_active_before_trace
                                           ? left_trace_num
                                           : trace_num_for_ipm_artificial_frame_policy(left_trace_pts.data(),
                                                                                       left_trace_num,
                                                                                       left_trimmed_first_frame_touch_index);
        const int right_ipm_trace_num = cross2_state_active_before_trace
                                            ? right_trace_num
                                            : trace_num_for_ipm_artificial_frame_policy(right_trace_pts.data(),
                                                                                        right_trace_num,
                                                                                        right_trimmed_first_frame_touch_index);
        left_ipm_num = extract_one_point_per_row_from_contour(left_trace_pts.data(),
                                                              left_ipm_trace_num,
                                                              true,
                                                              !cross2_state_active_before_trace,
                                                              left_ipm_pts.data(),
                                                              static_cast<int>(left_ipm_pts.size()));
        right_ipm_num = extract_one_point_per_row_from_contour(right_trace_pts.data(),
                                                               right_ipm_trace_num,
                                                               false,
                                                               !cross2_state_active_before_trace,
                                                               right_ipm_pts.data(),
                                                               static_cast<int>(right_ipm_pts.size()));
        if (g_cross_lower_left_corner_found.load())
        {
            if (build_cross_aux_boundary(classify_img,
                                         classify_white_threshold,
                                         left_wall_is_white,
                                         true,
                                         g_cross_lower_left_corner_x.load(),
                                         g_cross_lower_left_corner_y.load(),
                                         maze_trace_x_min,
                                         maze_trace_x_max,
                                         left_cross_aux_trace_pts.data(),
                                         left_cross_aux_trace_dirs.data(),
                                         &left_cross_aux_trace_num,
                                         left_cross_aux_regular_pts.data(),
                                         &left_cross_aux_regular_num,
                                         &left_cross_aux_transition))
            {
                g_cross_left_aux_found.store(true);
                g_cross_left_aux_transition_x.store(left_cross_aux_transition.x);
                g_cross_left_aux_transition_y.store(left_cross_aux_transition.y);
                save_cross_aux_trace_cache(true,
                                           left_cross_aux_trace_pts.data(),
                                           left_cross_aux_trace_dirs.data(),
                                           left_cross_aux_trace_num);
                save_cross_aux_regular_cache(true,
                                             left_cross_aux_regular_pts.data(),
                                             left_cross_aux_regular_num);
            }
        }
        if (g_cross_lower_right_corner_found.load())
        {
            if (build_cross_aux_boundary(classify_img,
                                         classify_white_threshold,
                                         right_wall_is_white,
                                         false,
                                         g_cross_lower_right_corner_x.load(),
                                         g_cross_lower_right_corner_y.load(),
                                         maze_trace_x_min,
                                         maze_trace_x_max,
                                         right_cross_aux_trace_pts.data(),
                                         right_cross_aux_trace_dirs.data(),
                                         &right_cross_aux_trace_num,
                                         right_cross_aux_regular_pts.data(),
                                         &right_cross_aux_regular_num,
                                         &right_cross_aux_transition))
            {
                g_cross_right_aux_found.store(true);
                g_cross_right_aux_transition_x.store(right_cross_aux_transition.x);
                g_cross_right_aux_transition_y.store(right_cross_aux_transition.y);
                save_cross_aux_trace_cache(false,
                                           right_cross_aux_trace_pts.data(),
                                           right_cross_aux_trace_dirs.data(),
                                           right_cross_aux_trace_num);
                save_cross_aux_regular_cache(false,
                                             right_cross_aux_regular_pts.data(),
                                             right_cross_aux_regular_num);
            }
        }
        if (g_cross_lower_left_corner_found.load())
        {
            left_num = truncate_boundary_at_cross_lower_corner_inplace(left_pts.data(),
                                                                       left_num,
                                                                       g_cross_lower_left_corner_x.load(),
                                                                       g_cross_lower_left_corner_y.load(),
                                                                       static_cast<int>(left_pts.size()));
            left_ipm_num = truncate_boundary_at_cross_lower_corner_inplace(left_ipm_pts.data(),
                                                                           left_ipm_num,
                                                                           g_cross_lower_left_corner_x.load(),
                                                                           g_cross_lower_left_corner_y.load(),
                                                                           static_cast<int>(left_ipm_pts.size()));
            left_cross_base_num = copy_boundary_points(left_pts.data(),
                                                       left_num,
                                                       left_cross_base_pts.data(),
                                                       static_cast<int>(left_cross_base_pts.size()));
            left_cross_base_ipm_num = copy_boundary_points(left_ipm_pts.data(),
                                                           left_ipm_num,
                                                           left_cross_base_ipm_pts.data(),
                                                           static_cast<int>(left_cross_base_ipm_pts.size()));
        }
        if (g_cross_lower_right_corner_found.load())
        {
            right_num = truncate_boundary_at_cross_lower_corner_inplace(right_pts.data(),
                                                                        right_num,
                                                                        g_cross_lower_right_corner_x.load(),
                                                                        g_cross_lower_right_corner_y.load(),
                                                                        static_cast<int>(right_pts.size()));
            right_ipm_num = truncate_boundary_at_cross_lower_corner_inplace(right_ipm_pts.data(),
                                                                            right_ipm_num,
                                                                            g_cross_lower_right_corner_x.load(),
                                                                            g_cross_lower_right_corner_y.load(),
                                                                            static_cast<int>(right_ipm_pts.size()));
            right_cross_base_num = copy_boundary_points(right_pts.data(),
                                                        right_num,
                                                        right_cross_base_pts.data(),
                                                        static_cast<int>(right_cross_base_pts.size()));
            right_cross_base_ipm_num = copy_boundary_points(right_ipm_pts.data(),
                                                            right_ipm_num,
                                                            right_cross_base_ipm_pts.data(),
                                                            static_cast<int>(right_cross_base_ipm_pts.size()));
        }
        const bool cross_state_active =
            (route_snapshot_before_trace.main_state == VISION_ROUTE_MAIN_CROSS);
        const bool cross1_state_active =
            (route_snapshot_before_trace.main_state == VISION_ROUTE_MAIN_CROSS) &&
            (route_snapshot_before_trace.sub_state == VISION_ROUTE_SUB_CROSS_1);
        const bool lower_corner_extrapolate_enabled =
            g_vision_runtime_config.cross_lower_corner_extrapolate_enabled && !cross_state_active;
        const bool skip_left_auto_extrapolate =
            !lower_corner_extrapolate_enabled || (cross1_state_active && g_cross_left_aux_found.load());
        const bool skip_right_auto_extrapolate =
            !lower_corner_extrapolate_enabled || (cross1_state_active && g_cross_right_aux_found.load());
        if (!skip_left_auto_extrapolate && left_num > 0)
        {
            const int extrapolate_min_y = g_vision_runtime_config.cross_lower_corner_extrapolate_min_y;
            const bool left_corner_valid =
                g_cross_lower_left_corner_found.load() &&
                (g_cross_lower_left_corner_y.load() > extrapolate_min_y);
            if (left_corner_valid)
            {
                left_num = extrapolate_cross_lower_boundary_inplace(left_pts.data(),
                                                                    left_num,
                                                                    g_cross_lower_left_corner_x.load(),
                                                                    g_cross_lower_left_corner_y.load(),
                                                                    static_cast<int>(left_pts.size()));
            }
        }
        if (!skip_right_auto_extrapolate && right_num > 0)
        {
            const int extrapolate_min_y = g_vision_runtime_config.cross_lower_corner_extrapolate_min_y;
            const bool right_corner_valid =
                g_cross_lower_right_corner_found.load() &&
                (g_cross_lower_right_corner_y.load() > extrapolate_min_y);
            if (right_corner_valid)
            {
                right_num = extrapolate_cross_lower_boundary_inplace(right_pts.data(),
                                                                     right_num,
                                                                     g_cross_lower_right_corner_x.load(),
                                                                     g_cross_lower_right_corner_y.load(),
                                                                     static_cast<int>(right_pts.size()));
            }
        }
        if (!skip_left_auto_extrapolate && left_ipm_num > 0)
        {
            const int extrapolate_min_y = g_vision_runtime_config.cross_lower_corner_extrapolate_min_y;
            const bool left_corner_valid =
                g_cross_lower_left_corner_found.load() &&
                (g_cross_lower_left_corner_y.load() > extrapolate_min_y);
            if (left_corner_valid)
            {
                left_ipm_num = extrapolate_cross_lower_boundary_inplace(left_ipm_pts.data(),
                                                                        left_ipm_num,
                                                                        g_cross_lower_left_corner_x.load(),
                                                                        g_cross_lower_left_corner_y.load(),
                                                                        static_cast<int>(left_ipm_pts.size()));
            }
        }
        if (!skip_right_auto_extrapolate && right_ipm_num > 0)
        {
            const int extrapolate_min_y = g_vision_runtime_config.cross_lower_corner_extrapolate_min_y;
            const bool right_corner_valid =
                g_cross_lower_right_corner_found.load() &&
                (g_cross_lower_right_corner_y.load() > extrapolate_min_y);
            if (right_corner_valid)
            {
                right_ipm_num = extrapolate_cross_lower_boundary_inplace(right_ipm_pts.data(),
                                                                         right_ipm_num,
                                                                         g_cross_lower_right_corner_x.load(),
                                                                         g_cross_lower_right_corner_y.load(),
                                                                         static_cast<int>(right_ipm_pts.size()));
            }
        }
    }
    else
    {
        g_src_left_boundary_straight_detected.store(false);
        g_src_right_boundary_straight_detected.store(false);
        left_num = copy_boundary_points(left_trace_pts.data(), left_trace_num, left_pts.data(), static_cast<int>(left_pts.size()));
        right_num = copy_boundary_points(right_trace_pts.data(), right_trace_num, right_pts.data(), static_cast<int>(right_pts.size()));
        left_regular_num = copy_boundary_points(left_pts.data(),
                                                left_num,
                                                left_regular_pts.data(),
                                                static_cast<int>(left_regular_pts.size()));
        right_regular_num = copy_boundary_points(right_pts.data(),
                                                 right_num,
                                                 right_regular_pts.data(),
                                                 static_cast<int>(right_regular_pts.size()));
        left_ipm_num = copy_boundary_points(left_pts.data(), left_num, left_ipm_pts.data(), static_cast<int>(left_ipm_pts.size()));
        right_ipm_num = copy_boundary_points(right_pts.data(), right_num, right_ipm_pts.data(), static_cast<int>(right_ipm_pts.size()));
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
                left_ipm_num = 0;
            }
            else
            {
                right_ok = false;
                right_num = 0;
                right_ipm_num = 0;
            }
        }
    }
    auto t_maze_trace_end = std::chrono::steady_clock::now();

    vision_route_state_input_t route_input{};
    route_input.base_preferred_source = g_ipm_line_error_preferred_source.load();
    route_input.left_corner_found = g_cross_lower_left_corner_found.load();
    route_input.right_corner_found = g_cross_lower_right_corner_found.load();
    route_input.left_corner_x = g_cross_lower_left_corner_x.load();
    route_input.left_corner_y = g_cross_lower_left_corner_y.load();
    route_input.left_corner_src_y = g_cross_lower_left_corner_y.load();
    route_input.left_corner_index = g_cross_lower_left_corner_index.load();
    route_input.left_corner_post_frame_wall_rows = left_corner_post_frame_wall_rows;
    route_input.right_corner_x = g_cross_lower_right_corner_x.load();
    route_input.right_corner_y = g_cross_lower_right_corner_y.load();
    route_input.right_corner_src_y = g_cross_lower_right_corner_y.load();
    route_input.right_corner_index = g_cross_lower_right_corner_index.load();
    route_input.right_corner_post_frame_wall_rows = right_corner_post_frame_wall_rows;
    route_input.left_has_frame_wall = g_src_left_trace_has_frame_wall.load();
    route_input.right_has_frame_wall = g_src_right_trace_has_frame_wall.load();
    route_input.left_start_frame_wall_rows = left_start_frame_wall_rows;
    route_input.right_start_frame_wall_rows = right_start_frame_wall_rows;
    route_input.start_boundary_gap_x = start_boundary_gap_x;
    route_input.left_straight = g_src_left_boundary_straight_detected.load();
    route_input.right_straight = g_src_right_boundary_straight_detected.load();
    route_input.left_boundary_count = left_num;
    route_input.right_boundary_count = right_num;
    route_input.selected_centerline_count = vision_line_error_layer_selected_centerline_count();
    route_input.straight_required_last_index = vision_line_error_layer_required_last_index_for_straight();
    route_input.straight_abs_error_sum = vision_line_error_layer_abs_error_sum_to_required_index();
    route_input.left_circle_entry_raw_gap_ok = circle_entry_raw_boundary_gap_ready(left_trace_pts_raw.data(),
                                                                                   left_trace_raw_num,
                                                                                   right_trace_pts_raw.data(),
                                                                                   right_trace_raw_num,
                                                                                   route_input.left_corner_src_y);
    route_input.right_circle_entry_raw_gap_ok = circle_entry_raw_boundary_gap_ready(left_trace_pts_raw.data(),
                                                                                    left_trace_raw_num,
                                                                                    right_trace_pts_raw.data(),
                                                                                    right_trace_raw_num,
                                                                                    route_input.right_corner_src_y);
    route_input.frame_encoder_delta = static_cast<uint32>(std::lround((std::fabs(motor_thread_left_count()) + std::fabs(motor_thread_right_count())) * 0.5f));
    vision_route_state_machine_update(&route_input);
    const vision_route_state_snapshot_t route_snapshot = vision_route_state_machine_snapshot();

    if (route_snapshot.main_state == VISION_ROUTE_MAIN_CROSS)
    {
        if (route_snapshot.sub_state == VISION_ROUTE_SUB_CROSS_1)
        {
            if (g_cross_left_aux_found.load() && left_cross_aux_regular_num > 0)
            {
                maze_point_t left_upper_corner{};
                int left_upper_trace_index = -1;
                if (find_nth_dir_point_on_trace(left_cross_aux_trace_pts.data(),
                                                left_cross_aux_trace_dirs.data(),
                                                left_cross_aux_trace_num,
                                                5,
                                                3,
                                                &left_upper_corner,
                                                &left_upper_trace_index))
                {
                    g_cross_left_upper_corner_found.store(true);
                    g_cross_left_upper_corner_index.store(left_upper_trace_index);
                    g_cross_left_upper_corner_x.store(left_upper_corner.x);
                    g_cross_left_upper_corner_y.store(left_upper_corner.y);

                    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_aux_tail{};
                    const int left_aux_tail_num = copy_boundary_points(left_cross_aux_regular_pts.data(),
                                                                       left_cross_aux_regular_num,
                                                                       left_aux_tail.data(),
                                                                       static_cast<int>(left_aux_tail.size()));
                    const int left_aux_tail_truncated_num =
                        truncate_regular_boundary_before_point_inplace(left_aux_tail.data(),
                                                                      left_aux_tail_num,
                                                                      left_upper_corner);

                    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_bridge_pts{};
                    const maze_point_t left_lower_corner{
                        g_cross_lower_left_corner_x.load(),
                        g_cross_lower_left_corner_y.load()
                    };
                    const int left_bridge_num = build_line_points_between_with_y_step(left_lower_corner,
                                                                                      left_upper_corner,
                                                                                      2,
                                                                                      left_bridge_pts.data(),
                                                                                      static_cast<int>(left_bridge_pts.size()));

                    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_combined_pts{};
                    const maze_point_t *left_base_pts = (left_cross_base_num > 0) ? left_cross_base_pts.data() : left_pts.data();
                    const int left_base_num = (left_cross_base_num > 0) ? left_cross_base_num : left_num;
                    const int left_combined_num = concatenate_boundary_segments(left_base_pts,
                                                                                left_base_num,
                                                                                left_bridge_pts.data(),
                                                                                left_bridge_num,
                                                                                left_aux_tail.data(),
                                                                                left_aux_tail_truncated_num,
                                                                                left_combined_pts.data(),
                                                                                static_cast<int>(left_combined_pts.size()));
                    left_num = copy_boundary_points(left_combined_pts.data(),
                                                    left_combined_num,
                                                    left_pts.data(),
                                                    static_cast<int>(left_pts.size()));
                    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_combined_ipm_pts{};
                    const maze_point_t *left_base_ipm_pts = (left_cross_base_ipm_num > 0) ? left_cross_base_ipm_pts.data() : left_ipm_pts.data();
                    const int left_base_ipm_num = (left_cross_base_ipm_num > 0) ? left_cross_base_ipm_num : left_ipm_num;
                    const int left_combined_ipm_num = concatenate_boundary_segments(left_base_ipm_pts,
                                                                                    left_base_ipm_num,
                                                                                    left_bridge_pts.data(),
                                                                                    left_bridge_num,
                                                                                    left_aux_tail.data(),
                                                                                    left_aux_tail_truncated_num,
                                                                                    left_combined_ipm_pts.data(),
                                                                                    static_cast<int>(left_combined_ipm_pts.size()));
                    left_ipm_num = copy_boundary_points(left_combined_ipm_pts.data(),
                                                        left_combined_ipm_num,
                                                        left_ipm_pts.data(),
                                                        static_cast<int>(left_ipm_pts.size()));
                }
            }
            if (g_cross_right_aux_found.load() && right_cross_aux_regular_num > 0)
            {
                maze_point_t right_upper_corner{};
                int right_upper_trace_index = -1;
                if (find_nth_dir_point_on_trace(right_cross_aux_trace_pts.data(),
                                                right_cross_aux_trace_dirs.data(),
                                                right_cross_aux_trace_num,
                                                5,
                                                3,
                                                &right_upper_corner,
                                                &right_upper_trace_index))
                {
                    g_cross_right_upper_corner_found.store(true);
                    g_cross_right_upper_corner_index.store(right_upper_trace_index);
                    g_cross_right_upper_corner_x.store(right_upper_corner.x);
                    g_cross_right_upper_corner_y.store(right_upper_corner.y);

                    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_aux_tail{};
                    const int right_aux_tail_num = copy_boundary_points(right_cross_aux_regular_pts.data(),
                                                                        right_cross_aux_regular_num,
                                                                        right_aux_tail.data(),
                                                                        static_cast<int>(right_aux_tail.size()));
                    const int right_aux_tail_truncated_num =
                        truncate_regular_boundary_before_point_inplace(right_aux_tail.data(),
                                                                      right_aux_tail_num,
                                                                      right_upper_corner);

                    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_bridge_pts{};
                    const maze_point_t right_lower_corner{
                        g_cross_lower_right_corner_x.load(),
                        g_cross_lower_right_corner_y.load()
                    };
                    const int right_bridge_num = build_line_points_between_with_y_step(right_lower_corner,
                                                                                       right_upper_corner,
                                                                                       2,
                                                                                       right_bridge_pts.data(),
                                                                                       static_cast<int>(right_bridge_pts.size()));

                    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_combined_pts{};
                    const maze_point_t *right_base_pts = (right_cross_base_num > 0) ? right_cross_base_pts.data() : right_pts.data();
                    const int right_base_num = (right_cross_base_num > 0) ? right_cross_base_num : right_num;
                    const int right_combined_num = concatenate_boundary_segments(right_base_pts,
                                                                                 right_base_num,
                                                                                 right_bridge_pts.data(),
                                                                                 right_bridge_num,
                                                                                 right_aux_tail.data(),
                                                                                 right_aux_tail_truncated_num,
                                                                                 right_combined_pts.data(),
                                                                                 static_cast<int>(right_combined_pts.size()));
                    right_num = copy_boundary_points(right_combined_pts.data(),
                                                     right_combined_num,
                                                     right_pts.data(),
                                                     static_cast<int>(right_pts.size()));
                    std::array<maze_point_t, VISION_BOUNDARY_NUM> right_combined_ipm_pts{};
                    const maze_point_t *right_base_ipm_pts = (right_cross_base_ipm_num > 0) ? right_cross_base_ipm_pts.data() : right_ipm_pts.data();
                    const int right_base_ipm_num = (right_cross_base_ipm_num > 0) ? right_cross_base_ipm_num : right_ipm_num;
                    const int right_combined_ipm_num = concatenate_boundary_segments(right_base_ipm_pts,
                                                                                     right_base_ipm_num,
                                                                                     right_bridge_pts.data(),
                                                                                     right_bridge_num,
                                                                                     right_aux_tail.data(),
                                                                                     right_aux_tail_truncated_num,
                                                                                     right_combined_ipm_pts.data(),
                                                                                     static_cast<int>(right_combined_ipm_pts.size()));
                    right_ipm_num = copy_boundary_points(right_combined_ipm_pts.data(),
                                                         right_combined_ipm_num,
                                                         right_ipm_pts.data(),
                                                         static_cast<int>(right_ipm_pts.size()));
                }
            }
        }
        else if (route_snapshot.sub_state == VISION_ROUTE_SUB_CROSS_2)
        {
            static constexpr int kCross2JumpXThresholdPx = 15;
            static constexpr int kCross2CutForwardPoints = 2;
            const maze_point_t left_anchor{15, 100};
            const maze_point_t right_anchor{145, 100};

            maze_point_t left_cut_point{};
            int left_cut_index = -1;
            if (find_regular_boundary_jump_cut_point(left_regular_pts.data(),
                                                     left_regular_num,
                                                     kCross2JumpXThresholdPx,
                                                     kCross2CutForwardPoints,
                                                     &left_cut_point,
                                                     &left_cut_index))
            {
                std::array<maze_point_t, VISION_BOUNDARY_NUM> left_tail_pts{};
                const int left_tail_num = copy_boundary_points(left_regular_pts.data() + left_cut_index,
                                                               left_regular_num - left_cut_index,
                                                               left_tail_pts.data(),
                                                               static_cast<int>(left_tail_pts.size()));
                std::array<maze_point_t, VISION_BOUNDARY_NUM> left_bridge_pts{};
                const int left_bridge_num = build_line_points_between(left_anchor,
                                                                      left_cut_point,
                                                                      left_bridge_pts.data(),
                                                                      static_cast<int>(left_bridge_pts.size()));
                std::array<maze_point_t, VISION_BOUNDARY_NUM> left_combined_ipm_pts{};
                const int left_combined_ipm_num = concatenate_boundary_segments(left_bridge_pts.data(),
                                                                                left_bridge_num,
                                                                                left_tail_pts.data(),
                                                                                left_tail_num,
                                                                                nullptr,
                                                                                0,
                                                                                left_combined_ipm_pts.data(),
                                                                                static_cast<int>(left_combined_ipm_pts.size()));
                left_ipm_num = copy_boundary_points(left_combined_ipm_pts.data(),
                                                    left_combined_ipm_num,
                                                    left_ipm_pts.data(),
                                                    static_cast<int>(left_ipm_pts.size()));
            }

            maze_point_t right_cut_point{};
            int right_cut_index = -1;
            if (find_regular_boundary_jump_cut_point(right_regular_pts.data(),
                                                     right_regular_num,
                                                     kCross2JumpXThresholdPx,
                                                     kCross2CutForwardPoints,
                                                     &right_cut_point,
                                                     &right_cut_index))
            {
                std::array<maze_point_t, VISION_BOUNDARY_NUM> right_tail_pts{};
                const int right_tail_num = copy_boundary_points(right_regular_pts.data() + right_cut_index,
                                                                right_regular_num - right_cut_index,
                                                                right_tail_pts.data(),
                                                                static_cast<int>(right_tail_pts.size()));
                std::array<maze_point_t, VISION_BOUNDARY_NUM> right_bridge_pts{};
                const int right_bridge_num = build_line_points_between(right_anchor,
                                                                       right_cut_point,
                                                                       right_bridge_pts.data(),
                                                                       static_cast<int>(right_bridge_pts.size()));
                std::array<maze_point_t, VISION_BOUNDARY_NUM> right_combined_ipm_pts{};
                const int right_combined_ipm_num = concatenate_boundary_segments(right_bridge_pts.data(),
                                                                                 right_bridge_num,
                                                                                 right_tail_pts.data(),
                                                                                 right_tail_num,
                                                                                 nullptr,
                                                                                 0,
                                                                                 right_combined_ipm_pts.data(),
                                                                                 static_cast<int>(right_combined_ipm_pts.size()));
                right_ipm_num = copy_boundary_points(right_combined_ipm_pts.data(),
                                                     right_combined_ipm_num,
                                                     right_ipm_pts.data(),
                                                     static_cast<int>(right_ipm_pts.size()));
            }
        }
    }

    if (route_snapshot.main_state == VISION_ROUTE_MAIN_CIRCLE_LEFT)
    {
        if (route_snapshot.sub_state == VISION_ROUTE_SUB_CIRCLE_LEFT_3)
        {
            const maze_point_t fixed_start = (right_regular_num > 0) ? right_regular_pts[0]
                                                                     : ((right_num > 0) ? right_pts[0] : maze_point_t{kProcWidth - 2, maze_start_row});
            maze_point_t target = fixed_start;
            if (find_circle_stage3_target_on_regular_boundary(left_regular_pts.data(),
                                                              left_regular_num,
                                                              true,
                                                              g_vision_runtime_config.circle_guide_target_offset_stage3,
                                                              &target))
            {
                std::array<maze_point_t, VISION_BOUNDARY_NUM> guide_pts{};
                const int guide_num = build_line_points_between(fixed_start,
                                                                target,
                                                                guide_pts.data(),
                                                                static_cast<int>(guide_pts.size()));
                if (guide_num > 0)
                {
                    copy_boundary_points(guide_pts.data(), guide_num, right_pts.data(), VISION_BOUNDARY_NUM);
                    right_num = std::clamp(guide_num, 0, VISION_BOUNDARY_NUM);
                    copy_boundary_points(guide_pts.data(), guide_num, right_ipm_pts.data(), VISION_BOUNDARY_NUM);
                    right_ipm_num = std::clamp(guide_num, 0, VISION_BOUNDARY_NUM);
                    int cached_count = 0;
                    fill_single_line_arrays_from_points(guide_pts.data(),
                                                        guide_num,
                                                        g_src_right_circle_guide_x,
                                                        g_src_right_circle_guide_y,
                                                        &cached_count,
                                                        kProcWidth,
                                                        kProcHeight);
                    g_src_right_circle_guide_count = static_cast<uint16>(std::clamp(cached_count, 0, VISION_BOUNDARY_NUM));
                }
            }
        }
        else if (route_snapshot.sub_state == VISION_ROUTE_SUB_CIRCLE_LEFT_5)
        {
            const maze_point_t start = route_input.right_corner_found
                                           ? maze_point_t{route_input.right_corner_x, route_input.right_corner_y}
                                           : ((right_regular_num > 0) ? right_regular_pts[0]
                                                                      : ((right_num > 0) ? right_pts[0]
                                                                                         : maze_point_t{kProcWidth - 2, maze_start_row}));
            maze_point_t target{1, 1};
            if (!find_circle_stage5_target_on_regular_boundary(left_regular_pts.data(),
                                                               left_regular_num,
                                                               true,
                                                               &target))
            {
                target = maze_point_t{1, 1};
            }
            std::array<maze_point_t, VISION_BOUNDARY_NUM> guide_pts{};
            const int guide_num = build_line_points_between(start,
                                                            target,
                                                            guide_pts.data(),
                                                            static_cast<int>(guide_pts.size()));
            if (guide_num > 0)
            {
                copy_boundary_points(guide_pts.data(), guide_num, right_pts.data(), VISION_BOUNDARY_NUM);
                right_num = std::clamp(guide_num, 0, VISION_BOUNDARY_NUM);
                copy_boundary_points(guide_pts.data(), guide_num, right_ipm_pts.data(), VISION_BOUNDARY_NUM);
                right_ipm_num = std::clamp(guide_num, 0, VISION_BOUNDARY_NUM);
                int cached_count = 0;
                fill_single_line_arrays_from_points(guide_pts.data(),
                                                    guide_num,
                                                    g_src_right_circle_guide_x,
                                                    g_src_right_circle_guide_y,
                                                    &cached_count,
                                                    kProcWidth,
                                                    kProcHeight);
                g_src_right_circle_guide_count = static_cast<uint16>(std::clamp(cached_count, 0, VISION_BOUNDARY_NUM));
            }
        }
    }
    else if (route_snapshot.main_state == VISION_ROUTE_MAIN_CIRCLE_RIGHT)
    {
        if (route_snapshot.sub_state == VISION_ROUTE_SUB_CIRCLE_RIGHT_3)
        {
            const maze_point_t fixed_start = (left_regular_num > 0) ? left_regular_pts[0]
                                                                    : ((left_num > 0) ? left_pts[0] : maze_point_t{1, maze_start_row});
            maze_point_t target = fixed_start;
            if (find_circle_stage3_target_on_regular_boundary(right_regular_pts.data(),
                                                              right_regular_num,
                                                              false,
                                                              g_vision_runtime_config.circle_guide_target_offset_stage3,
                                                              &target))
            {
                std::array<maze_point_t, VISION_BOUNDARY_NUM> guide_pts{};
                const int guide_num = build_line_points_between(fixed_start,
                                                                target,
                                                                guide_pts.data(),
                                                                static_cast<int>(guide_pts.size()));
                if (guide_num > 0)
                {
                    copy_boundary_points(guide_pts.data(), guide_num, left_pts.data(), VISION_BOUNDARY_NUM);
                    left_num = std::clamp(guide_num, 0, VISION_BOUNDARY_NUM);
                    copy_boundary_points(guide_pts.data(), guide_num, left_ipm_pts.data(), VISION_BOUNDARY_NUM);
                    left_ipm_num = std::clamp(guide_num, 0, VISION_BOUNDARY_NUM);
                    int cached_count = 0;
                    fill_single_line_arrays_from_points(guide_pts.data(),
                                                        guide_num,
                                                        g_src_left_circle_guide_x,
                                                        g_src_left_circle_guide_y,
                                                        &cached_count,
                                                        kProcWidth,
                                                        kProcHeight);
                    g_src_left_circle_guide_count = static_cast<uint16>(std::clamp(cached_count, 0, VISION_BOUNDARY_NUM));
                }
            }
        }
        else if (route_snapshot.sub_state == VISION_ROUTE_SUB_CIRCLE_RIGHT_5)
        {
            const maze_point_t start = route_input.left_corner_found
                                           ? maze_point_t{route_input.left_corner_x, route_input.left_corner_y}
                                           : ((left_regular_num > 0) ? left_regular_pts[0]
                                                                     : ((left_num > 0) ? left_pts[0]
                                                                                       : maze_point_t{1, maze_start_row}));
            maze_point_t target{kProcWidth - 2, 1};
            if (!find_circle_stage5_target_on_regular_boundary(right_regular_pts.data(),
                                                               right_regular_num,
                                                               false,
                                                               &target))
            {
                target = maze_point_t{kProcWidth - 2, 1};
            }
            std::array<maze_point_t, VISION_BOUNDARY_NUM> guide_pts{};
            const int guide_num = build_line_points_between(start,
                                                            target,
                                                            guide_pts.data(),
                                                            static_cast<int>(guide_pts.size()));
            if (guide_num > 0)
            {
                copy_boundary_points(guide_pts.data(), guide_num, left_pts.data(), VISION_BOUNDARY_NUM);
                left_num = std::clamp(guide_num, 0, VISION_BOUNDARY_NUM);
                copy_boundary_points(guide_pts.data(), guide_num, left_ipm_pts.data(), VISION_BOUNDARY_NUM);
                left_ipm_num = std::clamp(guide_num, 0, VISION_BOUNDARY_NUM);
                int cached_count = 0;
                fill_single_line_arrays_from_points(guide_pts.data(),
                                                    guide_num,
                                                    g_src_left_circle_guide_x,
                                                    g_src_left_circle_guide_y,
                                                    &cached_count,
                                                    kProcWidth,
                                                    kProcHeight);
                g_src_left_circle_guide_count = static_cast<uint16>(std::clamp(cached_count, 0, VISION_BOUNDARY_NUM));
            }
        }
    }

    // 主输出边界使用原图坐标系：左右边线 + 均值中线（无丢线补偿）。
    fill_boundary_arrays_from_maze(left_pts.data(), left_num, right_pts.data(), right_num);
    line_error = 0;

    if (g_vision_processor_config.enable_inverse_perspective)
    {
        render_ipm_boundary_image_and_update_boundaries(left_ipm_pts.data(),
                                                        left_ipm_num,
                                                        right_ipm_pts.data(),
                                                        right_ipm_num,
                                                        classify_img,
                                                        classify_white_threshold,
                                                        y_min,
                                                        maze_trace_x_min,
                                                        maze_trace_x_max,
                                                        route_snapshot.preferred_source);
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
    g_processed_frame_seq.fetch_add(1U);

    return true;
}

uint32 vision_image_processor_processed_frame_seq()
{
    return g_processed_frame_seq.load();
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

void vision_image_processor_set_trace_method(vision_trace_method_enum method)
{
    const int method_value = (method == VISION_TRACE_METHOD_EIGHT_NEIGHBOR)
                                 ? static_cast<int>(VISION_TRACE_METHOD_EIGHT_NEIGHBOR)
                                 : static_cast<int>(VISION_TRACE_METHOD_MAZE);
    g_maze_trace_method.store(method_value);
}

vision_trace_method_enum vision_image_processor_trace_method()
{
    return (g_maze_trace_method.load() == static_cast<int>(VISION_TRACE_METHOD_EIGHT_NEIGHBOR))
               ? VISION_TRACE_METHOD_EIGHT_NEIGHBOR
               : VISION_TRACE_METHOD_MAZE;
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

void vision_image_processor_set_ipm_boundary_angle_step(int step)
{
    g_ipm_boundary_angle_step.store(std::max(1, step));
}

int vision_image_processor_ipm_boundary_angle_step()
{
    return g_ipm_boundary_angle_step.load();
}

void vision_image_processor_set_ipm_boundary_straight_min_points(int points)
{
    g_ipm_boundary_straight_min_points.store(std::max(1, points));
}

int vision_image_processor_ipm_boundary_straight_min_points()
{
    return g_ipm_boundary_straight_min_points.load();
}

void vision_image_processor_set_ipm_boundary_straight_check_count(int count)
{
    g_ipm_boundary_straight_check_count.store(std::max(1, count));
}

int vision_image_processor_ipm_boundary_straight_check_count()
{
    return g_ipm_boundary_straight_check_count.load();
}

void vision_image_processor_set_ipm_boundary_straight_min_cos(float min_cos)
{
    g_ipm_boundary_straight_min_cos.store(std::clamp(min_cos, -1.0f, 1.0f));
}

float vision_image_processor_ipm_boundary_straight_min_cos()
{
    return g_ipm_boundary_straight_min_cos.load();
}

void vision_image_processor_set_ipm_track_width_px(float width_px)
{
    g_ipm_track_width_px.store(std::max(0.0f, width_px));
}

float vision_image_processor_ipm_track_width_px()
{
    return g_ipm_track_width_px.load();
}

void vision_image_processor_set_ipm_center_target_offset_from_left_px(float offset_px)
{
    g_ipm_center_target_offset_from_left_px.store(std::max(0.0f, offset_px));
}

float vision_image_processor_ipm_center_target_offset_from_left_px()
{
    return g_ipm_center_target_offset_from_left_px.load();
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

void vision_image_processor_set_ipm_centerline_curvature_enabled(bool enabled)
{
    g_ipm_centerline_curvature_enabled.store(enabled);
    vision_line_error_layer_set_curvature_enabled(enabled);
}

bool vision_image_processor_ipm_centerline_curvature_enabled()
{
    return g_ipm_centerline_curvature_enabled.load();
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

int vision_image_processor_ipm_selected_centerline_count()
{
    return vision_line_error_layer_selected_centerline_count();
}

int vision_image_processor_ipm_straight_required_last_index()
{
    return vision_line_error_layer_required_last_index_for_straight();
}

float vision_image_processor_ipm_straight_abs_error_sum()
{
    return vision_line_error_layer_abs_error_sum_to_required_index();
}

float vision_image_processor_ipm_mean_abs_offset_error()
{
    return vision_line_error_layer_mean_abs_offset();
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

void vision_image_processor_get_eight_neighbor_left_trace(uint16 **x, uint16 **y, uint8 **dir, uint16 *dot_num, int *first_frame_touch_index)
{
    if (x) *x = g_eight_left_trace_x;
    if (y) *y = g_eight_left_trace_y;
    if (dir) *dir = g_eight_left_trace_dir;
    if (dot_num) *dot_num = static_cast<uint16>(g_eight_left_trace_count);
    if (first_frame_touch_index) *first_frame_touch_index = g_eight_left_first_frame_touch_index;
}

void vision_image_processor_get_eight_neighbor_right_trace(uint16 **x, uint16 **y, uint8 **dir, uint16 *dot_num, int *first_frame_touch_index)
{
    if (x) *x = g_eight_right_trace_x;
    if (y) *y = g_eight_right_trace_y;
    if (dir) *dir = g_eight_right_trace_dir;
    if (dot_num) *dot_num = static_cast<uint16>(g_eight_right_trace_count);
    if (first_frame_touch_index) *first_frame_touch_index = g_eight_right_first_frame_touch_index;
}

void vision_image_processor_get_cross_lower_corner_state(bool *left_found,
                                                         int *left_index,
                                                         int *left_x,
                                                         int *left_y,
                                                         bool *right_found,
                                                         int *right_index,
                                                         int *right_x,
                                                         int *right_y,
                                                         bool *pair_valid)
{
    if (left_found) *left_found = g_cross_lower_left_corner_found.load();
    if (left_index) *left_index = g_cross_lower_left_corner_index.load();
    if (left_x) *left_x = g_cross_lower_left_corner_x.load();
    if (left_y) *left_y = g_cross_lower_left_corner_y.load();
    if (right_found) *right_found = g_cross_lower_right_corner_found.load();
    if (right_index) *right_index = g_cross_lower_right_corner_index.load();
    if (right_x) *right_x = g_cross_lower_right_corner_x.load();
    if (right_y) *right_y = g_cross_lower_right_corner_y.load();
    if (pair_valid) *pair_valid = g_cross_lower_corner_pair_valid.load();
}

void vision_image_processor_get_cross_aux_line_state(bool is_left,
                                                     bool *found,
                                                     int *transition_x,
                                                     int *transition_y,
                                                     uint16 **trace_x,
                                                     uint16 **trace_y,
                                                     uint8 **trace_dir,
                                                     uint16 *trace_num,
                                                     uint16 **regular_x,
                                                     uint16 **regular_y,
                                                     uint16 *regular_num)
{
    if (is_left)
    {
        if (found) *found = g_cross_left_aux_found.load();
        if (transition_x) *transition_x = g_cross_left_aux_transition_x.load();
        if (transition_y) *transition_y = g_cross_left_aux_transition_y.load();
        if (trace_x) *trace_x = g_cross_left_aux_trace_x;
        if (trace_y) *trace_y = g_cross_left_aux_trace_y;
        if (trace_dir) *trace_dir = g_cross_left_aux_trace_dir;
        if (trace_num) *trace_num = g_cross_left_aux_trace_count;
        if (regular_x) *regular_x = g_cross_left_aux_regular_x;
        if (regular_y) *regular_y = g_cross_left_aux_regular_y;
        if (regular_num) *regular_num = g_cross_left_aux_regular_count;
        return;
    }

    if (found) *found = g_cross_right_aux_found.load();
    if (transition_x) *transition_x = g_cross_right_aux_transition_x.load();
    if (transition_y) *transition_y = g_cross_right_aux_transition_y.load();
    if (trace_x) *trace_x = g_cross_right_aux_trace_x;
    if (trace_y) *trace_y = g_cross_right_aux_trace_y;
    if (trace_dir) *trace_dir = g_cross_right_aux_trace_dir;
    if (trace_num) *trace_num = g_cross_right_aux_trace_count;
    if (regular_x) *regular_x = g_cross_right_aux_regular_x;
    if (regular_y) *regular_y = g_cross_right_aux_regular_y;
    if (regular_num) *regular_num = g_cross_right_aux_regular_count;
}

void vision_image_processor_get_cross_upper_corner_state(bool *left_found,
                                                         int *left_index,
                                                         int *left_x,
                                                         int *left_y,
                                                         bool *right_found,
                                                         int *right_index,
                                                         int *right_x,
                                                         int *right_y)
{
    if (left_found) *left_found = g_cross_left_upper_corner_found.load();
    if (left_index) *left_index = g_cross_left_upper_corner_index.load();
    if (left_x) *left_x = g_cross_left_upper_corner_x.load();
    if (left_y) *left_y = g_cross_left_upper_corner_y.load();
    if (right_found) *right_found = g_cross_right_upper_corner_found.load();
    if (right_index) *right_index = g_cross_right_upper_corner_index.load();
    if (right_x) *right_x = g_cross_right_upper_corner_x.load();
    if (right_y) *right_y = g_cross_right_upper_corner_y.load();
}

void vision_image_processor_get_cross_route_debug_state(int *left_corner_post_frame_wall_rows,
                                                        int *right_corner_post_frame_wall_rows,
                                                        int *start_boundary_gap_x)
{
    if (left_corner_post_frame_wall_rows) *left_corner_post_frame_wall_rows = g_cross_left_corner_post_frame_wall_rows.load();
    if (right_corner_post_frame_wall_rows) *right_corner_post_frame_wall_rows = g_cross_right_corner_post_frame_wall_rows.load();
    if (start_boundary_gap_x) *start_boundary_gap_x = g_cross_start_boundary_gap_x.load();
}

void vision_image_processor_get_src_trace_frame_wall_state(bool *left_has_frame_wall,
                                                           bool *right_has_frame_wall)
{
    if (left_has_frame_wall) *left_has_frame_wall = g_src_left_trace_has_frame_wall.load();
    if (right_has_frame_wall) *right_has_frame_wall = g_src_right_trace_has_frame_wall.load();
}

void vision_image_processor_get_src_start_frame_wall_rows(int *left_rows, int *right_rows)
{
    if (left_rows) *left_rows = g_src_left_start_frame_wall_rows.load();
    if (right_rows) *right_rows = g_src_right_start_frame_wall_rows.load();
}

void vision_image_processor_get_src_circle_guide_lines(uint16 **left_x,
                                                       uint16 **left_y,
                                                       uint16 *left_num,
                                                       uint16 **right_x,
                                                       uint16 **right_y,
                                                       uint16 *right_num)
{
    if (left_x) *left_x = g_src_left_circle_guide_x;
    if (left_y) *left_y = g_src_left_circle_guide_y;
    if (left_num) *left_num = g_src_left_circle_guide_count;
    if (right_x) *right_x = g_src_right_circle_guide_x;
    if (right_y) *right_y = g_src_right_circle_guide_y;
    if (right_num) *right_num = g_src_right_circle_guide_count;
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

void vision_image_processor_get_src_boundary_straight_state(bool *left_straight, bool *right_straight)
{
    if (left_straight) *left_straight = g_src_left_boundary_straight_detected.load();
    if (right_straight) *right_straight = g_src_right_boundary_straight_detected.load();
}

int vision_image_processor_route_main_state()
{
    return static_cast<int>(vision_route_state_machine_main_state());
}

int vision_image_processor_route_sub_state()
{
    return static_cast<int>(vision_route_state_machine_sub_state());
}

int vision_image_processor_route_preferred_source()
{
    return vision_route_state_machine_preferred_source();
}

uint32 vision_image_processor_route_encoder_since_state_enter()
{
    return vision_route_state_machine_encoder_since_state_enter();
}

int vision_image_processor_route_cross_loss_count()
{
    return vision_route_state_machine_cross_loss_count();
}

int vision_image_processor_route_left_loss_count()
{
    return static_cast<int>(vision_route_state_machine_snapshot().left_loss_count);
}

int vision_image_processor_route_left_gain_count()
{
    return static_cast<int>(vision_route_state_machine_snapshot().left_gain_count);
}

int vision_image_processor_route_right_loss_count()
{
    return static_cast<int>(vision_route_state_machine_snapshot().right_loss_count);
}

int vision_image_processor_route_right_gain_count()
{
    return static_cast<int>(vision_route_state_machine_snapshot().right_gain_count);
}

int vision_image_processor_zebra_cross_count()
{
    return g_zebra_cross_count.load();
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
