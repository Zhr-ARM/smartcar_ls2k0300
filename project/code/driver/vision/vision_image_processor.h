#ifndef VISION_IMAGE_PROCESSOR_H_
#define VISION_IMAGE_PROCESSOR_H_

#include "zf_common_headfile.h"
#include <cstddef>

#define VISION_DOWNSAMPLED_WIDTH 160
#define VISION_DOWNSAMPLED_HEIGHT 120
#define VISION_IPM_WIDTH 280
#define VISION_IPM_HEIGHT 140

// 轮廓线误差：在图像高度 line_sample_ratio 处，
// 中线 x 与图像中心 x 的差值（右偏为正，左偏为负）。
// 是否调用：line_follow_thread 直接使用该全局量做控制输入。
extern int line_error;
// line_error 采样行比例（0.0~1.0）：
// sample_y = clamp(int(VISION_DOWNSAMPLED_HEIGHT * line_sample_ratio), 0, VISION_DOWNSAMPLED_HEIGHT - 1)。
// 图像坐标系中 y 向下增大：0.0 靠近顶部，1.0 靠近底部。
// 默认 0.55f（约在 y=66/120 位置取样）。
// 如何修改：提高比例值会让采样行下移（更靠近车前近处）；降低则上移（更看远处）。
extern float line_sample_ratio;

typedef enum
{
    VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT = 0,
    VISION_IPM_LINE_ERROR_FROM_RIGHT_SHIFT = 1,
    VISION_IPM_LINE_ERROR_FROM_AUTO = 2
} vision_ipm_line_error_source_enum;

typedef enum
{
    VISION_IPM_LINE_ERROR_FIXED_INDEX = 0,
    VISION_IPM_LINE_ERROR_WEIGHTED_INDEX = 1,
    VISION_IPM_LINE_ERROR_SPEED_INDEX = 2,
    // 兼容保留值：当前行为与 VISION_IPM_LINE_ERROR_WEIGHTED_INDEX 相同。
    VISION_IPM_LINE_ERROR_WEIGHTED_SPEED_DELTA = 3
} vision_ipm_line_error_method_enum;

typedef enum
{
    VISION_TRACE_METHOD_MAZE = 0,
    VISION_TRACE_METHOD_EIGHT_NEIGHBOR = 1
} vision_trace_method_enum;

// 作用：初始化图像处理模块（内部通过 vision_frame_capture 初始化采图）。
// 参数：camera_path 为空时使用默认相机路径。
// 是否调用：是，vision_pipeline_init 调用。
bool vision_image_processor_init(const char *camera_path);
// 作用：清理图像处理模块资源（停止采图线程）。
// 是否调用：是，vision_pipeline_cleanup 调用。
void vision_image_processor_cleanup();

// 作用：执行一帧“采图+灰度/二值+迷宫法边线+逆透视+中线误差”。
// 返回：true=处理成功，false=当前周期未取到新帧。
// 是否调用：是，vision_pipeline_process_step 每帧调用。
bool vision_image_processor_process_step();
// 作用：读取“最近一次成功完成处理”的视觉帧序号。
// 约定：仅当 process_step 成功处理完一帧时递增，供控制层判断是否拿到了新视觉样本。
uint32 vision_image_processor_processed_frame_seq();
// 从 g_vision_runtime_config / g_vision_processor_config 重新同步内部缓存。
// 说明：用于启动时 TOML 加载后刷新全局静态默认值，避免某些原子量仍保留编译期初值。
void vision_image_processor_reload_config_from_globals();

// 作用：设置/获取迷宫法左右起点搜索行（单行搜索）。
// 如何修改：建议范围 [1, H-2]；值越大越靠近图像底部。
// 是否调用：是，main.cpp 启动时配置。
void vision_image_processor_set_maze_start_row(int row);
int vision_image_processor_get_maze_start_row();
// 原图边界追踪方法：
// - VISION_TRACE_METHOD_MAZE: 当前默认的左手/右手迷宫法；
// - VISION_TRACE_METHOD_EIGHT_NEIGHBOR: 基于 8 邻域的边界跟踪。
void vision_image_processor_set_trace_method(vision_trace_method_enum method);
vision_trace_method_enum vision_image_processor_trace_method();
// 原图迷宫法巡线水平区间限制（包含边界）。
// 约定：巡线起点搜索和左右手追踪都只能在 [x_min, x_max] 内进行；
// 一旦下一步触碰到区间边界，就立即停止巡线，避免去畸变黑边干扰。
// 默认值：10 ~ 150（以 160 宽图像为基准）。
void vision_image_processor_set_maze_trace_x_range(int x_min, int x_max);
void vision_image_processor_get_maze_trace_x_range(int *x_min, int *x_max);

// 去畸变开关（运行时可配）。
// true : 开启去畸变（使用 cameraMatrix/distCoeffs/move_xy）；
// false: 跳过去畸变，直接使用原始采图。
// 是否调用：建议在 main.cpp 启动配置区设置。
void vision_image_processor_set_undistort_enabled(bool enabled);
bool vision_image_processor_undistort_enabled();

// 逆透视后边界三角滤波开关（对 IPM 边界点做原地平滑）。
// true : 开启三角滤波（1-2-1）；
// false: 关闭三角滤波（直接使用逆透视原始边界点）。
// 说明：关闭仅跳过该平滑步骤，不影响后续处理链。
void vision_image_processor_set_ipm_triangle_filter_enabled(bool enabled);
bool vision_image_processor_ipm_triangle_filter_enabled();
// 逆透视后边界等距采样开关（作用于处理链边界）。
// true : 开启按弧长等距重采样；
// false: 关闭，保留当前点序列。
void vision_image_processor_set_ipm_resample_enabled(bool enabled);
bool vision_image_processor_ipm_resample_enabled();
// 逆透视后边界等距采样步长（单位：像素）。
// 建议 >= 1.0；数值越小点更密、越大点更稀。
void vision_image_processor_set_ipm_resample_step_px(float step_px);
float vision_image_processor_ipm_resample_step_px();
// 边界三点法夹角 cos 计算步长（索引步长，>=1）。
void vision_image_processor_set_ipm_boundary_angle_step(int step);
int vision_image_processor_ipm_boundary_angle_step();
void vision_image_processor_set_ipm_boundary_straight_min_points(int points);
int vision_image_processor_ipm_boundary_straight_min_points();
void vision_image_processor_set_ipm_boundary_straight_check_count(int count);
int vision_image_processor_ipm_boundary_straight_check_count();
void vision_image_processor_set_ipm_boundary_straight_min_cos(float min_cos);
float vision_image_processor_ipm_boundary_straight_min_cos();

// 逆透视后赛道宽度像素（单位：px）。
void vision_image_processor_set_ipm_track_width_px(float width_px);
float vision_image_processor_ipm_track_width_px();
// 目标中线距离左边界的偏移像素（单位：px）。
// 约定：
// - 左边界生成中线时，向右平移该值；
// - 右边界生成中线时，向左平移 (赛道宽度 - 该值)。
void vision_image_processor_set_ipm_center_target_offset_from_left_px(float offset_px);
float vision_image_processor_ipm_center_target_offset_from_left_px();

// 逆透视处理中线后处理总开关（去重/平滑/重采样）。
void vision_image_processor_set_ipm_centerline_postprocess_enabled(bool enabled);
bool vision_image_processor_ipm_centerline_postprocess_enabled();
// 逆透视处理中线三角滤波开关。
void vision_image_processor_set_ipm_centerline_triangle_filter_enabled(bool enabled);
bool vision_image_processor_ipm_centerline_triangle_filter_enabled();
// 逆透视处理中线等距采样开关与步长（px）。
void vision_image_processor_set_ipm_centerline_resample_enabled(bool enabled);
bool vision_image_processor_ipm_centerline_resample_enabled();
void vision_image_processor_set_ipm_centerline_resample_step_px(float step_px);
float vision_image_processor_ipm_centerline_resample_step_px();
void vision_image_processor_set_ipm_centerline_curvature_enabled(bool enabled);
bool vision_image_processor_ipm_centerline_curvature_enabled();
// line_error 跟踪点配置：
// - source: 偏好左/偏好右/无偏好自动（按边界点数择优）；
// - method: 固定索引 / 前缀线性加权索引 / 随速度索引 / 兼容模式(行为同前缀线性加权)；
// - fixed_index: 固定索引模式下使用的点索引；
// - point_indices/weights: 加权索引模式下使用的索引与权重；
// - speed_k/speed_b: 随速度索引模式公式 idx = k * speed + b（仅 method=2）；
// - index_min/max: 随速度索引模式的索引限制区间（仅 method=2）。
// 若加权索引中的某索引超出当前中线长度，则该索引权重会按剩余有效权重比例重分配。
void vision_image_processor_set_ipm_line_error_source(vision_ipm_line_error_source_enum source);
vision_ipm_line_error_source_enum vision_image_processor_ipm_line_error_source();
void vision_image_processor_set_ipm_line_error_method(vision_ipm_line_error_method_enum method);
vision_ipm_line_error_method_enum vision_image_processor_ipm_line_error_method();
void vision_image_processor_set_ipm_line_error_fixed_index(int point_index);
int vision_image_processor_ipm_line_error_fixed_index();
void vision_image_processor_set_ipm_line_error_weighted_points(const int *point_indices,
                                                              const float *weights,
                                                              size_t count);
size_t vision_image_processor_ipm_line_error_weighted_point_count();
void vision_image_processor_set_ipm_line_error_speed_formula(float speed_k, float speed_b);
void vision_image_processor_get_ipm_line_error_speed_formula(float *speed_k, float *speed_b);
void vision_image_processor_set_ipm_line_error_index_range(int index_min, int index_max);
void vision_image_processor_get_ipm_line_error_index_range(int *index_min, int *index_max);
int vision_image_processor_ipm_line_error_track_index();
void vision_image_processor_get_ipm_line_error_track_point(bool *valid, int *x, int *y);
void vision_image_processor_set_ipm_centerline_curvature_step(int step);
int vision_image_processor_ipm_centerline_curvature_step();
int vision_image_processor_ipm_selected_centerline_count();
int vision_image_processor_ipm_straight_required_last_index();
float vision_image_processor_ipm_straight_abs_error_sum();
float vision_image_processor_ipm_mean_abs_offset_error();
float vision_image_processor_ipm_front_weighted_abs_error_sum(int point_count);
// 按中线分段误差混合：
// 1) 先按 split_ratio 把中线分成前段 [0, split_ratio) 与后段 [split_ratio, 1]；
// 2) 分别计算两段绝对误差均值；
// 3) 按 front_weight / rear_weight 做加权融合。
float vision_image_processor_ipm_segmented_blended_abs_error(float split_ratio,
                                                             float front_weight,
                                                             float rear_weight);

// 读取最近一帧处理耗时（单位：us）
// capture_wait_us: 等待相机新帧
// preprocess_us  : 当前分辨率 BGR/Gray 生成
// otsu_us        : 全图OTSU二值化耗时
// maze_us        : 下60%区域迷宫法双边线提取耗时
// total_us       : process_step总耗时
void vision_image_processor_get_last_perf_us(uint32 *capture_wait_us,
                                             uint32 *preprocess_us,
                                             uint32 *otsu_us,
                                             uint32 *maze_us,
                                             uint32 *total_us);
uint8 vision_image_processor_get_last_otsu_threshold();
// 读取最近一帧红色矩形识别耗时（单位：us）。
// 是否调用：是，vision_thread 性能统计和 TCP 状态上报调用。
void vision_image_processor_get_last_red_detect_us(uint32 *red_detect_us);

// 读取最近一帧 maze 阶段子流程耗时（单位：us）与有效性统计
// maze_setup_us       : maze阶段初始化（参数准备）
// maze_start_us       : 左右起点搜索（find_maze_start_from_bottom）
// maze_trace_left_us  : 左边线巡线耗时
// maze_trace_right_us : 右边线巡线耗时
// maze_post_us        : 后处理（边线填充/逆透视与中线生成）
// left_points/right_points : 左右边线输出点数
// left_ok/right_ok         : 左右起点是否找到
void vision_image_processor_get_last_maze_detail_us(uint32 *maze_setup_us,
                                                    uint32 *maze_start_us,
                                                    uint32 *maze_trace_left_us,
                                                    uint32 *maze_trace_right_us,
                                                    uint32 *maze_post_us,
                                                    uint16 *left_points,
                                                    uint16 *right_points,
                                                    bool *left_ok,
                                                    bool *right_ok);

// 图像数据访问接口。
// 当前阶段处理图像统一为 VISION_DOWNSAMPLED_WIDTH x VISION_DOWNSAMPLED_HEIGHT。
// 是否调用：被 transport/pipeline/web 状态发送等路径复用。
const uint8 *vision_image_processor_gray_image();
const uint8 *vision_image_processor_binary_u8_image();
const uint8 *vision_image_processor_bgr_image();
const uint8 *vision_image_processor_bgr_full_image();
// 降采样接口与主接口一致（当前输入即处理分辨率）
const uint8 *vision_image_processor_gray_downsampled_image();
const uint8 *vision_image_processor_binary_downsampled_u8_image();
const uint8 *vision_image_processor_bgr_downsampled_image();

// 边界数据访问接口。
// 当前返回原图(无逆透视)坐标系下的边界数组:
// x1/y1 左边线, x2/y2 左右均值中线, x3/y3 右边线。
// 参数均为“输出参数指针”，可传 nullptr 跳过不关心字段。
// 是否调用：client sender、TCP 状态上报、控制链路。
void vision_image_processor_get_boundaries(uint16 **x1, uint16 **x2, uint16 **x3,
                                           uint16 **y1, uint16 **y2, uint16 **y3,
                                           uint16 *dot_num);
void vision_image_processor_get_boundary_side_counts(uint16 *left_dot_num, uint16 *right_dot_num);
// 八邻域原始轮廓点与生长方向，dir 与同索引点对齐：
// - dir=0..7 对应 seeds_l/seeds_r 的循环索引；
// - dir=255 表示该点没有有效后继方向（通常是最后一点或非八邻域模式）。
void vision_image_processor_get_eight_neighbor_left_trace(uint16 **x, uint16 **y, uint8 **dir, uint16 *dot_num, int *first_frame_touch_index);
void vision_image_processor_get_eight_neighbor_right_trace(uint16 **x, uint16 **y, uint8 **dir, uint16 *dot_num, int *first_frame_touch_index);
// 八邻域 dir 软模板识别到的十字下角点：
// 模板为 {4,5} 前平台 -> 短过渡(允许少量 3) -> 2 后平台；
// 输出角点偏向前平台末端，优先取最后一个 dir=4。
void vision_image_processor_get_cross_lower_corner_state(bool *left_found,
                                                         int *left_index,
                                                         int *left_x,
                                                         int *left_y,
                                                         bool *right_found,
                                                         int *right_index,
                                                         int *right_x,
                                                         int *right_y,
                                                         bool *pair_valid);
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
                                                     uint16 *regular_num);
void vision_image_processor_get_cross_upper_corner_state(bool *left_found,
                                                         int *left_index,
                                                         int *left_x,
                                                         int *left_y,
                                                         bool *right_found,
                                                         int *right_index,
                                                         int *right_x,
                                                         int *right_y);
void vision_image_processor_get_cross_stage2_frozen_lower_corner_state(bool *left_found,
                                                                       int *left_x,
                                                                       int *left_y,
                                                                       bool *right_found,
                                                                       int *right_x,
                                                                       int *right_y);
void vision_image_processor_get_cross_route_debug_state(int *left_corner_post_frame_wall_rows,
                                                        int *right_corner_post_frame_wall_rows,
                                                        int *start_boundary_gap_x);
void vision_image_processor_get_src_trace_frame_wall_state(bool *left_has_frame_wall,
                                                           bool *right_has_frame_wall);
void vision_image_processor_get_src_start_frame_wall_rows(int *left_rows, int *right_rows);
void vision_image_processor_get_src_circle_guide_lines(uint16 **left_x,
                                                       uint16 **left_y,
                                                       uint16 *left_num,
                                                       uint16 **right_x,
                                                       uint16 **right_y,
                                                       uint16 *right_num);
// 逆透视后边界数据（另存，供控制等后续模块使用）
void vision_image_processor_get_ipm_boundaries(uint16 **x1, uint16 **x2, uint16 **x3,
                                               uint16 **y1, uint16 **y2, uint16 **y3,
                                               uint16 *dot_num);
void vision_image_processor_get_ipm_boundary_side_counts(uint16 *left_dot_num, uint16 *right_dot_num);
void vision_image_processor_get_src_boundary_straight_state(bool *left_straight, bool *right_straight);
int vision_image_processor_route_main_state();
int vision_image_processor_route_sub_state();
int vision_image_processor_route_preferred_source();
uint32 vision_image_processor_route_encoder_since_state_enter();
int vision_image_processor_route_cross_loss_count();
int vision_image_processor_route_left_loss_count();
int vision_image_processor_route_left_gain_count();
int vision_image_processor_route_right_loss_count();
int vision_image_processor_route_right_gain_count();
int vision_image_processor_zebra_cross_count();
// 逆透视处理链“平移中线”结果：
// - from_left : 左边界向右法向平移得到；
// - from_right: 右边界向左法向平移得到。
void vision_image_processor_get_ipm_shifted_centerline_from_left(uint16 **x, uint16 **y, uint16 *dot_num);
void vision_image_processor_get_ipm_shifted_centerline_from_right(uint16 **x, uint16 **y, uint16 *dot_num);
// 将左右偏移中线从 IPM 坐标回投到原图后的结果。
void vision_image_processor_get_src_shifted_centerline_from_left(uint16 **x, uint16 **y, uint16 *dot_num);
void vision_image_processor_get_src_shifted_centerline_from_right(uint16 **x, uint16 **y, uint16 *dot_num);

// 红色实心矩形检测结果（坐标与尺寸）。
// set_* 接口由 infer 模块写入，get_* 接口供发送/UI读取。
void vision_image_processor_get_red_rect(bool *found, int *x, int *y, int *w, int *h, int *cx, int *cy);
int vision_image_processor_get_red_rect_area();
void vision_image_processor_set_red_rect(bool found, int x, int y, int w, int h, int cx, int cy, int area);
void vision_image_processor_set_last_red_detect_us(uint32 red_detect_us);

// ncnn 输入 ROI（用于发送图像叠加调试框）。
// set_* 由 infer 模块写入，get_* 由 transport 状态发送读取。
void vision_image_processor_set_ncnn_roi(bool valid, int x, int y, int w, int h);
void vision_image_processor_get_ncnn_roi(bool *valid, int *x, int *y, int *w, int *h);

#endif
