#ifndef VISION_CONFIG_H_
#define VISION_CONFIG_H_

#include "driver/vision/vision_image_processor.h"

#include <stddef.h>

// line_error 加权点数量上限。
// 作用：限定配置数组容量，避免运行时传入过多采样点。
#define VISION_LINE_ERROR_MAX_WEIGHTED_POINTS 16
// ncnn 配置标签数量上限。
// 作用：限定默认模型标签表容量，避免运行时动态分配。
#define VISION_NCNN_CONFIG_MAX_LABELS 16

typedef enum
{
    VISION_WEB_DATA_PROFILE_FULL = 0,
    VISION_WEB_DATA_PROFILE_RAW_MINIMAL = 1
} vision_web_data_profile_enum;

typedef enum
{
    VISION_WEB_IMAGE_FORMAT_JPEG = 0,
    VISION_WEB_IMAGE_FORMAT_PNG = 1,
    VISION_WEB_IMAGE_FORMAT_BMP = 2
} vision_web_image_format_enum;

typedef struct
{
    // ==================== 参数区域 3: 网页发送 ====================
    // 包括 UDP 视频发送、TCP 状态发送及其字段开关。
    // 推理总开关：false 时关闭红色识别与 ncnn 推理链路。
    bool infer_enabled;
    // ncnn 子开关：false 时保留红框检测，但不做 ncnn 分类。
    bool ncnn_enabled;
    // ncnn 默认模型输入宽度。
    int ncnn_input_width;
    // ncnn 默认模型输入高度。
    int ncnn_input_height;
    // ncnn 默认模型标签数量。
    size_t ncnn_label_count;
    // ncnn 默认模型标签表，索引顺序必须与模型输出类别顺序一致。
    const char *ncnn_labels[VISION_NCNN_CONFIG_MAX_LABELS];
    // 红框 HSV 检测与 ncnn ROI 裁剪参数（full 图 320x240 坐标系）。
    int red_roi_h_span;
    int red_roi_s_min;
    int red_roi_v_min;
    int red_roi_close_iter;
    int red_roi_open_iter;
    int red_roi_area_min;
    float red_roi_ratio_w;
    float red_roi_ratio_h;
    float red_roi_offset_ratio;
    // 车载屏显示开关：true 时启动 screen_display_thread。
    bool screen_display_enabled;
    // ==================== 网页图传选型说明 ====================
    // 当前代码已实现的网页图片格式：JPEG / PNG / BMP（灰度 / 二值 / 彩图）。
    //
    // 若以“负载量从小到大”为标准，通常推荐顺序为：
    // 1. 灰度 JPEG：最省，但有损，可能影响本地复算。
    // 2. 彩图 JPEG：通常也较省，但有损，且网页侧还需转灰度。
    // 3. 灰度 PNG：无损压缩，适合本地复算，通常是最推荐方案。
    // 4. 彩图 PNG：无损压缩，但负载明显高于灰度 PNG。
    // 5. 灰度 BMP：无损不压缩，保真好，但带宽较高。
    // 6. 彩图 BMP：无损不压缩，带宽最高，通常不建议。
    //
    // 若以“本地复算保真度”为标准，推荐顺序为：
    // 灰度 PNG > 灰度 BMP > 彩图 PNG > 彩图 BMP > 灰度 JPEG > 彩图 JPEG。
    //
    // 当前这份配置里，可切换的是“是否发送灰度 / 二值 / 彩图”
    // 以及三路图像各自的格式（JPEG / PNG / BMP）与 FULL / RAW_MINIMAL。
    // UDP 网页图传开关：控制是否向电脑端发送视频帧。
    bool udp_web_enabled;
    // UDP 网页图传最大发送帧率，0 表示不限速。
    uint32 udp_web_max_fps;
    // UDP 网页图传是否发送灰度图。
    // 推荐：
    // 1. 本地复算优先：只开灰度图，再配 RAW_MINIMAL。
    // 2. 若担心 JPEG 失真，优先改用灰度 PNG，而不是直接上 BMP。
    bool udp_web_send_gray_jpeg;
    // UDP 网页灰度图格式：0=JPEG，1=PNG，2=BMP。
    int udp_web_gray_image_format;
    // UDP 网页图传是否发送二值图。
    // 推荐默认关闭。二值图可由网页侧根据灰度图 + otsu_threshold 现算。
    bool udp_web_send_binary_jpeg;
    // UDP 网页二值图格式：0=JPEG，1=PNG，2=BMP。
    int udp_web_binary_image_format;
    // UDP 网页图传是否发送原始 BGR 彩图。
    // 推荐：
    // 1. 展示彩图优先：只开彩图图传，再配 RAW_MINIMAL。
    // 2. 若同时开启灰度 + 彩图，会额外占用带宽，通常不建议长期并开。
    bool udp_web_send_rgb_jpeg;
    // UDP 网页彩图格式：0=JPEG，1=PNG，2=BMP。
    int udp_web_rgb_image_format;
    // 网页端 TCP 数据模式：0=全量调试数据，1=仅原始最小状态。
    // 推荐：
    // 1. FULL：用于完整调试，但状态负载明显更高。
    // 2. RAW_MINIMAL：用于“图片 + 最小状态 + 网页侧本地复算”，更适合长期使用。
    int udp_web_data_profile;
    // TCP 状态上报开关：控制是否向电脑端发送 JSON 状态数据。
    bool udp_web_tcp_enabled;
    // TCP 状态字段开关（通用）。
    bool udp_web_tcp_send_ts_ms;
    // TCP 状态字段开关（视觉相关）。
    bool udp_web_tcp_send_line_error;
    bool udp_web_tcp_send_cpu_usage_percent;
    bool udp_web_tcp_send_mem_usage_percent;
    // TCP 状态字段开关（电机相关）。
    bool udp_web_tcp_send_base_speed;
    bool udp_web_tcp_send_left_target_count;
    bool udp_web_tcp_send_right_target_count;
    bool udp_web_tcp_send_left_current_count;
    bool udp_web_tcp_send_right_current_count;
    bool udp_web_tcp_send_left_filtered_count;
    bool udp_web_tcp_send_right_filtered_count;
    bool udp_web_tcp_send_left_error;
    bool udp_web_tcp_send_right_error;
    bool udp_web_tcp_send_left_feedforward;
    bool udp_web_tcp_send_right_feedforward;
    bool udp_web_tcp_send_left_correction;
    bool udp_web_tcp_send_right_correction;
    bool udp_web_tcp_send_left_decel_assist;
    bool udp_web_tcp_send_right_decel_assist;
    bool udp_web_tcp_send_left_duty;
    bool udp_web_tcp_send_right_duty;
    bool udp_web_tcp_send_left_hardware_duty;
    bool udp_web_tcp_send_right_hardware_duty;
    bool udp_web_tcp_send_left_dir_level;
    bool udp_web_tcp_send_right_dir_level;
    // TCP 状态字段开关（视觉相关）。
    // OTSU / 性能 / 迷宫结果。
    bool udp_web_tcp_send_otsu_threshold;
    bool udp_web_tcp_send_perf_capture_wait_us;
    bool udp_web_tcp_send_perf_preprocess_us;
    bool udp_web_tcp_send_perf_otsu_us;
    bool udp_web_tcp_send_perf_maze_us;
    bool udp_web_tcp_send_perf_total_us;
    bool udp_web_tcp_send_maze_left_points_raw;
    bool udp_web_tcp_send_maze_right_points_raw;
    // 视觉检测结果。
    bool udp_web_tcp_send_red_found;
    bool udp_web_tcp_send_red_rect;
    bool udp_web_tcp_send_roi_valid;
    bool udp_web_tcp_send_roi_rect;
    // 巡线/偏差调试结果。
    bool udp_web_tcp_send_ipm_track_valid;
    bool udp_web_tcp_send_ipm_track_method;
    bool udp_web_tcp_send_ipm_centerline_source;
    bool udp_web_tcp_send_ipm_track_index;
    bool udp_web_tcp_send_ipm_track_point;
    // 边界/中线点列。
    bool udp_web_tcp_send_left_boundary;
    bool udp_web_tcp_send_right_boundary;
    bool udp_web_tcp_send_ipm_left_boundary;
    bool udp_web_tcp_send_ipm_right_boundary;
    bool udp_web_tcp_send_ipm_centerline_selected_shift;
    bool udp_web_tcp_send_src_centerline_selected_shift;
    bool udp_web_tcp_send_ipm_centerline_selected_count;
    bool udp_web_tcp_send_src_centerline_selected_count;
    bool udp_web_tcp_send_ipm_centerline_selected_curvature;
    // 尺寸元数据（视觉相关）。
    bool udp_web_tcp_send_gray_size;
    bool udp_web_tcp_send_ipm_size;
    // 电脑端接收服务 IP（运行 vision_pc_receiver.py 的主机地址）。
    const char *udp_web_server_ip;
    // 电脑端 UDP 视频端口（需与 PC 接收端 --udp-port 一致）。
    uint16 udp_web_video_port;
    // 电脑端 TCP 状态端口（需与 PC 接收端 --tcp-port 一致）。
    uint16 udp_web_meta_port;

    // ==================== 参数区域 1: 视觉处理 ====================
    // 包括迷宫法起点、去畸变、IPM 边界/中线后处理。
    // 迷宫法左右起点搜索行，固定单行搜索。
    int maze_start_row;
    // 原图巡线方法：0=迷宫法，1=八邻域法。
    int maze_trace_method;
    // 迷宫法巡线回退停止阈值：若后续 y > 当前最小 y + 阈值，则停止巡线。
    int maze_trace_y_fallback_stop_delta;
    // 十字下角点 dir 模板识别开关：用于八邻域原始 dir 序列，检测相邻平台跳变
    // 模板：{4,5}/{5,6} 前平台 -> 短过渡(1..6) -> {1,2}/{2,3} 后平台。
    bool cross_lower_corner_dir_enabled;
    // 十字下角点识别：前平台窗口长度，用于检查是否形成较长的 {4,5} 或 {5,6} 平台。
    int cross_lower_corner_pre_window;
    // 十字下角点识别：后平台窗口长度，用于检查是否形成较长的 {1,2} 或 {2,3} 平台。
    int cross_lower_corner_post_window;
    // 十字下角点识别：前平台窗口内至少需要多少个点落在 {4,5} 或 {5,6} 这两组相邻平台之一。
    int cross_lower_corner_pre_min_votes;
    // 十字下角点识别：后平台窗口内至少需要多少个点落在 {1,2} 或 {2,3} 这两组相邻平台之一。
    int cross_lower_corner_post_min_votes;
    // 十字下角点识别：前后平台之间允许的短过渡最大长度；该模板假设平台长、跳变突然。
    int cross_lower_corner_transition_max_len;
    // 十字下角点识别：左右候选角点 y 坐标最大允许差，超出则双边稳定标志无效。
    int cross_lower_corner_pair_y_diff_max;
    // 十字下角点跳变精修：规则边界搜索窗口半宽（点数）。
    int cross_lower_corner_jump_window;
    // 十字下角点跳变精修：x跳变幅度判据的倍率阈值。
    float cross_lower_corner_jump_ratio;
    // 原图直边判断：检查边界数组前多少个点的 dir。
    int src_boundary_straight_check_count;
    // 原图直边判断：前 N 个点中 dir=4/5 的最小占比阈值，范围 [0,1]。
    float src_boundary_straight_dir45_ratio_min;
    // 去畸变开关：true=开启去畸变，false=关闭去畸变直通原图。
    bool undistort_enabled;
    // ---------- 边界双处理流水线：共同预处理 ----------
    // 逆透视处理链边界三角滤波开关（作用于角点支路，单次平滑）。
    bool ipm_triangle_filter_enabled;
    // 逆透视处理链边界等距采样开关。
    bool ipm_resample_enabled;
    // 逆透视处理链边界等距采样步长（px）。
    float ipm_resample_step_px;
    // 边界顺序近点去重阈值（px）。
    float ipm_boundary_min_point_dist_px;
    // 边界回跳毛刺抑制：短段长度阈值（px）。
    float ipm_boundary_spike_short_seg_max_px;
    // 边界回跳毛刺抑制：反向判定 cos 阈值（<=阈值视为反向）。
    float ipm_boundary_spike_reverse_cos_threshold;

    // ---------- 边界双处理流水线：角点支路 ----------
    // 边界三点法夹角 cos 计算步长（索引步长，默认3）。
    int ipm_boundary_angle_step;
    // 角点候选 cos 阈值（<=阈值进入局部极小值检测）。
    float ipm_boundary_corner_cos_threshold;
    // 角点 NMS 半径（索引半径）。
    int ipm_boundary_corner_nms_radius;
    // 是否在检测到首个角点后截断其后的边界点。
    bool ipm_boundary_truncate_at_first_corner_enabled;
    // 直边检测要求的最小边界点数。
    int ipm_boundary_straight_min_points;
    // 直边检测检查窗口长度（从边界起点开始按索引检查）。
    int ipm_boundary_straight_check_count;
    // 直边检测要求的最小 cos 阈值。
    float ipm_boundary_straight_min_cos;

    // ---------- 边界双处理流水线：中线生成 ----------
    // 逆透视后赛道宽度像素（px）。
    float ipm_track_width_px;
    // 目标中线距离左边界的偏移像素（px）。
    // 例如：
    // - 取赛道宽度一半表示居中；
    // - 更小表示更靠左；
    // - 更大表示更靠右。
    float ipm_center_target_offset_from_left_px;
    // 目标板确认：连续多少帧同类且高置信才进入目标板状态。
    int infer_target_confirm_count;
    // 目标板确认：Top1 置信度阈值。
    float infer_target_confidence_threshold;
    // 目标板绕行：相对进入前中线的左右偏移量 n（px）。
    float infer_avoid_offset_delta_px;
    // 目标板退出：进入状态后连续多少帧无红框时恢复默认偏移。
    int infer_target_exit_no_red_count;
    // 逆透视处理中线独立后处理总开关（去重/平滑/重采样）。
    bool ipm_centerline_postprocess_enabled;
    // 逆透视处理中线三角滤波开关。
    bool ipm_centerline_triangle_filter_enabled;
    // 逆透视处理中线等距采样开关。
    bool ipm_centerline_resample_enabled;
    // 逆透视处理中线等距采样步长（px）。
    float ipm_centerline_resample_step_px;
    // 所选偏移中线曲率计算总开关。
    bool ipm_centerline_curvature_enabled;
    // 所选偏移中线曲率计算步长（索引步长，默认3）。
    int ipm_centerline_curvature_step;
    // 斑马线检测总开关：true=执行计数，false=关闭斑马线检测。
    bool zebra_cross_detection_enabled;
    // 双边都丢线时是否保持上一帧平移中线数组。
    bool keep_last_centerline_on_double_loss;
    // 状态机圆环识别开关：true=允许进入圆环状态，false=禁用圆环识别。
    bool route_circle_detection_enabled;
    // 圆环入口判定：对侧边界最少点数，同时角点索引需距离边界尾部至少保留该余量。
    int route_circle_entry_min_boundary_count;
    // 圆环入口判定：角点索引需小于“对侧边界点数 - 该余量”。
    int route_circle_entry_corner_tail_margin;
    // 圆环入口判定：从角点 y 往上偏移多少行后开始检查原始边界间距。
    int route_circle_entry_corner_row_offset;
    // 圆环入口判定：连续检查多少行原始左右边界间距。
    int route_circle_entry_gap_check_rows;
    // 圆环入口判定：原始左右边界在每个检查行上的最小间距阈值（px）。
    int route_circle_entry_min_raw_boundary_gap;
    // 圆环入口判定：角点 y 最小阈值。
    int route_circle_entry_corner_y_min;
    // 圆环状态 1/5 进入下一阶段时，起始贴边连续行数阈值。
    int route_circle_stage_frame_wall_rows_enter;
    // 圆环状态 3 进入下一阶段时，对侧起始贴边连续行数阈值。
    int route_circle_stage3_frame_wall_rows_trigger;
    // 圆环状态 6：搜线起始行最多抬高到该行（行号越小越靠上）。
    int route_circle_stage6_maze_start_row;
    // 圆环状态中线目标偏移（仅在 circle4/5 生效，左圆环）：相对左边界的偏移像素。
    // 右圆环在 circle4/5 自动使用镜像偏移：track_width - 该值。
    float route_circle_center_target_offset_from_left_px;
    // 圆环补线：规则/原始边界中，贴边连续段最小长度阈值。
    int circle_guide_min_frame_wall_segment_len;
    // 圆环 state3 补线：对侧规则边界目标点在贴边连续段结束后向后偏移的索引数。
    int circle_guide_target_offset_stage3;
    // 圆环 state5 补线：贴边连续段结束后，锚点向后偏移的索引数。
    int circle_guide_anchor_offset_stage5;
    // 圆环送 IPM 前硬截断触边判定边距（像素）。
    int route_circle_apply_touch_margin_px;
    // straight 判定：参与判定的最少中线点数要求（固定窗口长度）。
    int route_straight_min_centerline_points;
    // 进入 straight 状态所需的连续满足帧数。
    int route_straight_enter_consecutive_frames;
    // straight 判定：固定窗口（前 N 个点）绝对误差和上限。
    float route_straight_abs_error_sum_max;
    // 十字辅助边界：从同侧下角点沿同一 x 向上找“白->黑”转变时，最多向上扫描多少行。
    int cross_aux_vertical_scan_max_rows;
    // 十字辅助边界八邻域巡线最大点数。
    int cross_aux_trace_max_points;
    // 十字辅助边界最多向上延伸多少行。
    int cross_aux_trace_upward_rows_max;
    // 十字辅助边界重捕获：以上次辅助起点为中心向上搜索的像素数。
    int cross_aux_reacquire_up_px;
    // 十字辅助边界重捕获：以上次辅助起点为中心向下搜索的像素数。
    int cross_aux_reacquire_down_px;
    // 十字辅助边界拟合扫描：参与局部拟合的下角点前置 trace 点数。
    int cross_aux_fit_prev_points;
    // 十字辅助边界拟合扫描：左侧斜率减、右侧斜率加的外侧偏置量。
    float cross_aux_fit_slope_bias;
    // 十字上角点识别：命中 dir=5 后，后续需连续多少个点满足 dir<=5。
    int cross_upper_dir5_post_check_count;
    // 十字辅助边界历史点：历史过渡点 x 与同侧上角点 x 之间的最小偏移像素数（左: hist_x < upper_x - offset，右: hist_x > upper_x + offset）。
    int cross_aux_history_x_offset;
    // 7×7 正方形历史角点跟踪：开关。
    bool cross_upper_history_enabled;
    // 7×7 正方形半边长（3 → 边长 7）。
    int cross_upper_history_square_half;
    // 7×7 正方形边上预期黑像素数。
    int cross_upper_history_black_pixels;
    // 7×7 正方形最大平移迭代次数。
    int cross_upper_history_max_iterations;
    // 7×7 正方形每次平移像素数。
    int cross_upper_history_shift_px;

    // ==================== 参数区域 2: 偏差计算 ====================
    // 包括 line_error 取点策略与索引范围约束参数。
    // line_error 使用哪条平移中线追踪：左平移或右平移。
    int ipm_line_error_source;
    // line_error 计算方法：0=固定索引，1=前缀指数加权索引，2=随速度索引，3=兼容模式(行为同1)。
    int ipm_line_error_method;
    // 固定索引模式下使用的中线点索引。
    int ipm_line_error_fixed_index;
    // line_error 采用的加权索引点数量。
    size_t ipm_line_error_weighted_point_count;
    // line_error 加权索引点（0-based）。
    int ipm_line_error_point_indices[VISION_LINE_ERROR_MAX_WEIGHTED_POINTS];
    // line_error 各索引点对应权重。
    float ipm_line_error_weights[VISION_LINE_ERROR_MAX_WEIGHTED_POINTS];
    // 随速度索引模式公式中的速度系数 k：idx = k * speed + b（仅 method=2 生效）。
    float ipm_line_error_speed_k;
    // 随速度索引模式公式中的常数项 b：idx = k * speed + b（仅 method=2 生效）。
    float ipm_line_error_speed_b;
    // 随速度索引模式允许的最小索引（仅 method=2 生效）。
    int ipm_line_error_index_min;
    // 随速度索引模式允许的最大索引（仅 method=2 生效）。
    int ipm_line_error_index_max;
    // 比例前缀加权模式：参与 line_error 计算的中线前缀比例（0~1]。
    float ipm_line_error_prefix_ratio;
    // 比例前缀加权模式：指数权重参数 lambda（w(i)=exp(-lambda*t), t=i/(n-1)）。
    float ipm_line_error_exp_lambda;
} vision_runtime_config_t;

typedef struct
{
    // 迷宫法单侧巡线最大输出点数。
    int maze_trace_max_points;
    // 迷宫法允许追踪的纵向区域百分比（100=全高）。
    int maze_lower_region_percent;
    // 二值化模式：0=全图OTSU, 1=按需局部均值自适应, 2=全图局部均值自适应。
    int binarization_mode;
    // 自适应窗口大小（仅 mode=1），建议奇数 5~9。
    int adaptive_window_size;
    // 自适应阈值常数（仅 mode=1），threshold = mean - constant。
    int adaptive_constant;
    // 是否启用“边线点逆透视 + IPM 边界输出”流程。
    bool enable_inverse_perspective;
    // 逆透视输出宽度。
    int ipm_output_width;
    // 逆透视输出高度。
    int ipm_output_height;
    // 逆透视矩阵（用户标定参数）。
    double change_un_mat[3][3];
    // 相机内参矩阵（用户标定参数）。
    double camera_matrix[3][3];
    // 畸变系数，顺序：[k1, k2, p1, p2, k3]。
    double dist_coeffs[5];
    // 去畸变后平移补偿 x（像素）：正值右移。
    int undistort_move_x;
    // 去畸变后平移补偿 y（像素）：正值下移。
    int undistort_move_y;
    // line_error 默认采样行比例。
    float default_line_sample_ratio;
    // 原图迷宫法巡线默认最小 x。
    int default_maze_trace_x_min;
    // 原图迷宫法巡线默认最大 x。
    int default_maze_trace_x_max;
    // 原图裁剪上边界（与 crop_denominator 配合换算为比例）。
    int crop_top;
    // 原图裁剪下边界（与 crop_denominator 配合换算为比例）。
    int crop_bottom;
    // 原图裁剪分母，默认 120。
    int crop_denominator;
} vision_processor_config_t;

// 视觉运行时配置：
// main.cpp 在启动时统一从这里读取并下发到各视觉模块。
extern vision_runtime_config_t g_vision_runtime_config;

// 视觉处理器内部配置：
// vision_image_processor.cpp 在初始化和算法处理中统一从这里读取。
extern vision_processor_config_t g_vision_processor_config;

#endif
