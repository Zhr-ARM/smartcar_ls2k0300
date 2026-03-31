#ifndef VISION_CONFIG_H_
#define VISION_CONFIG_H_

#include "driver/vision/vision_image_processor.h"

#include <stddef.h>

// line_error 加权点数量上限。
// 作用：限定配置数组容量，避免运行时传入过多采样点。
#define VISION_LINE_ERROR_MAX_WEIGHTED_POINTS 16

typedef struct
{
    // 图传输出模式：
    // VISION_THREAD_SEND_BINARY / VISION_THREAD_SEND_GRAY / VISION_THREAD_SEND_RGB565。
    int send_mode;
    // 图传发送上限帧率，0 表示不限速。
    uint32 send_max_fps;
    // 推理总开关：false 时关闭红色识别与 ncnn 推理链路。
    bool infer_enabled;
    // 客户端发送开关：控制逐飞助手发送链路是否启用。
    bool client_sender_enabled;
    // 车载屏显示开关：true 时启动 screen_display_thread。
    bool screen_display_enabled;
    // UDP 网页图传开关：控制是否向电脑端发送视频帧。
    bool udp_web_enabled;
    // UDP 网页图传最大发送帧率，0 表示不限速。
    uint32 udp_web_max_fps;
    // TCP 状态上报开关：控制是否向电脑端发送 JSON 状态数据。
    bool udp_web_tcp_enabled;
    // 电脑端接收服务 IP（运行 vision_pc_receiver.py 的主机地址）。
    const char *udp_web_server_ip;
    // 电脑端 UDP 视频端口（需与 PC 接收端 --udp-port 一致）。
    uint16 udp_web_video_port;
    // 电脑端 TCP 状态端口（需与 PC 接收端 --tcp-port 一致）。
    uint16 udp_web_meta_port;
    // 逐飞客户端 UDP 通道开关（与网页端 UDP/TCP 独立）。
    bool assistant_udp_enabled;
    // 逐飞客户端接收端 IP。
    const char *assistant_server_ip;
    // 逐飞客户端接收端端口。
    uint16 assistant_server_port;
    // 采图模式：检测到红色矩形后，每 1s 保存一次推理 ROI 彩图。
    bool roi_capture_mode;
    // 迷宫法左右起点搜索行，固定单行搜索。
    int maze_start_row;
    // 去畸变开关：true=开启去畸变，false=关闭去畸变直通原图。
    bool undistort_enabled;
    // 逆透视处理链边界三角滤波开关。
    bool ipm_triangle_filter_enabled;
    // 逆透视处理链边界近重复点过滤开关。
    bool ipm_min_point_dist_filter_enabled;
    // 逆透视处理链边界近重复点过滤阈值（px）。
    float ipm_min_point_dist_px;
    // 逆透视处理链边界等距采样开关。
    bool ipm_resample_enabled;
    // 逆透视处理链边界等距采样步长（px）。
    float ipm_resample_step_px;
    // 逆透视处理链边界法向平移距离（px），用于生成平移中线。
    float ipm_boundary_shift_distance_px;
    // 逆透视处理中线独立后处理总开关（去重/平滑/重采样）。
    bool ipm_centerline_postprocess_enabled;
    // 逆透视处理中线三角滤波开关。
    bool ipm_centerline_triangle_filter_enabled;
    // 逆透视处理中线等距采样开关。
    bool ipm_centerline_resample_enabled;
    // 逆透视处理中线等距采样步长（px）。
    float ipm_centerline_resample_step_px;
    // 逆透视处理中线近重复点过滤阈值（px）。
    float ipm_centerline_min_point_dist_px;
    // line_error 使用哪条平移中线追踪：左平移或右平移。
    int ipm_line_error_source;
    // line_error 计算方法：0=固定索引，1=加权索引，2=随速度索引。
    int ipm_line_error_method;
    // 固定索引模式下使用的中线点索引。
    int ipm_line_error_fixed_index;
    // line_error 采用的加权索引点数量。
    size_t ipm_line_error_weighted_point_count;
    // line_error 加权索引点（0-based）。
    int ipm_line_error_point_indices[VISION_LINE_ERROR_MAX_WEIGHTED_POINTS];
    // line_error 各索引点对应权重。
    float ipm_line_error_weights[VISION_LINE_ERROR_MAX_WEIGHTED_POINTS];
    // 随速度索引模式公式中的速度系数 k：idx = k * speed + b。
    float ipm_line_error_speed_k;
    // 随速度索引模式公式中的常数项 b：idx = k * speed + b。
    float ipm_line_error_speed_b;
    // 随速度索引模式允许的最小索引。
    int ipm_line_error_index_min;
    // 随速度索引模式允许的最大索引。
    int ipm_line_error_index_max;
    // 环岛入口判定时，“角点到对侧边界点”的目标距离（IPM 像素）。
    // 作用：在检测到一侧直角后，只在对侧边界上寻找与该角点距离接近本值的点集做环岛候选判断。
    float roundabout_corner_match_distance_px;
    // 环岛入口判定的距离容差（IPM 像素）。
    // 作用：允许实际距离在 [distance - tolerance, distance + tolerance] 内浮动，增强对采样抖动的鲁棒性。
    float roundabout_corner_match_tolerance_px;
    // 环岛相关“直线/平稳段”判定的 NMS 绝对值阈值。
    // 作用：当前代码中仅使用“低 NMS”判断，若一段边界前缀内所有点的 |nms| 都小于该值，则认为该段足够平稳。
    float roundabout_straight_nms_abs_max;
    // 环岛入环/出环阶段前缀检查长度（沿边界累计长度，单位 px）。
    // 作用：在 running/exit 等状态中，只检查从边界起始点开始前这么长的一段边界是否持续低 NMS。
    float roundabout_straight_span_px;
    // 非环岛 running 状态下的边界法向平移距离（px）。
    // 作用：环岛状态机未进入 running 时，IPM 偏移中线生成使用该平移值。
    float roundabout_normal_shift_distance_px;
    // 环岛 running 状态下的边界法向平移距离（px）。
    // 作用：当 roundabout_mode 为 left_running/right_running 时，临时改用该平移值生成偏移中线。
    float roundabout_running_shift_distance_px;
} vision_runtime_config_t;

typedef struct
{
    // 迷宫法单侧巡线最大输出点数。
    int maze_trace_max_points;
    // 迷宫法允许追踪的纵向区域百分比（100=全高）。
    int maze_lower_region_percent;
    // OTSU 二值化策略：true=按需 OTSU，false=先生成全图二值图。
    bool demand_otsu_enable;
    // 按需 OTSU 时是否保留完整二值图缓存。
    bool demand_otsu_keep_full_binary_cache;
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
} vision_processor_config_t;

// 视觉运行时配置：
// main.cpp 在启动时统一从这里读取并下发到各视觉模块。
extern const vision_runtime_config_t g_vision_runtime_config;

// 视觉处理器内部配置：
// vision_image_processor.cpp 在初始化和算法处理中统一从这里读取。
extern const vision_processor_config_t g_vision_processor_config;

#endif
