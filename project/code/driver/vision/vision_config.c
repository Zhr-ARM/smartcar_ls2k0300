#include "driver/vision/vision_config.h"

const vision_runtime_config_t g_vision_runtime_config = {
    // 图传输出模式：
    // 0=binary 二值图，1=gray 灰度图(带线)，2=rgb565 彩图。
    .send_mode = 2,
    // 图传发送上限帧率，0 表示不限速。
    .send_max_fps = 60,
    // 推理总开关：false 时关闭红色识别与 ncnn 推理。
    .infer_enabled = false,
    // 逐飞客户端发送开关。
    .client_sender_enabled = false,
    // 车载屏显示开关。
    .screen_display_enabled = false,
    // UDP 网页图传总开关。
    .udp_web_enabled = false,
    // UDP 网页图传发送上限帧率，0 表示不限速。
    .udp_web_max_fps = 30,
    // TCP 状态上报开关。
    .udp_web_tcp_enabled = false,
    // 电脑端接收服务 IP。
    .udp_web_server_ip = "172.21.79.179",
    // 电脑端 UDP 视频端口。
    .udp_web_video_port = 10000,
    // 电脑端 TCP 状态端口。
    .udp_web_meta_port = 10001,
    // 逐飞助手独立 UDP 通道开关。
    .assistant_udp_enabled = false,
    // 逐飞助手接收端 IP。
    .assistant_server_ip = "172.21.79.129",
    // 逐飞助手接收端端口。
    .assistant_server_port = 8899,
    // ROI 抓图模式：检测到红框后周期性保存推理 ROI。
    .roi_capture_mode = false,
    // 迷宫法左右起点搜索行，值越大越靠近图像底部。
    .maze_start_row = 90,
    // 去畸变开关：true=启用标定参数矫正，false=原图直通。
    .undistort_enabled = true,
    // IPM 处理链边界三角滤波开关。
    .ipm_triangle_filter_enabled = true,
    // IPM 处理链边界近重复点过滤开关。
    .ipm_min_point_dist_filter_enabled = true,
    // IPM 处理链边界近重复点过滤阈值，单位 px。
    .ipm_min_point_dist_px = 2.0f,
    // IPM 处理链边界等距采样开关。
    .ipm_resample_enabled = true,
    // IPM 处理链边界等距采样步长，单位 px。
    .ipm_resample_step_px = 1.0f,
    // 边界法向平移距离，单位 px，用于生成平移中线。
    .ipm_boundary_shift_distance_px = 15.0f,
    // 中线独立后处理总开关。
    .ipm_centerline_postprocess_enabled = true,
    // 中线三角滤波开关。
    .ipm_centerline_triangle_filter_enabled = true,
    // 中线等距采样开关。
    .ipm_centerline_resample_enabled = true,
    // 中线等距采样步长，单位 px。
    .ipm_centerline_resample_step_px = 6.0f,
    // 中线近重复点过滤阈值，单位 px。
    .ipm_centerline_min_point_dist_px = 2.0f,
    // line_error 使用哪条平移中线：左平移或右平移。
    .ipm_line_error_source = VISION_IPM_LINE_ERROR_FROM_LEFT_SHIFT,
    // line_error 计算方法：0=固定索引，1=加权索引，2=随速度索引。
    .ipm_line_error_method = 1,
    // 固定索引模式下默认使用的中线点索引。
    .ipm_line_error_fixed_index = 4,
    // line_error 加权点数量。
    .ipm_line_error_weighted_point_count = 3,
    // line_error 加权索引点（0-based）。
    .ipm_line_error_point_indices = {4, 8, 12},
    // line_error 各索引点对应权重。
    .ipm_line_error_weights = {0.4f, 0.4f, 0.2f},
    // 随速度索引模式公式中的速度系数 k：idx = k * speed + b。
    .ipm_line_error_speed_k = 0.02f,
    // 随速度索引模式公式中的常数项 b：idx = k * speed + b。
    .ipm_line_error_speed_b = 2.0f,
    // 随速度索引模式允许的最小索引。
    .ipm_line_error_index_min = 0,
    // 随速度索引模式允许的最大索引。
    .ipm_line_error_index_max = 30,
    // 环岛入口判定时，一侧直角点到对侧边界候选点的目标距离，单位为逆透视坐标 px。
    // 距离越大，表示越倾向于在更“外圈”的对侧边界区域寻找环岛特征。
    .roundabout_corner_match_distance_px = 45.0f,
    // 环岛入口判定的距离容差，单位 px。
    // 实际会在 [45 - 5, 45 + 5] 这样的范围内收集候选点；数值越大越宽松。
    .roundabout_corner_match_tolerance_px = 5.0f,
    // 环岛状态机里“平稳段”判断的 NMS 绝对值上限。
    // 当前逻辑已不再使用“近似共线”兜底，只有当前缀内所有点 |nms| 都小于该值才算满足条件。
    .roundabout_straight_nms_abs_max = 0.2f,
    // 环岛 running / exit 阶段检查的边界前缀长度，单位 px。
    // 从边界起始点开始沿边界累计这么长，只要这段内全部点的 |nms| 都足够小，就认为进入下一阶段。
    .roundabout_straight_span_px = 100.0f,
    // 普通状态下的边界法向平移距离，单位 px。
    // 当不处于环岛 running 状态时，偏移中线生成统一回到这个值。
    .roundabout_normal_shift_distance_px = 15.0f,
    // 环岛 running 状态下临时使用的边界法向平移距离，单位 px。
    // 进入 left_running / right_running 后会覆盖普通平移距离，退出后恢复为上面的 normal 值。
    .roundabout_running_shift_distance_px = 25.0f
};

const vision_processor_config_t g_vision_processor_config = {
    // 迷宫法单侧最大输出点数。
    .maze_trace_max_points = 120,
    // 迷宫法允许追踪的纵向区域百分比，100 表示全高。
    .maze_lower_region_percent = 100,
    // OTSU 策略：true=按需 OTSU，false=先生成整图二值图。
    .demand_otsu_enable = true,
    // 按需 OTSU 时是否保留整图二值缓存，便于调试和发送。
    .demand_otsu_keep_full_binary_cache = true,
    // 是否启用逆透视流程。
    .enable_inverse_perspective = true,
    // 逆透视输出宽度。
    .ipm_output_width = VISION_IPM_WIDTH,
    // 逆透视输出高度。
    .ipm_output_height = VISION_IPM_HEIGHT,
    // 逆透视矩阵（标定参数）。
    .change_un_mat = {
        {0.077734, -0.064964, -0.133337},
        {-0.003830, 0.006425, 2.751086},
        {-0.000113, -0.000825, 0.217794}
    },
    // 相机内参矩阵（标定参数）。
    .camera_matrix = {
        {88.352003, 0.000000, 76.039402},
        {0.000000, 88.094982, 60.149306},
        {0.000000, 0.000000, 1.000000}
    },
    // 畸变系数，顺序：[k1, k2, p1, p2, k3]。
    .dist_coeffs = {0.301435, -0.683306, -0.001952, -0.019920, 0.420358},
    // 去畸变后 x 方向平移补偿，正值向右。
    .undistort_move_x = 2,
    // 去畸变后 y 方向平移补偿，正值向下。
    .undistort_move_y = -2,
    // line_error 默认采样行比例。
    .default_line_sample_ratio = 0.55f,
    // 迷宫法默认允许搜索的最小 x，屏蔽左侧黑边。
    .default_maze_trace_x_min = 10,
    // 迷宫法默认允许搜索的最大 x，屏蔽右侧黑边。
    .default_maze_trace_x_max = 150
};
