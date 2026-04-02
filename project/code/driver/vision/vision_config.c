#include "driver/vision/vision_config.h"

const vision_runtime_config_t g_vision_runtime_config = {
    // ==================== 本地显示与逐飞助手链路 ====================
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

    // ==================== 网页端 UDP 视频发送配置 ====================
    // UDP 网页图传总开关。
    .udp_web_enabled = true,
    // UDP 网页图传发送上限帧率，0 表示不限速。
    .udp_web_max_fps = 30,
    // 是否向网页端发送灰度 JPEG。
    // 当前网页端可用该灰度图结合 TCP 下发的 otsu_threshold 自行实时二值化。
    .udp_web_send_gray_jpeg = true,
    // 是否向网页端发送二值 JPEG。
    // 当前需求为停发二值图，因此默认关闭。
    .udp_web_send_binary_jpeg = false,

    // ==================== 网页端 TCP 状态发送总开关 ====================
    // TCP 状态上报开关。
    .udp_web_tcp_enabled = true,

    // ==================== TCP 基础状态字段 ====================
    // 发送主板当前时间戳（ms）。
    .udp_web_tcp_send_ts_ms = false,
    // 发送当前巡线误差 line_error。
    .udp_web_tcp_send_line_error = true,
    // 发送当前基础巡航速度 base_speed。
    .udp_web_tcp_send_base_speed = true,
    // 发送左轮实时目标速度 left_target_count。
    .udp_web_tcp_send_left_target_count = true,
    // 发送右轮实时目标速度 right_target_count。
    .udp_web_tcp_send_right_target_count = true,
    // 发送左轮编码器实时速度 left_current_count。
    .udp_web_tcp_send_left_current_count = true,
    // 发送右轮编码器实时速度 right_current_count。
    .udp_web_tcp_send_right_current_count = true,
    // 发送左轮滤波后的速度反馈 left_filtered_count。
    .udp_web_tcp_send_left_filtered_count = false,
    // 发送右轮滤波后的速度反馈 right_filtered_count。
    .udp_web_tcp_send_right_filtered_count = false,
    // 发送左轮速度误差 left_error。
    .udp_web_tcp_send_left_error = false,
    // 发送右轮速度误差 right_error。
    .udp_web_tcp_send_right_error = false,
    // 发送左轮前馈输出 left_feedforward。
    .udp_web_tcp_send_left_feedforward = false,
    // 发送右轮前馈输出 right_feedforward。
    .udp_web_tcp_send_right_feedforward = false,
    // 发送左轮 PID 修正项 left_correction。
    .udp_web_tcp_send_left_correction = false,
    // 发送右轮 PID 修正项 right_correction。
    .udp_web_tcp_send_right_correction = false,
    // 发送左轮减速辅助项 left_decel_assist。
    .udp_web_tcp_send_left_decel_assist = false,
    // 发送右轮减速辅助项 right_decel_assist。
    .udp_web_tcp_send_right_decel_assist = false,
    // 发送左轮当前实际输出 duty。
    .udp_web_tcp_send_left_duty = false,
    // 发送右轮当前实际输出 duty。
    .udp_web_tcp_send_right_duty = false,
    // 发送左轮最终硬件 duty（已包含方向翻转语义）。
    .udp_web_tcp_send_left_hardware_duty = false,
    // 发送右轮最终硬件 duty（已包含方向翻转语义）。
    .udp_web_tcp_send_right_hardware_duty = false,
    // 发送左轮方向 GPIO 电平。
    .udp_web_tcp_send_left_dir_level = false,
    // 发送右轮方向 GPIO 电平。
    .udp_web_tcp_send_right_dir_level = false,
    // 发送本帧计算得到的 OTSU 阈值，供网页端灰度图实时二值化。
    .udp_web_tcp_send_otsu_threshold = true,

    // ==================== TCP 性能统计字段 ====================
    // 发送等待新图像帧耗时（us）。
    .udp_web_tcp_send_perf_capture_wait_us = false,
    // 发送预处理耗时（us），包括去畸变、灰度图生成等。
    .udp_web_tcp_send_perf_preprocess_us = false,
    // 发送 OTSU 阶段耗时（us）。
    .udp_web_tcp_send_perf_otsu_us = false,
    // 发送迷宫法巡线阶段耗时（us）。
    .udp_web_tcp_send_perf_maze_us = false,
    // 发送整帧视觉处理总耗时（us）。
    .udp_web_tcp_send_perf_total_us = true,
    // 发送迷宫法真实左边界点数（未做网页显示数组补齐）。
    .udp_web_tcp_send_maze_left_points_raw = true,
    // 发送迷宫法真实右边界点数（未做网页显示数组补齐）。
    .udp_web_tcp_send_maze_right_points_raw = true,

    // ==================== TCP 检测结果字段 ====================
    // 发送是否检测到红色目标。
    .udp_web_tcp_send_red_found = false,
    // 发送红框位置与尺寸 [x, y, w, h, cx, cy]。
    .udp_web_tcp_send_red_rect = false,
    // 发送 ncnn ROI 是否有效。
    .udp_web_tcp_send_roi_valid = false,
    // 发送 ROI 区域 [x, y, w, h]。
    .udp_web_tcp_send_roi_rect = false,

    // ==================== TCP 巡线跟踪点字段 ====================
    // 发送当前 IPM 跟踪点是否有效。
    .udp_web_tcp_send_ipm_track_valid = false,
    // 发送 line_error 的取点方式：固定索引 / 加权索引 / 随速度索引。
    .udp_web_tcp_send_ipm_track_method = false,
    // 发送当前采用的是左偏移中线还是右偏移中线。
    .udp_web_tcp_send_ipm_centerline_source = true,
    // 发送当前实际命中的中线点索引。
    .udp_web_tcp_send_ipm_track_index = true,
    // 发送当前实际跟踪点坐标 [x, y]。
    .udp_web_tcp_send_ipm_track_point = true,
    // 发送加权模式的首点索引参数。
    .udp_web_tcp_send_ipm_weighted_first_index = false,
    // 发送加权模式的决策点索引参数。
    .udp_web_tcp_send_ipm_weighted_decision_index = false,
    // 发送加权模式的默认点间距参数。
    .udp_web_tcp_send_ipm_weighted_default_spacing = false,
    // 发送加权模式阈值 1。
    .udp_web_tcp_send_ipm_weighted_spacing_threshold_1 = false,
    // 发送加权模式阈值 2。
    .udp_web_tcp_send_ipm_weighted_spacing_threshold_2 = false,
    // 发送加权模式阈值 3。
    .udp_web_tcp_send_ipm_weighted_spacing_threshold_3 = false,
    // 发送阈值 1 触发后的点间距。
    .udp_web_tcp_send_ipm_weighted_spacing_value_1 = false,
    // 发送阈值 2 触发后的点间距。
    .udp_web_tcp_send_ipm_weighted_spacing_value_2 = false,
    // 发送阈值 3 触发后的点间距。
    .udp_web_tcp_send_ipm_weighted_spacing_value_3 = false,
    // 发送首点的当前偏差。
    .udp_web_tcp_send_ipm_weighted_first_point_error = true,
    // 发送本帧实际使用的加权点间距。
    .udp_web_tcp_send_ipm_weighted_current_spacing = true,
    // 发送逆透视图上的决策点坐标。
    .udp_web_tcp_send_ipm_weighted_decision_point = true,
    // 发送原图上的决策点坐标。
    .udp_web_tcp_send_src_weighted_decision_point = true,

    // ==================== TCP 元素状态机字段 ====================
    // 发送十字模式是否开启。
    .udp_web_tcp_send_intersection_mode = false,
    // 发送十字模式探测到的停止行。
    .udp_web_tcp_send_intersection_stop_row = false,
    // 发送十字模式当前使用的起始搜索行。
    .udp_web_tcp_send_intersection_current_start_row = false,
    // 发送当前环岛状态机模式值。
    .udp_web_tcp_send_roundabout_mode = false,

    // ==================== TCP 原图边界点列 ====================
    // 发送原图坐标系下的左边界点列。
    .udp_web_tcp_send_left_boundary = true,
    // 发送原图坐标系下的右边界点列。
    .udp_web_tcp_send_right_boundary = true,

    // ==================== TCP IPM 边界点列 ====================
    // 发送逆透视处理后的左边界点列。
    .udp_web_tcp_send_ipm_left_boundary = true,
    // 发送逆透视处理后的右边界点列。
    .udp_web_tcp_send_ipm_right_boundary = true,
    // 发送逆透视后的原始左边界点列（未做后处理）。
    .udp_web_tcp_send_ipm_raw_left_boundary = false,
    // 发送逆透视后的原始右边界点列（未做后处理）。
    .udp_web_tcp_send_ipm_raw_right_boundary = false,

    // ==================== TCP IPM 角点调试字段 ====================
    // 发送左侧角点检测使用的边界点列。
    .udp_web_tcp_send_ipm_corner_left_boundary = false,
    // 发送右侧角点检测使用的边界点列。
    .udp_web_tcp_send_ipm_corner_right_boundary = false,
    // 发送左侧角点原始角度量。
    .udp_web_tcp_send_ipm_corner_left_raw_angle = false,
    // 发送右侧角点原始角度量。
    .udp_web_tcp_send_ipm_corner_right_raw_angle = false,
    // 发送左侧角点 NMS 后指标。
    .udp_web_tcp_send_ipm_corner_left_nms = false,
    // 发送右侧角点 NMS 后指标。
    .udp_web_tcp_send_ipm_corner_right_nms = false,

    // ==================== TCP 平移中线调试字段 ====================
    // 发送当前被 line_error 选择使用的 IPM 平移中线。
    .udp_web_tcp_send_ipm_centerline_selected_shift = true,
    // 发送当前被 line_error 选择使用的回投原图平移中线。
    .udp_web_tcp_send_src_centerline_selected_shift = true,
    // 发送当前被 line_error 选择使用的 IPM 平移中线总点数。
    .udp_web_tcp_send_ipm_centerline_selected_count = true,
    // 发送当前被 line_error 选择使用的回投原图平移中线总点数。
    .udp_web_tcp_send_src_centerline_selected_count = true,
    // 发送当前被 line_error 选择使用的 IPM 平移中线曲率数组。
    .udp_web_tcp_send_ipm_centerline_selected_curvature = true,
    // 发送曲率前瞻中的当前编码器平均速度 v。
    .udp_web_tcp_send_ipm_curvature_speed_v = true,
    // 发送曲率前瞻中的指数加权有效曲率 k_eff。
    .udp_web_tcp_send_ipm_curvature_effective = true,
    // 发送曲率前瞻中的归一化前瞻系数 eta。
    .udp_web_tcp_send_ipm_curvature_eta = true,
    // 发送曲率前瞻计算得到的实际前瞻索引。
    .udp_web_tcp_send_ipm_curvature_lookahead_index = true,
    // 发送曲率前瞻索引对应的 IPM 点坐标 [x, y]。
    .udp_web_tcp_send_ipm_curvature_lookahead_point = true,
    // 发送基于 eta 中心的高斯加权积分偏移 error。
    .udp_web_tcp_send_ipm_curvature_weighted_error = true,
    // 发送窗口内最大绝对曲率 kappa_max。
    .udp_web_tcp_send_ipm_curvature_kappa_max = true,
    // 发送窗口内最大曲率变化强度 delta_kappa_max。
    .udp_web_tcp_send_ipm_curvature_delta_kappa_max = true,
    // 发送曲率限速法计算得到的建议基准速度 v_curve（仅调试显示）。
    .udp_web_tcp_send_ipm_curvature_base_speed_curve = true,
    // 发送曲率限速原始值（未加曲率变化惩罚、未限幅）。
    .udp_web_tcp_send_ipm_curvature_v_curve_raw = true,
    // 发送曲率限速加曲率变化惩罚后的值。
    .udp_web_tcp_send_ipm_curvature_v_curve_after_dkappa = true,
    // 发送误差限速值。
    .udp_web_tcp_send_ipm_curvature_v_error_limit = true,
    // 发送最终目标基准速度（min(curve,error) 后）。
    .udp_web_tcp_send_ipm_curvature_v_target = true,

    // ==================== TCP 尺寸元数据字段 ====================
    // 发送灰度图尺寸 [width, height]。
    .udp_web_tcp_send_gray_size = true,
    // 发送 IPM 图尺寸 [width, height]。
    .udp_web_tcp_send_ipm_size = true,

    // ==================== 网页端网络地址配置 ====================
    // 电脑端接收服务 IP。
    .udp_web_server_ip = "172.21.79.129",
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

    // ==================== 视觉处理运行参数 ====================
    // ROI 抓图模式：检测到红框后周期性保存推理 ROI。
    .roi_capture_mode = false,
    // 迷宫法左右起点搜索行，值越大越靠近图像底部。
    .maze_start_row = 90,
    // 路口识别总开关：开启后允许进入十字/路口状态机。
    .intersection_enabled = false,
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
    // 所选偏移中线曲率计算步长（索引步长）。
    .ipm_centerline_curvature_step = 3,
    // 双边都丢线时保留上一帧平移中线，避免 line_error 直接掉回 0。
    .keep_last_centerline_on_double_loss = true,
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
    .ipm_line_error_weights = {0.5f, 0.3f, 0.2f},
    // 加权模式下固定使用的首点索引。
    .ipm_line_error_weighted_first_index = 4,
    // 加权模式下用于决定动态间距的点索引。
    .ipm_line_error_weighted_decision_index = 4,
    // 加权模式默认点间距。
    .ipm_line_error_weighted_default_spacing = 4,
    // 首点偏差超过 5 时，点间距切到 3。
    .ipm_line_error_weighted_spacing_threshold_1 = 4,
    // 首点偏差超过 10 时，点间距切到 2。
    .ipm_line_error_weighted_spacing_threshold_2 = 7,
    // 首点偏差超过 15 时，点间距切到 1。
    .ipm_line_error_weighted_spacing_threshold_3 = 12,
    // 阈值 1 对应点间距。
    .ipm_line_error_weighted_spacing_value_1 = 3,
    // 阈值 2 对应点间距。
    .ipm_line_error_weighted_spacing_value_2 = 2,
    // 阈值 3 对应点间距。
    .ipm_line_error_weighted_spacing_value_3 = 1,
    // 随速度索引模式公式中的速度系数 k：idx = k * speed + b。
    .ipm_line_error_speed_k = 0.02f,
    // 随速度索引模式公式中的常数项 b：idx = k * speed + b。
    .ipm_line_error_speed_b = 2.0f,
    // 随速度索引模式允许的最小索引。
    .ipm_line_error_index_min = 0,
    // 随速度索引模式允许的最大索引。
    .ipm_line_error_index_max = 30,
    // ==================== 动态前瞻速度参数（集中配置区） ====================
    // 调参建议顺序（先粗后细）：
    // 1) 先调 ay_allow / speed_gain / v_min_global，确定“速度上下边界与量级”；
    // 2) 再调 delta_kappa_gain，压住 S 弯/复合弯；
    // 3) 最后调 error_gain / error_deadband，处理偏离中线时的保守程度。
    //
    // 曲率前瞻指数权重衰减 lambda：
    // - 调大：更看近处，lookahead更保守；
    // - 调小：更看远处，lookahead更平滑。
    .ipm_curvature_lookahead_lambda = 2.0f,
    // 曲率前瞻曲率抑制 mu：
    // - 调大：eta 更容易被曲率压低（前瞻变短）；
    // - 调小：eta 更高（前瞻更长）。
    .ipm_curvature_lookahead_mu = 50.20f,
    // 历史保留参数（当前不生效，代码已改为用 line_follow base_speed）。
    .ipm_curvature_lookahead_v_max = 70.0f,
    // 曲率限速横向加速度允许值 ay_allow（工程安全值）：
    // - 调大：整体更快；
    // - 调小：整体更稳。
    .ipm_curve_speed_ay_allow = 2.0f,
    // 曲率限速 epsilon（防直道发散）：
    // - 调大：直道速度峰值更低更稳；
    // - 调小：直道速度更高但更敏感。
    .ipm_curve_speed_kappa_epsilon = 0.02f,
    // 曲率变化惩罚 K_delta_kappa：
    // - 调大：S 弯/复合弯更保守；
    // - 调小：连续弯更激进。
    .ipm_curve_speed_delta_kappa_gain = 8.0f,
    // 曲率速度映射缩放 speed_gain：
    // - 调大：整体速度档位上移；
    // - 调小：整体速度档位下移。
    .ipm_curve_speed_gain = 140.0f,
    // 误差限速系数 error_gain：
    // - 调大：偏离中线时降速更明显；
    // - 调小：偏离中线时速度保留更多。
    .ipm_curve_speed_error_gain = 0.01f,
    // 误差限速死区 error_deadband：
    // - 调大：小偏差不降速，直道更顺；
    // - 调小：更敏感，更早降速。
    .ipm_curve_speed_error_deadband = 3.0f,
    // 最低保护速度 v_min_global：
    // - 调大：弯道不会太慢；
    // - 调小：允许更慢通过复杂弯道。
    .ipm_curve_speed_v_min_global = 80.0f,
    // 额外前看点数 extra_plan_points：
    // - 调大：提前预判更远风险，偏保守；
    // - 调小：更关注近处，响应更直接。
    .ipm_curve_speed_extra_plan_points = 8,
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
    // ==================== 视觉处理器内部参数 ====================
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
