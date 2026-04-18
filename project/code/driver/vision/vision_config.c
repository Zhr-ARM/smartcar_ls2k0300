#include "driver/vision/vision_config.h"

vision_runtime_config_t g_vision_runtime_config = {
    // ==================== 参数区域 3: 网页发送 ====================
    // 说明：发送相关参数集中在最前，包含逐飞助手、UDP 视频、TCP 状态上报。
    //
    // ==================== 网页图传怎么选 ====================
    // 当前代码已实现：灰度 / 二值 / 彩图 均可选 JPEG / PNG / BMP。
    //
    // 若按“负载量从小到大”粗略排序：
    // 1. 灰度 JPEG
    // 2. 彩图 JPEG
    // 3. 灰度 PNG（未来建议）
    // 4. 彩图 PNG（未来建议）
    // 5. 灰度 BMP（未来建议）
    // 6. 彩图 BMP（未来建议，通常不建议）
    //
    // 若按“本地复算保真度”排序：
    // 1. 灰度 PNG
    // 2. 灰度 BMP
    // 3. 彩图 PNG
    // 4. 彩图 BMP
    // 5. 灰度 JPEG
    // 6. 彩图 JPEG
    //
    // 当前代码里最常用的 3 套配置：
    // A. 本地复算优先：
    //    udp_web_send_gray_jpeg = true
    //    udp_web_send_rgb_jpeg = false
    //    udp_web_send_binary_jpeg = false
    //    udp_web_data_profile = VISION_WEB_DATA_PROFILE_RAW_MINIMAL
    //
    // B. 前端展示彩图优先：
    //    udp_web_send_gray_jpeg = false
    //    udp_web_send_rgb_jpeg = true
    //    udp_web_send_binary_jpeg = false
    //    udp_web_data_profile = VISION_WEB_DATA_PROFILE_RAW_MINIMAL
    //
    // C. 全量调试：
    //    可选灰度图或彩图图传
    //    udp_web_data_profile = VISION_WEB_DATA_PROFILE_FULL
    //
    // 经验建议：
    // 1. 不建议长期同时发送灰度图和彩图，重复占带宽。
    // 2. 不建议长期发送二值图，网页侧可由灰度图 + otsu_threshold 现算。
    // 3. 若要兼顾“低负载 + 高保真”，优先把灰度图格式切到 PNG。
    // ==================== 本地显示与逐飞助手链路 ====================
    // 图传输出模式：
    // 0=binary 二值图，1=gray 灰度图(带线)。
    .send_mode = 1,
    // 图传发送上限帧率，0 表示不限速。
    .send_max_fps = 60,
    // 推理总开关：false 时关闭红色识别与 ncnn 推理。
    .infer_enabled = false,
    // ncnn 子开关：false 时只保留红框检测，不做 ncnn 分类。
    .ncnn_enabled = false,
    // ncnn 默认模型输入尺寸。
    .ncnn_input_width = 64,
    .ncnn_input_height = 64,
    // ncnn 标签顺序必须与模型输出类别索引严格一致。
    .ncnn_label_count = 3,
    .ncnn_labels = {
        "supplies",
        "vehicles",
        "weapons",
    },
    // 逐飞客户端发送开关。
    .client_sender_enabled = false,
    // 车载屏显示开关。
    .screen_display_enabled = false,

    // ==================== 网页端 UDP 视频发送配置 ====================
    // UDP 网页图传总开关。
    .udp_web_enabled = true,
    // UDP 网页图传发送上限帧率，0 表示不限速。
    .udp_web_max_fps = 30,
    // 是否向网页端发送灰度图。
    // 当前网页端可用该灰度图结合 TCP 下发的 otsu_threshold 自行实时二值化。
    // 若目标是“本地复算尽量贴近主板”，优先开这个。
    .udp_web_send_gray_jpeg = true,
    // 灰度图格式：0=JPEG，1=PNG，2=BMP。
    // 当前保持 JPEG，不改变现有配置行为。
    .udp_web_gray_image_format = VISION_WEB_IMAGE_FORMAT_JPEG,
    // 是否向网页端发送二值图。
    // 当前需求为停发二值图，因此默认关闭。
    // 推荐继续保持关闭，除非临时想单独观察二值结果。
    .udp_web_send_binary_jpeg = false,
    // 二值图格式：0=JPEG，1=PNG，2=BMP。
    // 当前保持 JPEG，不改变现有配置行为。
    .udp_web_binary_image_format = VISION_WEB_IMAGE_FORMAT_JPEG,
    // 是否向网页端发送原始 BGR 彩图。
    // 若目标是“网页主要看彩图”，开这个；网页侧可在没有灰度图时自行转灰度。
    .udp_web_send_rgb_jpeg = false,
    // 彩图格式：0=JPEG，1=PNG，2=BMP。
    // 当前保持 JPEG，不改变现有配置行为。
    .udp_web_rgb_image_format = VISION_WEB_IMAGE_FORMAT_PNG,
    // 网页端 TCP 数据模式：
    // 0=全量调试数据，1=仅原始图像 + 最小原始状态。
    .udp_web_data_profile = 0,

    // ==================== 网页端 TCP 状态发送总开关 ====================
    // TCP 状态上报开关。
    .udp_web_tcp_enabled = true,

    // ==================== TCP 基础状态字段（通用） ====================
    // 发送主板当前时间戳（ms）。
    .udp_web_tcp_send_ts_ms = false,

    // ==================== TCP 字段（视觉相关） ====================
    // 发送当前巡线误差 line_error。
    .udp_web_tcp_send_line_error = true,
    // 发送主板 CPU 占用百分比（主板端每秒更新一次）。
    .udp_web_tcp_send_cpu_usage_percent = true,
    // 发送主板内存占用百分比（主板端每秒更新一次）。
    .udp_web_tcp_send_mem_usage_percent = true,

    // ==================== TCP 字段（电机相关） ====================
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
    .udp_web_tcp_send_ipm_track_valid = true,
    // 发送 line_error 的取点方式：固定索引 / 加权索引 / 随速度索引。
    .udp_web_tcp_send_ipm_track_method = false,
    // 发送当前采用的是左偏移中线还是右偏移中线。
    .udp_web_tcp_send_ipm_centerline_source = false,
    // 发送当前实际命中的中线点索引。
    .udp_web_tcp_send_ipm_track_index = false,
    // 发送当前实际跟踪点坐标 [x, y]。
    .udp_web_tcp_send_ipm_track_point = true,
    // 发送首点的当前偏差。

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
    // ==================== TCP 尺寸元数据字段 ====================
    // 发送灰度图尺寸 [width, height]。
    .udp_web_tcp_send_gray_size = true,
    // 发送 IPM 图尺寸 [width, height]。
    .udp_web_tcp_send_ipm_size = true,

    // ==================== 网页端网络地址配置 ====================
    // 电脑端接收服务 IP。
    .udp_web_server_ip = "172.21.79.129",
    // .udp_web_server_ip = "10.119.73.102",
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

    // ==================== 参数区域 1: 视觉处理 ====================
    // 说明：视觉处理链参数（迷宫法、去畸变、IPM 边界/中线后处理）集中在这里。
    // 迷宫法左右起点搜索行，值越大越靠近图像底部。
    .maze_start_row = 100,
    // 原图巡线方法：0=迷宫法，1=八邻域法。
    .maze_trace_method = 1,
    // 迷宫法巡线回退停止阈值（y > min_y + 阈值即停）。
    .maze_trace_y_fallback_stop_delta = 15,
    // 十字下角点 dir 模板识别开关：仅依赖八邻域原始 dir/points，按相邻平台跳变模板识别。
    .cross_lower_corner_dir_enabled = true,
    // 前平台窗口：检测十字下角点前，要求最近一段形成较长的 {4,5} 或 {5,6} 平台。
    .cross_lower_corner_pre_window = 8,
    // 后平台窗口：检测十字下角点后，要求后续一段形成较长的 {1,2} 或 {2,3} 平台。
    .cross_lower_corner_post_window = 8,
    // 前平台最少票数：窗口内至少有这些点数落在 {4,5} 或 {5,6} 这两组相邻平台之一。
    .cross_lower_corner_pre_min_votes = 7,
    // 后平台最少票数：窗口内至少有这些点数落在 {1,2} 或 {2,3} 这两组相邻平台之一。
    .cross_lower_corner_post_min_votes = 7,
    // 最大过渡长度：允许前后长平台之间只存在很短的跳变区，跳变区可短暂扫过 1..6。
    .cross_lower_corner_transition_max_len = 4,
    // 左右下角点 y 坐标最大差值：用于判断双下角点是否稳定同时成立。
    .cross_lower_corner_pair_y_diff_max = 30,
    // 普通原图下角点处理：默认开启“截断后拟合直线并向上补一段”。
    .cross_lower_corner_extrapolate_enabled = false,
    // 十字补线触发的下角点最小 y 阈值。
    .cross_lower_corner_extrapolate_min_y = 35,
    // 十字下角点补线向上的延伸长度（按 y 行数计算）。
    .cross_lower_corner_extrapolate_y_span = 30,
    // 原图直边判断：检查边界数组前 60 个点。
    .src_boundary_straight_check_count = 80,
    // 原图直边判断：前 N 个点中 90% 以上为 dir=4/5 判定为直边。
    .src_boundary_straight_dir45_ratio_min = 0.95f,
    // 去畸变开关：true=启用标定参数矫正，false=原图直通。
    .undistort_enabled = false,
    // ---------- 边界双处理流水线：共同预处理 ----------
    // IPM 处理链边界三角滤波开关（作用于角点支路，单次平滑）。
    .ipm_triangle_filter_enabled = true,
    // IPM 处理链边界等距采样开关。
    .ipm_resample_enabled = true,
    // IPM 处理链边界等距采样步长，单位 px。
    .ipm_resample_step_px = 3.0f,
    // 边界顺序近点去重阈值，单位 px。
    .ipm_boundary_min_point_dist_px = 1.4f,
    // 边界回跳毛刺抑制：短段长度阈值，单位 px。
    .ipm_boundary_spike_short_seg_max_px = 2.0f,
    // 边界回跳毛刺抑制：反向判定 cos 阈值。
    .ipm_boundary_spike_reverse_cos_threshold = -0.2f,
    // ---------- 边界双处理流水线：角点支路 ----------
    // 边界三点法夹角 cos 计算步长（索引步长）。
    .ipm_boundary_angle_step = 3,
    // 角点候选 cos 阈值（<=阈值进入局部极小值检测）。
    .ipm_boundary_corner_cos_threshold = 0.5f,
    // 角点 NMS 半径（索引半径）。
    .ipm_boundary_corner_nms_radius = 3,
    // 检测到首个角点后是否截断其后的边界。
    .ipm_boundary_truncate_at_first_corner_enabled = true,
    // 直边检测最小边界点数。
    .ipm_boundary_straight_min_points = 30,
    // 直边检测起始检查窗口长度。
    .ipm_boundary_straight_check_count = 20,
    // 直边检测起始窗口内 cos 最小阈值。
    .ipm_boundary_straight_min_cos = 0.95f,
    // ---------- 边界双处理流水线：中线生成 ----------
    // 逆透视后期望赛道宽度，单位 px。
    .ipm_track_width_px = 28.0f,
    // 目标中线距离左边界的偏移，单位 px。
    // 当前 14 表示落在 28px 赛道宽度的正中间。
    .ipm_center_target_offset_from_left_px = 14.0f,
    // 武器类别：靠左走。
    .ipm_center_target_offset_weapons_from_left_px = 4.0f,
    // 物资类别：靠右走。
    .ipm_center_target_offset_supplies_from_left_px = 24.0f,
    // 从正常巡线切入目标引导前，先累计 5 个有效推理结果再决策类别。
    .infer_offset_vote_result_count = 5,
    // 连续 5 帧无红框后，退出目标引导并恢复默认偏移。
    .infer_offset_restore_no_red_count = 5,
    // 中线独立后处理总开关。
    .ipm_centerline_postprocess_enabled = true,
    // 中线三角滤波开关。
    .ipm_centerline_triangle_filter_enabled = true,
    // 中线等距采样开关。
    .ipm_centerline_resample_enabled = true,
    // 中线等距采样步长，单位 px。
    .ipm_centerline_resample_step_px = 3.0f,
    // 中线曲率计算总开关。
    .ipm_centerline_curvature_enabled = true,
    // 所选偏移中线曲率计算步长（索引步长）。
    .ipm_centerline_curvature_step = 3,
    // 斑马线检测开关。
    .zebra_cross_detection_enabled = false,
    // 双边都丢线时保留上一帧平移中线，避免 line_error 直接掉回 0。
    .keep_last_centerline_on_double_loss = true,
    // 状态机十字识别开关。
    .route_cross_detection_enabled = true,
    // 双侧角点后沿边框连续行数都至少达到 5，才允许 normal/straight 进入 cross。
    .route_cross_entry_corner_post_frame_wall_rows_min = 5,
    // cross_1 -> cross_2：起始巡线行一旦碰到边框边界就认为进入 cross_2 条件成立。
    .route_cross_stage2_enter_start_frame_wall_rows_min = 1,
    // cross_2 -> normal：左右起始边界 x 差小于 150 视为驶出十字。
    .route_cross_exit_start_gap_x_max = 155,
    // 状态机圆环识别开关。
    .route_circle_detection_enabled = true,
    // 圆环入口判定：对侧边界最少点数。
    .route_circle_entry_min_boundary_count = 20,
    // 圆环入口判定：角点索引距离边界尾部至少保留的余量。
    .route_circle_entry_corner_tail_margin = 20,
    // 圆环状态 1/5 的起始贴边连续行数阈值。
    .route_circle_stage_frame_wall_rows_enter = 30,
    // 圆环状态 3 的对侧起始贴边连续行数触发阈值。
    .route_circle_stage3_frame_wall_rows_trigger = 70,
    // 圆环状态 6：搜线起始行至少抬高到该行。
    .route_circle_stage6_maze_start_row = 50,
    // 圆环状态中线偏移：仅在 circle4/5 生效；左圆环相对左边界偏移，右圆环走镜像偏移。
    .route_circle_center_target_offset_from_left_px = 14.0f,
    // 圆环补线：贴边连续段最小长度阈值。
    .circle_guide_min_frame_wall_segment_len = 8,
    // 圆环 state3 补线左/右目标点在规则边界上的后移索引。
    .circle_guide_target_offset_stage3 = 5,
    // 圆环 state5 补线锚点向后偏移索引。
    .circle_guide_anchor_offset_stage5 = 0,
    // 进入 straight 状态所需连续帧数。
    .route_straight_enter_consecutive_frames = 2,
    // straight 判定：从 0 到最后误差索引的绝对误差和必须小于该值。
    .route_straight_abs_error_sum_max = 20.0f,
    // cross_1：沿角点同 x 向上找白->黑转变，最多 75 行。
    .cross_aux_vertical_scan_max_rows = 75,
    // cross_1：辅助边界八邻域最多保存 80 个点。
    .cross_aux_trace_max_points = 50,
    // cross_1：辅助边界最多向上延伸 30 行。
    .cross_aux_trace_upward_rows_max = 30,
    // cross_2：4->6 跳变前平台要求 3 个连续 dir=4。
    .cross_upper_dir4_pre_run_len = 3,
    // cross_2：允许最多 3 个过渡点，值可以落在 4/5/6。
    .cross_upper_transition_max_len = 3,
    // cross_2：4->6 跳变后平台要求 3 个连续 dir=6。
    .cross_upper_dir6_post_run_len = 3,
    // ==================== 参数区域 2: 偏差计算 ====================
    // 说明：line_error 取点参数集中在这里。
    // line_error 平移中线偏好源：0=偏好左，1=偏好右，2=无偏好(自动按边界点数)。
    .ipm_line_error_source = VISION_IPM_LINE_ERROR_FROM_AUTO,
    // line_error 计算方法：0=固定索引，1=加权索引，2=随速度索引，3=加权+速度增量。
    .ipm_line_error_method = 1,
    // 固定索引模式下默认使用的中线点索引。
    .ipm_line_error_fixed_index = 1,
    // line_error 加权点数量。
    .ipm_line_error_weighted_point_count = 3,
    // line_error 加权索引点（0-based）。
    .ipm_line_error_point_indices = {8, 16, 24},
    // line_error 各索引点对应权重。
    .ipm_line_error_weights = {0.5f, 0.3f, 0.2f},
    // 随速度索引模式公式中的速度系数 k：idx = k * speed + b。
    .ipm_line_error_speed_k = 0.02f,
    // 随速度索引模式公式中的常数项 b：idx = k * speed + b。
    .ipm_line_error_speed_b = 2.0f,
    // 随速度索引模式允许的最小索引。
    .ipm_line_error_index_min = 0,
    // 随速度索引模式允许的最大索引。
    .ipm_line_error_index_max = 30,
    // 基准速度方案B已移除，偏差层仅保留方案A所需参数。
};

vision_processor_config_t g_vision_processor_config = {
    // ==================== 视觉处理器内部参数 ====================
    // 迷宫法单侧最大输出点数。
    .maze_trace_max_points = 180,
    // 迷宫法允许追踪的纵向区域百分比，100 表示全高。
    .maze_lower_region_percent = 85,
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
        {0.029630, -0.024954, -0.117149},
        {-0.000000, 0.000207, 0.607059},
        {-0.000000, -0.000323, 0.051471}
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
    .default_maze_trace_x_min = 1,
    // 迷宫法默认允许搜索的最大 x，屏蔽右侧黑边。
    .default_maze_trace_x_max = 159
};
