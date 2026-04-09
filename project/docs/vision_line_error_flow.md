# line_error 计算流程概述

本文概述当前工程中，在“中线数组已经生成完成”之后，`line_error` 如何被计算出来，以及偏差相关调试字段分别属于哪一种模式。

## 1. 总体结论

当前 `line_error` 的核心定义是：

`line_error = 选中的 IPM 中线决策点 x - IPM 参考中心 x`

这里的“选中的 IPM 中线决策点”不是固定指原图某一行，也不是直接取左右边界均值，而是：

1. 先在逆透视坐标系(IPM)里，从左边界或右边界生成一条“平移中线”。
2. 再从这条“当前选中的平移中线”上，按配置的模式挑出一个决策点。
3. 最后用这个决策点的 `x` 与 `IPM 图像中心 x` 做差，得到 `line_error`。

## 2. 中线数组是怎么来的

相关代码：

- `project/code/driver/vision/vision_image_processor.cpp`
- `render_ipm_boundary_image_and_update_boundaries(...)`

流程如下：

1. 原图左右边界先映射到 IPM，得到 `left_ipm` 和 `right_ipm`。
2. 对左右边界做后处理，包括：
   - 近点去重
   - 小回跳毛刺抑制
   - 可选等距重采样
   - 角点检测和必要截断
3. 状态机根据当前状态和边界质量，决定“本帧偏好使用左边界还是右边界”。
4. 选中的那一侧边界，通过 `shift_boundary_along_normal(...)` 沿法向平移固定距离，生成一条“平移中线”。
5. 该中线随后再做：
   - 锚点桥接
   - 可选三角滤波
   - 可选等距重采样
6. 最终结果写入：
   - `g_ipm_shift_left_center_x / y`
   - `g_ipm_shift_right_center_x / y`
   - 以及当前选中的 `ipm_centerline_selected_shift`

这条“当前选中的平移中线数组”就是 `line_error` 的输入。

## 3. line_error 的调用链

关键入口：

- `vision_image_processor_process_step()`
- `vision_line_error_layer_compute_from_ipm_shifted_centerline(...)`

流程如下：

1. `vision_image_processor_process_step()` 在 IPM 中线生成完成后，取出当前选中的中线数组。
2. 参考中心传入 `kIpmOutputWidth / 2`，也就是当前 IPM 画面的中垂线。
3. 调用 `vision_line_error_layer_compute_from_ipm_shifted_centerline(...)` 计算最终误差。
4. 返回值写入全局 `line_error`。

## 4. 三种模式分别怎么计算

相关枚举定义在：

- `project/code/driver/vision/vision_image_processor.h`

当前支持三种模式：

### 4.1 固定索引模式

枚举值：

- `VISION_IPM_LINE_ERROR_FIXED_INDEX = 0`

含义：

- 直接取中线数组中的固定索引点 `idx = ipm_line_error_fixed_index`
- 使用该点坐标作为决策点

公式：

- `decision_x = xs[idx]`
- `line_error = decision_x - ipm_center_x_ref`

特点：

- 简单、稳定
- 但不随速度变化，也不参考多个点

### 4.2 加权索引模式

枚举值：

- `VISION_IPM_LINE_ERROR_WEIGHTED_INDEX = 1`

含义：

- 取配置中的多个中线索引点
- 用对应权重对这些点的 `(x, y)` 做加权平均
- 平均后的结果作为最终决策点

当前默认配置：

- `ipm_line_error_method = 1`
- `ipm_line_error_point_indices = {3, 7, 11}`
- `ipm_line_error_weights = {0.48f, 0.29f, 0.23f}`

公式：

- `decision_x = sum(xs[idx_i] * w_i)`
- `line_error = decision_x - ipm_center_x_ref`

补充说明：

- 如果某些索引越界，代码会只使用仍然有效的点，并把权重按有效权重重新缩放。
- 当前网页里的 `ipm_weighted_decision_point` 和 `src_weighted_decision_point`，记录的是“首个有效加权配置点”，主要用于调试加权模式。
- 它们不是最终加权平均后的虚拟点坐标。

### 4.3 速度索引模式

枚举值：

- `VISION_IPM_LINE_ERROR_SPEED_INDEX = 2`

含义：

- 先根据当前车速，动态算一个索引
- 再取该索引对应的中线点作为决策点

公式：

- `idx = round(ipm_line_error_speed_k * current_speed + ipm_line_error_speed_b)`
- 再将 `idx` 限制在 `[ipm_line_error_index_min, ipm_line_error_index_max]`
- `decision_x = xs[idx]`
- `line_error = decision_x - ipm_center_x_ref`

特点：

- 速度越高，可把决策点往更远处挪
- 适合让控制更“看远一点”

## 5. 当前参数区里每个字段属于哪种模式

位置：

- `project/code/driver/vision/vision_config.c`

### 5.1 所有模式都会用到

- `ipm_line_error_source`
  - 决定本帧更偏向左边界平移中线还是右边界平移中线
- `udp_web_tcp_send_ipm_track_index`
  - 发送“当前真正命中的决策点索引”
  - 固定 / 加权 / 速度 三种模式都会更新
- `udp_web_tcp_send_ipm_track_point`
  - 发送“当前真正命中的决策点坐标”
  - 固定 / 加权 / 速度 三种模式都会更新

### 5.2 仅固定索引模式使用

- `ipm_line_error_fixed_index`

### 5.3 仅加权索引模式使用

- `ipm_line_error_weighted_point_count`
- `ipm_line_error_point_indices`
- `ipm_line_error_weights`
- `udp_web_tcp_send_ipm_weighted_first_point_error`
- `udp_web_tcp_send_ipm_weighted_decision_point`
- `udp_web_tcp_send_src_weighted_decision_point`

说明：

- `ipm_weighted_first_point_error` 表示“首个有效加权配置点”的偏差
- `ipm_weighted_decision_point` / `src_weighted_decision_point` 表示“首个有效加权配置点”的坐标
- 它们都只对加权索引模式有意义

### 5.4 仅速度索引模式使用

- `ipm_line_error_speed_k`
- `ipm_line_error_speed_b`
- `ipm_line_error_index_min`
- `ipm_line_error_index_max`

## 6. 当前默认实际使用的是哪种模式

当前配置是：

- `ipm_line_error_method = 1`

因此当前默认实际使用的是：

- 加权索引模式

也就是说：

1. 当前 `line_error` 不是固定点模式
2. 也不是按速度算索引模式
3. 而是基于 `{3, 7, 11}` 这三个中线索引点做加权后得到

## 7. 本次顺手清理的已废弃代码

本次核对 `vision` 目录后，清理了两类已经不再参与主链的遗留内容：

1. `vision_image_processor.cpp` 中两段 `[[maybe_unused]]` 的旧红色矩形候选逻辑
   - `build_red_search_roi_from_midline()`
   - `detect_red_rectangle_bbox(...)`
   - 这两段当前没有任何主链调用，仅作为遗留代码保留
2. `render_ipm_boundary_image_and_update_boundaries(...)` 中未使用的 `preferred_source` 形参
   - 当前实际偏好源来自状态机快照 `route_snapshot.preferred_source`
   - 原形参已不参与计算

## 8. 相关代码位置

- 中线生成主流程：
  - `project/code/driver/vision/vision_image_processor.cpp`
- 误差计算层：
  - `project/code/driver/vision/vision_line_error_layer.cpp`
- 参数定义：
  - `project/code/driver/vision/vision_config.c`
  - `project/code/driver/vision/vision_config.h`
- 启动时参数下发：
  - `project/user/main.cpp`
