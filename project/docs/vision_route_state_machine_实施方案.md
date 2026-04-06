# 视觉状态机实施方案

## 1. 目标

在 `project/code/driver/vision` 的现有视觉主链中，加入一个状态机层，放在“直边识别完成之后、中线处理之前”，用于处理十字和环岛等赛道场景切换。

目标状态包含 4 个主状态：

- `normal`
- `circle_left`
- `circle_right`
- `cross`

其中 `circle_left` 和 `circle_right` 还需要再细分子状态，按左右镜像对称实现。

## 2. 当前工程理解

当前工程的视觉主链已经比较清晰：

- 采图、灰度、OTSU、迷宫法巡线在 `vision_image_processor.cpp` 内完成
- 左右边界、角点、直边状态都已经缓存并提供访问接口
- 中线偏移选择和 `line_error` 计算已经在 IPM 结果基础上完成
- TCP 状态上报已经可以发送 `line_error`、边界数组、角点状态和中线数组

也就是说，这次改动的正确位置不是重新改一条视觉管线，而是在现有几何结果之上增加“场景状态机 + 中线偏好切换 + 特殊场景外推策略”。

## 3. 状态机规则

### 3.1 `normal`

在 `normal` 状态下：

- 如果左右两边都识别到角点，并且逆透视坐标下两角点距离小于 40，则进入 `cross`
- 如果一边标识为直线，另一边有角点，则进入有角点一侧对应的 `circle` 状态

### 3.2 `cross`

在 `cross` 状态下：

- 如果两侧边界数组同时丢线，则丢线计数加 1
- 当丢线计数大于 3 时，退回 `normal`
- 进入 `cross` 后，需要对左右边界数组末端做外推补点，再送去算中线

外推规则：

- 取每侧边界最后 7 个点估计斜率
- 在最后一个点后，按重采样间距继续补 10 个点
- 再进入中线生成流程

### 3.3 `circle_left`

左环状态分为以下子状态：

- `circle_left_begin`
- `circle_left_in`
- `circle_left_running`
- `circle_left_out`
- `circle_left_end`

要求：

- 刚进入时，子状态为 `circle_left_begin`
- `circle_left_begin` 里，中线偏移策略优先使用右边界，同时观察左边界巡线情况
- 如果左边界数量小于等于 0，则丢线标识加 1
- 丢线标识大于 3 后，开始统计左边界数量大于等于 3 的次数
- 有线标识大于 3 后，进入 `circle_left_in`
- `circle_left_in` 里，中线偏移策略优先使用左边界
- 退出条件：从入状态开始累加编码器，累加值大于 5000，或者左线数组数量小于等于 3，则进入 `circle_left_running`
- `circle_left_running` 里，中线偏移策略使用右边界，同时关注右侧角点，若找到则进入 `circle_left_out`
- `circle_left_out` 里，中线偏移策略使用左线，同时观察右侧边界是否为直边，若满足则进入 `circle_left_end`
- `circle_left_end` 里，中线偏好切回右边界，继续观察左边界情况；当两侧边界都是直边时，退回 `normal`

### 3.4 `circle_right`

右环状态与左环完全镜像，所有左右边界、角点、直边、偏移策略和计数逻辑对称反转即可。

## 4. 工程落点

建议改动集中在以下区域：

- `project/code/driver/vision/vision_image_processor.cpp`
- `project/code/driver/vision/vision_image_processor.h`
- `project/code/driver/vision/vision_config.h`
- `project/code/driver/vision/vision_config.c`
- `project/code/driver/vision/vision_transport.cpp`
- 需要时新增一个独立的状态机模块文件，例如 `vision_route_state_machine.h/.cpp`

## 5. 实现原则

- 不破坏现有巡线、发送和电机线程结构
- 状态机只依赖已算好的边界、角点、直边和编码器输入
- 圆环和十字判定尽量放在视觉几何层，控制层只读取最终状态和中线结果
- 左右环尽量写成一套镜像逻辑，减少重复代码
- 继续保留现有 `line_error` 计算入口，避免上层控制链大改

## 6. 需要新增的状态数据

至少需要保存：

- 主状态
- 子状态
- 当前进入状态的编码器基准值
- 左右丢线计数
- 左右有线计数
- cross 丢线计数
- 当前中线偏好源
- 当前是否启用 cross 外推

## 7. 验证方式

建议按以下顺序验证：

1. 先编译通过，不改外部接口行为
2. 打印或上报状态机主状态/子状态
3. 在普通直道场景验证不误入 `circle` / `cross`
4. 在模拟十字场景验证 `cross` 进入和退出
5. 在左右环场景分别验证子状态切换和中线偏好切换
6. 确认 `line_error` 在特殊场景下仍然连续、可控
