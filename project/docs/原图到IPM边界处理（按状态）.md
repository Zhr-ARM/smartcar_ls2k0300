# 原图到 IPM：拿到边界后各状态的处理（按当前代码）

本文对应当前主链实现：
- `project/code/driver/vision/vision_image_processor.cpp`
- `project/code/driver/vision/vision_route_state_machine.cpp`

目标：说明在“迷宫法拿到左右边界之后”，不同状态对边界做了什么，再如何送入 IPM。

---

## 1. 总体链路（边界视角）

`二值图`  
-> `trace_left/right_boundary_selected_method` 得到轮廓点  
-> `extract_one_point_per_row_from_contour` 得到规整边界  
-> `left_pts/right_pts` 作为当前帧主边界  
-> （十字下角点检测、辅助线构建、下角点截断/可选外推）  
-> `vision_route_state_machine_update` 得到状态  
-> 按状态改写 `left_pts/right_pts`  
-> `fill_boundary_arrays_from_maze` 输出原图边界  
-> `left_ipm_pts/right_ipm_pts = copy(left_pts/right_pts)`  
-> `transform_boundary_points_to_ipm` + IPM 公共后处理  
-> 生成 IPM 边界与平移中线

关键结论：`left_pts/right_pts` 是唯一主边界真值；IPM 输入边界始终由它们复制得到。

---

## 2. 状态机之前：先做的边界预处理

在八邻域主流程中，状态机更新前会先做这些操作：

1. 十字下角点检测（左右各自）。
2. 若某侧下角点命中：该侧先执行“下角点截断”，结果保存为 `cross_base`。
3. 可选“下角点向上外推”（`cross_lower_corner_extrapolate_enabled`）：
- 仅在当前不是 CROSS 主状态时自动启用。
- 若当前是 `CROSS_1` 且该侧已有 aux 线，会跳过自动外推。

这一步决定了后续状态处理的初始边界形态。

---

## 3. 各状态对边界的处理

## 3.1 NORMAL / STRAIGHT

- 不做额外状态专属补线。
- 边界保持“规整后 + 公共预处理（如下角点截断/可选外推）”结果。

## 3.2 CROSS 主状态

### CROSS_1

1. 若某侧下角点未命中：该侧边界直接清零（防止错误延续）。
2. 若某侧 aux 线可用并找到上角点：
- 取 `cross_base`（下角点截断后的基底）；
- 构建 `下角点 -> 上角点` 的桥接线；
- 拼接 `base + bridge + aux上角点之后尾段` 回写该侧边界。
3. 若 aux 已有但上角点未命中：
- 回退为该侧 `cross_base`（不保留错误桥接）。

### CROSS_2

1. 进入 CROSS_2 时冻结 CROSS_1 最后有效下角点。
2. 用冻结下角点重新构建 aux 线并找上角点。
3. 找到上角点：该侧边界直接改为“aux 中上角点之后的段”。
4. 未找到：该侧边界清零。

### CROSS_3

1. 在规整边界上找“横向跳变 cut 点”。
2. 以固定锚点桥接到 cut 点。
3. 边界改为 `anchor->cut 的桥 + cut 后尾段`（左右各自独立）。

## 3.3 CIRCLE_LEFT / CIRCLE_RIGHT 主状态

左右逻辑镜像，核心是“在 state3/state5 主动替换对侧边界为 guide 线”。

### 左圆环

- `CIRCLE_LEFT_3`：用左侧规整边界找目标点，生成 guide，**替换右边界**。
- `CIRCLE_LEFT_5`：从右角点（或右起点）连到目标点，生成 guide，**替换右边界**。
- 其他子状态（1/2/4/6）不做上述替换。

### 右圆环

- `CIRCLE_RIGHT_3`：用右侧规整边界找目标点，生成 guide，**替换左边界**。
- `CIRCLE_RIGHT_5`：从左角点（或左起点）连到目标点，生成 guide，**替换左边界**。
- 其他子状态（1/2/4/6）不做上述替换。

### 圆环通用“送 IPM 前硬截断”

只要处于圆环主状态（左右任一）都会执行：

1. 先去掉起始人工边框前缀；
2. 再找首次触边位置（放宽到距边框 2px）；
3. 从首次触边处直接截断（不保留触边点后续段）。

---

## 4. 送入 IPM 之后的统一处理（与状态的关系）

`left_ipm_pts/right_ipm_pts` 由最终 `left_pts/right_pts` 复制得到，再做：

1. `src -> ipm` 点变换；
2. 公共边界后处理：近点去重、回跳毛刺抑制、可选等距采样。

状态对 IPM 阶段的主要影响不在“边界改写”，而在“中线选边/偏移”：

1. `preferred_source` 控制优先由左/右边界平移生成中线。
2. 圆环 `state4/state5` 会切换中线目标偏移（`route_circle_center_target_offset_from_left_px`）。
3. CROSS 主状态下，不追加“IPM 底部中点到中线起点”的桥接段。

---

## 5. 一句话总结

边界处理分两层：
- 原图层：状态机直接改 `left_pts/right_pts`（十字拼接、圆环 guide 替换、圆环触边截断）。
- IPM 层：主要做统一几何后处理与中线策略，IPM 边界输入始终来自最终原图主边界。
