# 十字场景：原图边界到 IPM 数组（单轨实现）

当前实现目标：  
`边界截断/补线/桥接` 全部只在 `left_pts/right_pts`（原图边界）完成，  
`left_ipm_pts/right_ipm_pts` 只在最后一步由原图边界统一拷贝后送入 IPM。

---

## 1. 核心变量

`主边界（唯一处理主线）`
- `left_pts/right_pts`
- `left_num/right_num`

`十字角点与状态缓存`
- 下角点：`g_cross_lower_left/right_corner_found/index/x/y`
- 辅助线：`g_cross_left/right_aux_*`
- 上角点：`g_cross_left/right_upper_corner_*`

`IPM 输入缓存（末尾才生成）`
- `left_ipm_pts/right_ipm_pts`
- `left_ipm_num/right_ipm_num`

`最终输出数组`
- 原图：`g_xy_x1/x2/x3_boundary`, `g_xy_y1/y2/y3_boundary`
- IPM：`g_ipm_xy_x1/x2/x3_boundary`, `g_ipm_xy_y1/y2/y3_boundary`

---

## 2. 全流程（`->`）

`二值图`  
-> `trace_left/right_boundary_selected_method`  
-> `left_trace_pts/right_trace_pts + dirs`  
-> `update_cross_lower_corner_detection_cache`（得到 `g_cross_lower_*`）  
-> `extract_one_point_per_row_from_contour`  
-> `left_regular_pts/right_regular_pts`  
-> `copy_boundary_points`  
-> `left_pts/right_pts`（主边界起点）

十字处理（仍然只改主边界）：

`下角点命中`  
-> `truncate_boundary_at_cross_lower_corner_inplace(left_pts/right_pts)`  
-> （可选）`extrapolate_cross_lower_boundary_inplace(left_pts/right_pts)`  
-> `CROSS_1` 时：`lower -> upper -> aux tail` 拼接，回写 `left_pts/right_pts`  
-> `CROSS_2` 时：固定锚点桥接 cut point，回写 `left_pts/right_pts`

圆环阶段（若命中）：

`circle guide 线生成`  
-> 回写 `left_pts/right_pts`

最终输出：

`left_pts/right_pts`  
-> `fill_boundary_arrays_from_maze`  
-> `g_xy_*`

`left_pts/right_pts`  
-> `copy_boundary_points`  
-> `left_ipm_pts/right_ipm_pts`（此处是唯一 src->ipm 入参映射点）  
-> `render_ipm_boundary_image_and_update_boundaries`  
-> `transform_boundary_points_to_ipm` + 去重/去毛刺/重采样  
-> `g_ipm_*`

---

## 3. 状态机在链路中的作用

`route_input`（角点、frame-wall、gap、straight 等）  
-> `vision_route_state_machine_update`  
-> `route_snapshot.main_state/sub_state`

十字迁移：
- `NORMAL/STRAIGHT -> CROSS_1`：`cross_entry_ready`
- `CROSS_1 -> CROSS_2`：`cross_stage2_ready`
- `CROSS_2 -> NORMAL`：`cross_exit_ready`

状态只决定“如何改 `left_pts/right_pts`”，不再单独改 IPM 入参数组。

---

## 4. 一致性结论

当前版本已经是单轨：  
`left_pts/right_pts` 是唯一边界真值，  
`left_ipm_pts/right_ipm_pts` 仅由最终主边界派生。  
因此不会再出现“CROSS_2 只改 IPM 输入导致 src/ipm 分叉”的结构性问题。
