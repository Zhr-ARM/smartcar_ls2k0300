# project 与 vision_algorithm_extract 基础视觉流程对比

## 1. 范围说明

本文档只比较下面这段基础视觉链路：

1. 从灰度图开始
2. 到边界数组生成完成
3. 到角点识别完成
4. 到中线计算完成

本文档**不比较**下面这些内容：

- 十字识别
- 环岛识别
- 三岔、车库、AprilTag 等元素识别
- 上层状态机控制逻辑

本文档要比较的两个对象是：

- `vision_algorithm_extract`
- `project/code/driver/vision`

其中：

- `vision_algorithm_extract` 更像旧工程中“算法本体”的提取版
- `project` 是当前工作区里工程化后的视觉主链

---

## 2. 总结先行

如果先给一个结论：

### 2.1 两者相同的主干思想

两者在大方向上是一致的，都是：

1. 拿到灰度图
2. 做阈值化
3. 通过迷宫法提取左右边界
4. 把边界变换到俯视坐标
5. 对边界做平滑与等距化
6. 做局部几何分析识别角点
7. 通过边界法向平移生成中线

### 2.2 两者最主要的差异

`project` 相比 `vision_algorithm_extract`，主要变化有四类：

1. **工程封装更完整**
   - 处理流程集中在 `vision_image_processor.cpp`
   - 有完整缓存、配置项、数组回填接口

2. **边界预处理更强**
   - 除了三角滤波和重采样，还多了：
     - 近点去重
     - 回跳毛刺抑制

3. **角点检测数学形式不同**
   - `vision_algorithm_extract`：局部角度变化率 + NMS
   - `project`：三点夹角余弦 + 局部极小值 + NMS

4. **中线生成更偏“单侧优选”**
   - `vision_algorithm_extract`：左右中线候选都存在，由上层决定跟哪边
   - `project`：先根据左右边界质量选定左源或右源，再只保留一条主要控制中线

---

## 3. 代码入口与主要变量

## 3.1 `vision_algorithm_extract` 的主要文件

基础视觉链路主要在：

- `vision_algorithm_extract/src/imgproc.c`
- `vision_algorithm_extract/src/camera_param.c`
- `vision_algorithm_extract/include/deps/main.h`

典型公共变量：

- 图像：
  - `img_raw`
  - `img_thres`
  - `img_line`
- 边界原始点：
  - `ipts0 / ipts1`
  - `ipts0_num / ipts1_num`
- 俯视边界点：
  - `rpts0 / rpts1`
  - `rpts0_num / rpts1_num`
- 滤波后边界：
  - `rpts0b / rpts1b`
- 重采样后边界：
  - `rpts0s / rpts1s`
- 局部角度变化：
  - `rpts0a / rpts1a`
- NMS 后角度峰值：
  - `rpts0an / rpts1an`
- 中线：
  - `rptsc0 / rptsc1`
- 角点：
  - `Lpt0_found / Lpt1_found`
  - `Ypt0_found / Ypt1_found`

## 3.2 `project` 的主要文件

基础视觉链路主要在：

- `project/code/driver/vision/vision_image_processor.cpp`
- `project/code/driver/vision/vision_image_processor.h`
- `project/code/driver/vision/vision_config.c`

典型公共变量：

- 图像：
  - `g_image_gray`
  - `g_image_binary_u8`
  - `g_image_bgr`
  - `g_image_ipm_bgr`
- 原图边界数组：
  - `g_xy_x1_boundary / g_xy_y1_boundary`
  - `g_xy_x2_boundary / g_xy_y2_boundary`
  - `g_xy_x3_boundary / g_xy_y3_boundary`
- IPM 边界数组：
  - `g_ipm_xy_x1_boundary / g_ipm_xy_y1_boundary`
  - `g_ipm_xy_x2_boundary / g_ipm_xy_y2_boundary`
  - `g_ipm_xy_x3_boundary / g_ipm_xy_y3_boundary`
- 角点分析结果：
  - `g_ipm_left_boundary_angle_cos`
  - `g_ipm_right_boundary_angle_cos`
  - `g_ipm_left_boundary_corner_indices`
  - `g_ipm_right_boundary_corner_indices`
  - `g_ipm_left_boundary_corner_x / y`
  - `g_ipm_right_boundary_corner_x / y`
  - `g_src_left_boundary_corner_x / y`
  - `g_src_right_boundary_corner_x / y`
- 中线：
  - `g_ipm_shift_left_center_x / y`
  - `g_ipm_shift_right_center_x / y`
  - `g_src_shift_left_center_x / y`
  - `g_src_shift_right_center_x / y`
- 最终控制输出：
  - `line_error`

---

## 4. 从灰度图到二值图

## 4.1 `vision_algorithm_extract`

相关函数：

- `threshold`
- `adaptive_threshold`
- `getOSTUThreshold`

特点：

- 同时提供固定阈值、自适应阈值、OTSU
- 巡线函数本身还会在局部窗口内动态估计阈值

### 4.1.1 关键变量

- 输入图：`img_raw`
- 输出图：`img_thres`
- 局部阈值参数：
  - `block_size`
  - `clip_value`
- OTSU 参数：
  - `MinThreshold`
  - `MaxThreshold`
  - ROI 边界 `x0/x1/y0/y1`

### 4.1.2 算法逻辑

- `threshold`：全图固定阈值
- `adaptive_threshold`：局部均值减偏置
- `getOSTUThreshold`：统计灰度直方图，最大化类间方差

## 4.2 `project`

相关函数：

- `compute_global_otsu_threshold_u8`
- `build_binary_image_from_gray_threshold`

### 4.2.1 关键变量

- 输入灰度图：`g_image_gray`
- 输出二值图：`g_image_binary_u8`
- 阈值：`g_last_otsu_threshold`

### 4.2.2 算法逻辑

`project` 当前基础流程采用：

1. 对整张灰度图做全图 OTSU
2. 得到单个全局阈值
3. 把灰度图转成二值图

### 4.2.3 对比结论

- `vision_algorithm_extract`：阈值化手段更多，也允许巡线时做局部阈值
- `project`：更收敛，当前主流程固定走“全图 OTSU + 二值图”

---

## 5. 从二值图到边界点

## 5.1 `vision_algorithm_extract`：左手/右手自适应迷宫巡线

相关函数：

- `findline_lefthand_adaptive`
- `findline_righthand_adaptive`

### 5.1.1 输入输出

输入：

- 图像：`image_t *img`
- 起点：`x, y`
- 局部阈值参数：`block_size, clip_value`
- 输出数组：`pts[][2]`
- 输出数量：`*num`

输出：

- 一串沿边界走出的像素点

### 5.1.2 关键状态变量

- `dir`：当前朝向
- `turn`：连续转向次数
- `step`：已输出点数
- `dir_front / dir_frontleft / dir_frontright`

### 5.1.3 算法逻辑

对每一步：

1. 以当前位置为中心，计算局部均值阈值
2. 比较前方与左前/右前像素
3. 按左手法则或右手法则沿边界移动
4. 记录新的点
5. 直到：
   - 走满最大点数
   - 出界
   - 连续转向过多

### 5.1.4 特点

- 追边过程里就把局部阈值融合进来了
- 巡线算法和阈值判断是耦合的

## 5.2 `project`：起点搜索 + 左手/右手迷宫巡线

相关函数：

- `find_maze_start_from_row`
- `validate_maze_start_pair`
- `maze_trace_left_hand`
- `maze_trace_right_hand`

### 5.2.1 输入输出

输入：

- 二值图：`classify_img`
- 阈值：`white_threshold`
- 搜索行：`search_y`
- 搜索区间：`x_min/x_max`

输出：

- 左右起点：
  - `left_start_x / left_start_y`
  - `right_start_x / right_start_y`
- 左右边界点：
  - `left_pts`
  - `right_pts`

### 5.2.2 起点搜索逻辑

`find_maze_start_from_row` 会：

1. 从图像中心向左或向右扫描
2. 找到黑白交界
3. 把靠近路径的一侧作为巡线起始位置
4. 判断墙侧到底是白还是黑

相比 `vision_algorithm_extract`，`project` 把“从哪开始巡线”单独做成了显式阶段。

### 5.2.3 巡线逻辑

`maze_trace_left_hand` / `maze_trace_right_hand` 的核心仍然是迷宫法：

- 看前方
- 看左前/右前
- 更新方向
- 输出边界点

### 5.2.4 关键变量

- `wall_is_white`
- `x_min/x_max`
- `y_min`
- `y_fallback_stop_delta`
- `left_ok / right_ok`

### 5.2.5 相比提取版多出的工程逻辑

- 巡线前明确找起点
- 显式判断“墙是白色还是黑色”
- 约束巡线只在 `x_min ~ x_max` 范围内
- 如果 y 值回落太多，提前停止，防止绕回去
- 左右起点距离过近时，主动判定起点对无效

### 5.2.6 对比结论

- 两者核心巡线思想相同，都是迷宫法
- `vision_algorithm_extract` 更像纯算法函数
- `project` 更像完整的巡线子系统，包含：
  - 起点搜索
  - 起点校验
  - 边界身份判断
  - 提前停止规则

---

## 6. 边界数组回填方式

## 6.1 `vision_algorithm_extract`

该提取工程里，更强调“点集流水线变量”本身：

- `ipts -> rpts -> rptsb -> rptss -> rptsa -> rptsan`

边界更多以“点集变量”存在，而不是统一回填到一组发送/显示数组里。

也就是说：

- 它更偏算法中间结果
- 不强调统一输出缓存

## 6.2 `project`

相关函数：

- `fill_boundary_arrays_from_maze`
- `fill_boundary_arrays_from_points_to_target`
- `fill_single_line_arrays_from_points`

### 6.2.1 原图边界数组

`fill_boundary_arrays_from_maze` 会把左右边界回填到：

- 左边界：`g_xy_x1_boundary / g_xy_y1_boundary`
- 中线均值：`g_xy_x2_boundary / g_xy_y2_boundary`
- 右边界：`g_xy_x3_boundary / g_xy_y3_boundary`

这里的原图中线只是：

- 左右边界的简单均值

它不是后续真正用于控制的“法向平移中线”。

### 6.2.2 IPM 边界数组

`fill_boundary_arrays_from_points_to_target` 会把处理后的 IPM 左右边界回填到：

- `g_ipm_xy_x1_boundary / y1`
- `g_ipm_xy_x2_boundary / y2`
- `g_ipm_xy_x3_boundary / y3`

这里同样：

- `x2/y2` 是左右边界均值
- 主要用于调试和传输

### 6.2.3 对比结论

- `vision_algorithm_extract`：更偏算法点集流水线
- `project`：更强调“把结果整理成固定缓存数组”，方便发送、调试和控制接口复用

---

## 7. 从原图边界到俯视边界

## 7.1 `vision_algorithm_extract`

相关内容：

- `camera_param.c`
- `mapx / mapy`
- `H / H_inv`
- `map_inv`

### 7.1.1 输入输出

输入：

- 原图边界点 `ipts0 / ipts1`

输出：

- 俯视边界点 `rpts0 / rpts1`

### 7.1.2 算法逻辑

每个原图点 `(x, y)` 通过映射表：

- `mapx[y][x]`
- `mapy[y][x]`

得到俯视坐标。

## 7.2 `project`

相关函数：

- `transform_boundary_points_to_ipm`
- `transform_boundary_points_from_ipm_to_src`

### 7.2.1 输入输出

输入：

- 原图边界点 `left_pts / right_pts`

输出：

- `left_ipm / right_ipm`

### 7.2.2 算法逻辑

逐点调用：

- `src_point_to_ipm_point`

把原图点投到 IPM 平面。

中线或角点如果需要回原图，再通过：

- `ipm_point_to_src_point`

回去。

### 7.2.3 对比结论

- 两者都是“逐点映射到俯视平面”
- `vision_algorithm_extract` 更显式地暴露标定矩阵和大映射表
- `project` 把映射封装在函数里，对调用者更透明

---

## 8. 边界预处理

这是两者差异最大的部分之一。

## 8.1 `vision_algorithm_extract`

主流程通常是：

1. 原始边线 `rpts`
2. 三角滤波 `blur_points`
3. 等距重采样 `resample_points`
4. 局部角度变化 `local_angle_points`
5. NMS `nms_angle`

### 8.1.1 关键变量

- `rpts0b / rpts1b`
- `rpts0s / rpts1s`
- `rpts0a / rpts1a`
- `rpts0an / rpts1an`

### 8.1.2 特点

- 流程比较简洁
- 直接围绕角点分析展开

## 8.2 `project`

主流程是：

1. `left_ipm / right_ipm`
2. 近点去重 `remove_near_duplicate_boundary_points_inplace`
3. 回跳毛刺抑制 `remove_backtrack_spikes_inplace`
4. 等距重采样 `resample_boundary_points_equal_spacing_inplace`
5. 复制一份给角点支路 `left_angle / right_angle`
6. 对角点支路做三角滤波 `triangle_filter_boundary_points_inplace`
7. 做 angle-cos 角点分析

### 8.2.1 关键变量

- `left_proc / right_proc`
- `left_proc_num / right_proc_num`
- `left_angle / right_angle`
- `g_ipm_left_boundary_angle_cos`
- `g_ipm_right_boundary_angle_cos`

### 8.2.2 特点

- 三角滤波不再是整个边界处理链的第一步
- 而是作为“角点分析支路”的轻平滑
- 真正的主边界链多了：
  - 近点去重
  - 回跳毛刺抑制

### 8.2.3 对比结论

- `vision_algorithm_extract`：更直接，重心在“滤波 + 重采样 + 角度分析”
- `project`：更工程化，在重采样前先清理掉重复点和毛刺，降低后续误检

---

## 9. 三角滤波

## 9.1 `vision_algorithm_extract`

函数：

- `blur_points`

### 9.1.1 输入输出

- 输入：`pts_in`
- 输出：`pts_out`
- 参数：`kernel`

### 9.1.2 算法逻辑

对每个点：

1. 取邻域中的若干点
2. 按三角核权重加权平均
3. 得到平滑后的点

当 `kernel = 3` 时，本质上就相当于：

- `1-2-1`

### 9.1.3 结果变量

- `rpts0b / rpts1b`

## 9.2 `project`

函数：

- `triangle_filter_boundary_points_inplace`

### 9.2.1 算法逻辑

对于内部点：

- `new = (prev + 2 * cur + next) / 4`

对 `x`、`y` 分别做。

### 9.2.2 结果变量

- 直接原地修改 `left_angle / right_angle`

### 9.2.3 对比结论

- 两者原理相同
- `vision_algorithm_extract` 支持更一般的核长
- `project` 固定成单次 `1-2-1`，更轻量

---

## 10. 重采样

## 10.1 `vision_algorithm_extract`

函数：

- `resample_points`

### 10.1.1 输入输出

- 输入点集：`pts_in`
- 输出点集：`pts_out`
- 输入点数：`num1`
- 输出点数：`num2`
- 采样步长：`dist`

### 10.1.2 算法逻辑

它不是按数组索引重采样，而是：

1. 将相邻点视为折线段
2. 计算每段的欧氏长度
3. 沿折线累计弧长
4. 每达到一个固定几何距离 `dist`
5. 就在线段上插一个新点

因此它是：

- 按折线几何长度等距重采样

### 10.1.3 结果变量

- `rpts0s / rpts1s`

## 10.2 `project`

函数：

- `resample_boundary_points_equal_spacing_inplace`

### 10.2.1 输入输出

- 输入点集：`pts`
- 输入输出点数：`num`
- 画布尺寸：`width/height`
- 采样步长：`step_px`

### 10.2.2 算法逻辑

它同样是：

1. 把边界点视为折线
2. 逐段计算长度 `seg_len`
3. 维护累计距离 `dist_acc`
4. 当累计长度达到 `step_px`
5. 就在线段上插值生成新点

最后还会把终点补进去。

### 10.2.3 对比结论

- 两者都是按折线弧长等距重采样
- 都不是按索引间距重采样
- `project` 增加了：
  - 边界裁剪
  - 末尾点补齐
  - 更完整的越界保护

---

## 11. 角点识别

这是两者数学形式差异最大的部分。

## 11.1 `vision_algorithm_extract`：局部角度变化率

相关函数：

- `local_angle_points`
- `nms_angle`

### 11.1.1 输入输出

- 输入点集：`rpts0s / rpts1s`
- 输出角度变化：`rpts0a / rpts1a`
- NMS 后结果：`rpts0an / rpts1an`

### 11.1.2 算法逻辑

对每个点 `i`：

1. 取 `i-dist` 与 `i`
2. 取 `i` 与 `i+dist`
3. 构成前后两段向量
4. 用 `atan2` 算出局部转角变化
5. 得到一个角度变化序列

然后再做 NMS：

- 保留局部最强峰值
- 压制邻域内次峰

### 11.1.3 几何意义

- 直线处角度变化小
- 拐点处角度变化大

### 11.1.4 结果用途

后续基于：

- 角度变化峰值位置
- 峰值大小范围

判断某点是不是 L 角点。

## 11.2 `project`：三点夹角余弦 + 局部极小值

相关函数：

- `compute_boundary_angle_cos_3point_inplace`
- `detect_corner_indices_from_angle_cos`
- `truncate_by_first_corner_inplace`
- `build_corner_points_from_indices`

### 11.2.1 输入输出

输入：

- 平滑后的边界 `left_angle / right_angle`

输出：

- angle-cos 数组：
  - `g_ipm_left_boundary_angle_cos`
  - `g_ipm_right_boundary_angle_cos`
- 角点索引：
  - `g_ipm_left_boundary_corner_indices`
  - `g_ipm_right_boundary_corner_indices`
- 角点坐标：
  - IPM 坐标
  - 原图坐标

### 11.2.2 算法逻辑

对每个点 `i`：

1. 取 `i-s`、`i`、`i+s`
2. 构成前向量和后向量
3. 计算两向量夹角的 `cos`

几何意义：

- 越接近直线，`cos` 越接近 1
- 越接近直角，`cos` 越接近 0
- 越尖锐，`cos` 越小

之后再：

1. 先判断 `cos <= 阈值`
2. 再要求该点是局部极小值
3. 最后在邻域内做 NMS，只保留最尖锐点

### 11.2.3 特殊处理：按首角点截断边界

`project` 在识别出角点后，还会：

- 只保留第一个角点前面的边界
- 把后段截掉

这一步在 `truncate_by_first_corner_inplace` 中完成。

它意味着：

- `project` 的角点识别结果直接影响后续中线与控制边界

### 11.2.4 对比结论

- 两者目标相同，都是找局部拐点
- `vision_algorithm_extract` 用“角度变化峰值”
- `project` 用“夹角余弦谷值”
- `project` 额外把角点结果继续作用到边界截断与辅助线拼接

---

## 12. 辅助线机制

这是 `project` 相比 `vision_algorithm_extract` 非常明显的新增能力。

## 12.1 `vision_algorithm_extract`

在基础边界 -> 角点 -> 中线流程中，没有看到独立的“辅助线”拼接机制。

## 12.2 `project`

相关函数：

- `trace_auxiliary_lines_from_corners`
- `build_aux_start_from_corner`
- `append_points_with_bridge_inplace`

### 12.2.1 算法逻辑

当边界检测出角点后：

1. 在角点附近偏移出一个 seed 点
2. 从这个 seed 点继续做一次迷宫法短程巡线
3. 得到一条辅助线
4. 把辅助线映射到 IPM
5. 对辅助线也做同样的预处理与角点检测
6. 保留辅助线角点后的那一段
7. 再把它桥接拼回主边界

### 12.2.2 业务意义

这个机制是为了：

- 在主边界被角点截断后，尝试从角点往后再补一段合理边界
- 提升复杂结构下边界连续性

### 12.2.3 对比结论

- `vision_algorithm_extract`：无显式辅助线补边机制
- `project`：新增了“角点触发辅助线 + 桥接拼接”的工程增强

---

## 13. 中线计算

## 13.1 `vision_algorithm_extract`

相关函数：

- `track_leftline`
- `track_rightline`

### 13.1.1 输入输出

- 输入边线：`pts_in`
- 输出中线：`pts_out`
- 参数：
  - `approx_num`
  - `dist`

### 13.1.2 算法逻辑

对边线每个点：

1. 用前后 `approx_num` 个点估计切向方向
2. 得到单位切向
3. 计算法向
4. 平移 `dist`

左边线生成中线：

- 向右平移

右边线生成中线：

- 向左平移

### 13.1.3 结果形式

它天然支持两条中线候选：

- 左边线推导出的中线
- 右边线推导出的中线

上层再决定当前用哪条。

## 13.2 `project`

相关函数：

- `shift_boundary_along_normal`
- `postprocess_shifted_centerline_inplace`
- `select_shift_source_from_preference_and_counts`

### 13.2.1 输入输出

输入：

- 主边界 `src`
- 对侧边界 `guide`
- 平移距离 `shift_dist_px`

输出：

- 平移后的中线 `dst`

### 13.2.2 算法逻辑

对每个边界点：

1. 取前后点估计切向 `tx, ty`
2. 构造两组候选法向：
   - `(-ty, tx)`
   - `(ty, -tx)`
3. 通过对侧边界 `guide` 的方向约束，选择正确法向
4. 再通过上一点法向做连续性约束，避免法向翻转
5. 沿法向平移固定距离

这比 `vision_algorithm_extract` 的法向平移多了两层约束：

- 朝向对侧边界
- 与前一个法向保持连续

### 13.2.3 中线后处理

生成中线后，还会调用：

- `postprocess_shifted_centerline_inplace`

流程包括：

1. 近点去重
2. 回跳毛刺抑制
3. 可选三角滤波
4. 可选等距重采样

### 13.2.4 左右源选择逻辑

`project` 不是同时对外暴露两条同权重中线，而是先做源选择。

相关函数：

- `select_shift_source_from_preference_and_counts`

依据：

- 用户偏好源
- 左右边界点数
- 上一帧来源

选择出：

- 当前更适合从左边界推中线
- 还是从右边界推中线

然后只把被选中的主要中线交给控制层计算 `line_error`。

### 13.2.5 对比结论

- `vision_algorithm_extract`：左右中线候选并存，更偏原始算法形态
- `project`：通过法向方向约束、连续性约束、后处理、源选择，把中线做成更稳定的工程输出

---

## 14. 最终输出与控制接口

## 14.1 `vision_algorithm_extract`

基础流程的最终输出主要是：

- 左右边线点集
- 角点位置
- 左右中线候选
- `track_type`

它更像是：

- 供上层状态机继续使用的几何结果集合

## 14.2 `project`

基础流程的最终输出主要是：

- 原图边界数组
- IPM 边界数组
- 原图/IPM 角点坐标
- 原图/IPM 平移中线
- `line_error`

并通过：

- `vision_line_error_layer_compute_from_ipm_shifted_centerline`

进一步把中线转换成控制器直接使用的误差量。

### 14.2.1 关键区别

`project` 到“中线”还没有停，而是继续做到：

- 中线跟踪点
- 控制误差

所以它比提取工程更靠近最终控制接口。

---

## 15. 对比总结表

## 15.1 流程级对比

| 阶段 | vision_algorithm_extract | project |
| --- | --- | --- |
| 灰度图到二值图 | 固定阈值 / 自适应阈值 / OTSU，巡线里还带局部阈值 | 全图 OTSU + 二值图 |
| 起点搜索 | 未在本目录中显式展开 | 显式起点搜索与起点校验 |
| 巡线 | 左手/右手迷宫法，自适应局部阈值 | 左手/右手迷宫法，使用二值图与墙侧属性 |
| 原图边界数组 | 更偏点集变量，不强调统一缓存 | 明确回填左右边界与均值中线数组 |
| 原图到 IPM | 映射表 `mapx/mapy` | 封装成 `transform_boundary_points_to_ipm` |
| 边界预处理 | 三角滤波 + 重采样 | 去重 + 去毛刺 + 重采样 + 角点支路三角滤波 |
| 角点分析 | 局部角度变化率 + NMS | 三点夹角余弦 + 局部极小值 + NMS |
| 辅助线补边 | 无显式机制 | 有角点触发辅助线与桥接拼接 |
| 中线生成 | 左右边界法向平移，各自产生候选中线 | 先择优选边，再法向平移，后处理后输出主中线 |

## 15.2 核心思想对比

`vision_algorithm_extract` 更像：

- 一套“直接、简洁、贴近原始算法思路”的边线与角点处理链

`project` 更像：

- 在相同主干思想上，加入大量工程增强后的“可长期运行主链”

---

## 16. 最终结论

如果只比较“灰度图 -> 边界数组 -> 角点 -> 中线”这条链路，可以这样概括：

### 16.1 主干原理

两者主干原理是一致的：

- 迷宫法追边
- 俯视坐标处理
- 平滑与等距化
- 局部几何找角点
- 边界法向平移得中线

### 16.2 工程成熟度

`project` 是 `vision_algorithm_extract` 这类算法思想的工程增强版：

- 起点更稳
- 边界清洗更多
- 角点输出更完整
- 中线更稳
- 控制接口更直接

### 16.3 数学表达差异

角点识别是两者数学形式最不一样的地方：

- `vision_algorithm_extract`：角度变化率
- `project`：夹角余弦

但几何本质相同，都是在找局部拐点。

### 16.4 中线策略差异

- `vision_algorithm_extract` 倾向保留左右候选中线，交给上层策略决定
- `project` 倾向在基础视觉层先做边界择优，再输出一条更稳定的主中线

这也是两者“算法原型”与“工程主链”气质不同的最直观体现。
