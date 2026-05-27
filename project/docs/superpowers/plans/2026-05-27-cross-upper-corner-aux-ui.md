# 十字上角点辅助边界检测与 UI 可视化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在下角点检测同期，基于同侧辅助边界检测上角点，并在网页 UI 中用清晰颜色绘制辅助边界与上下左右角点。

**Architecture:** 保留现有十字状态机不动，只改视觉检测和调试可视化。C++ 侧复用现有 `build_cross_aux_boundary()` 生成的辅助 trace/regular 边界，把当前“第 N 个 dir 命中点”的上角点占位逻辑替换为可配置模板匹配；TCP 状态字段基本沿用已有 `cross_*_aux_*` 与 `cross_*_upper_corner_*` 字段。Web UI 侧只改 overlay 绘制：辅助边界始终按左右不同颜色可见，角点用小圆点和颜色编码，去掉拥挤文字。

**Tech Stack:** C++17 vision pipeline, existing TOML runtime config, existing TCP JSON status, vanilla JS canvas UI in `tools/pc_receiver_js/public/index.html`

---

## 当前代码基线

已有能力：

- `project/code/driver/vision/vision_image_processor.cpp` 已有辅助边界缓存、状态读取接口和 TCP 字段：
  - `build_cross_aux_boundary(...)`
  - `vision_image_processor_get_cross_aux_line_state(...)`
  - `vision_image_processor_get_cross_upper_corner_state(...)`
- `project/code/driver/vision/vision_transport.cpp` 已上报：
  - `cross_left_aux_trace`, `cross_right_aux_trace`
  - `cross_left_aux_regular`, `cross_right_aux_regular`
  - `cross_left_upper_corner_point`, `cross_right_upper_corner_point`
- `tools/pc_receiver_js/public/index.html` 已粗略绘制辅助边界和上下角点，但：
  - 辅助边界仅在 `route_sub_state === 1` 时显示；
  - 角点使用同一个绘制函数和文字标签；
  - 圆半径为 4，路口远时四个角点文字拥挤。

缺口：

- 上角点当前通过 `find_nth_or_last_dir_point_on_trace(..., 5, 6, ...)` 找第 6 个 dir=5 点，不是用户描述的模板匹配。
- 配置项 `cross_upper_dir4_pre_run_len` / `cross_upper_transition_max_len` / `cross_upper_dir6_post_run_len` 已存在，但当前上角点逻辑没有使用这些模板参数。
- 辅助边界可视化应脱离十字状态子状态限制，便于调参时始终看到检测结果。

## 涉及文件

| 文件 | 变更类型 |
|------|----------|
| `project/code/driver/vision/vision_image_processor.cpp` | 修改：新增上角点模板匹配函数，替换 CROSS_1/CROSS_2 上角点占位查找 |
| `project/code/driver/vision/vision_config.h` | 检查：已有上角点模板配置字段，不新增字段 |
| `project/code/driver/vision/vision_config.c` | 检查：已有上角点模板默认值，不新增字段 |
| `project/user/smartcar_config.toml` | 检查：已有上角点模板参数，按实验结果调参 |
| `project/code/driver/vision/vision_transport.cpp` | 检查：确认已有字段完整，无需新增协议字段 |
| `tools/pc_receiver_js/public/index.html` | 修改：辅助边界颜色、角点小圆与颜色编码、去掉文字标签 |

---

### Task 1: 实现上角点 dir 模板匹配函数

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp`

- [ ] **Step 1: 在 `find_nth_or_last_dir_point_on_trace` 前新增上角点模板匹配 helper**

插入位置：`project/code/driver/vision/vision_image_processor.cpp` 中 `static bool find_nth_or_last_dir_point_on_trace(...)` 定义之前。

```cpp
static bool dir_is_valid_transition(uint8 dir)
{
    return dir >= 1 && dir <= 8;
}

static bool all_dirs_equal(const uint8 *dirs, int start, int end, uint8 expected)
{
    if (dirs == nullptr || start < 0 || end <= start)
    {
        return false;
    }
    for (int i = start; i < end; ++i)
    {
        if (dirs[i] != expected)
        {
            return false;
        }
    }
    return true;
}

static bool find_cross_upper_corner_from_aux_trace(const maze_point_t *trace_pts,
                                                   const uint8 *trace_dirs,
                                                   int trace_count,
                                                   bool is_left,
                                                   maze_point_t *corner_point,
                                                   int *corner_index)
{
    if (corner_point) *corner_point = maze_point_t{0, 0};
    if (corner_index) *corner_index = -1;
    if (trace_pts == nullptr || trace_dirs == nullptr || trace_count <= 0)
    {
        return false;
    }

    const int pre_run = std::clamp(g_vision_runtime_config.cross_upper_dir4_pre_run_len, 1, VISION_BOUNDARY_NUM);
    const int transition_max = std::clamp(g_vision_runtime_config.cross_upper_transition_max_len, 0, VISION_BOUNDARY_NUM);
    const int post_run = std::clamp(g_vision_runtime_config.cross_upper_dir6_post_run_len, 1, VISION_BOUNDARY_NUM);

    // 左右上角点在辅助边界上互为镜像：左侧默认 4 -> 6，右侧默认 6 -> 4。
    const uint8 pre_dir = is_left ? 4 : 6;
    const uint8 post_dir = is_left ? 6 : 4;

    for (int i = pre_run; i + post_run <= trace_count; ++i)
    {
        const int pre_start = i - pre_run;
        if (!all_dirs_equal(trace_dirs, pre_start, i, pre_dir))
        {
            continue;
        }

        for (int transition_len = 0; transition_len <= transition_max; ++transition_len)
        {
            const int post_start = i + transition_len;
            const int post_end = post_start + post_run;
            if (post_end > trace_count)
            {
                break;
            }

            bool transition_ok = true;
            for (int k = i; k < post_start; ++k)
            {
                if (!dir_is_valid_transition(trace_dirs[k]))
                {
                    transition_ok = false;
                    break;
                }
            }
            if (!transition_ok || !all_dirs_equal(trace_dirs, post_start, post_end, post_dir))
            {
                continue;
            }

            const int idx = std::clamp(post_start, 0, trace_count - 1);
            if (corner_point) *corner_point = trace_pts[idx];
            if (corner_index) *corner_index = idx;
            return true;
        }
    }

    return false;
}
```

- [ ] **Step 2: 编译验证新增 helper**

Run:

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/project/out
make -j12
```

Expected: 编译通过。如果 `project/out` 不存在，先运行：

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/project
cmake -S user -B out -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DUVC_RES_PRESET=1 -DUVC_FORMAT_PRESET=0 -DENABLE_NCNN=0
cmake --build out -j12
```

- [ ] **Step 3: Commit**

```bash
git add project/code/driver/vision/vision_image_processor.cpp
git commit -m "feat: add cross upper corner template matcher"
```

---

### Task 2: 替换 CROSS_1 上角点占位逻辑

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp`

- [ ] **Step 1: 替换 CROSS_1 左上角查找调用**

在 `VISION_ROUTE_SUB_CROSS_1` 分支中，把左侧：

```cpp
if (find_nth_or_last_dir_point_on_trace(left_cross_aux_trace_pts.data(),
                                        left_cross_aux_trace_dirs.data(),
                                        left_cross_aux_trace_num,
                                        5,
                                        6,
                                        &left_upper_corner,
                                        &left_upper_trace_index))
```

替换为：

```cpp
if (find_cross_upper_corner_from_aux_trace(left_cross_aux_trace_pts.data(),
                                           left_cross_aux_trace_dirs.data(),
                                           left_cross_aux_trace_num,
                                           true,
                                           &left_upper_corner,
                                           &left_upper_trace_index))
```

- [ ] **Step 2: 替换 CROSS_1 右上角查找调用**

把右侧：

```cpp
if (find_nth_or_last_dir_point_on_trace(right_cross_aux_trace_pts.data(),
                                        right_cross_aux_trace_dirs.data(),
                                        right_cross_aux_trace_num,
                                        5,
                                        6,
                                        &right_upper_corner,
                                        &right_upper_trace_index))
```

替换为：

```cpp
if (find_cross_upper_corner_from_aux_trace(right_cross_aux_trace_pts.data(),
                                            right_cross_aux_trace_dirs.data(),
                                            right_cross_aux_trace_num,
                                            false,
                                            &right_upper_corner,
                                            &right_upper_trace_index))
```

- [ ] **Step 3: 编译验证**

Run:

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/project/out
make -j12
```

Expected: 编译通过。

- [ ] **Step 4: Commit**

```bash
git add project/code/driver/vision/vision_image_processor.cpp
git commit -m "feat: use upper corner template matching in cross1"
```

---

### Task 3: 替换 CROSS_2 上角点占位逻辑

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp`

- [ ] **Step 1: 替换 CROSS_2 左上角查找调用**

在 `VISION_ROUTE_SUB_CROSS_2` 分支中，把左侧条件：

```cpp
if (g_cross_left_aux_found.load() &&
    left_cross_aux_regular_num > 0 &&
    find_nth_or_last_dir_point_on_trace(left_cross_aux_trace_pts.data(),
                                        left_cross_aux_trace_dirs.data(),
                                        left_cross_aux_trace_num,
                                        5,
                                        6,
                                        &left_upper_corner,
                                        &left_upper_trace_index))
```

替换为：

```cpp
if (g_cross_left_aux_found.load() &&
    left_cross_aux_regular_num > 0 &&
    find_cross_upper_corner_from_aux_trace(left_cross_aux_trace_pts.data(),
                                           left_cross_aux_trace_dirs.data(),
                                           left_cross_aux_trace_num,
                                           true,
                                           &left_upper_corner,
                                           &left_upper_trace_index))
```

- [ ] **Step 2: 替换 CROSS_2 右上角查找调用**

把右侧条件：

```cpp
if (g_cross_right_aux_found.load() &&
    right_cross_aux_regular_num > 0 &&
    find_nth_or_last_dir_point_on_trace(right_cross_aux_trace_pts.data(),
                                        right_cross_aux_trace_dirs.data(),
                                        right_cross_aux_trace_num,
                                        5,
                                        6,
                                        &right_upper_corner,
                                        &right_upper_trace_index))
```

替换为：

```cpp
if (g_cross_right_aux_found.load() &&
    right_cross_aux_regular_num > 0 &&
    find_cross_upper_corner_from_aux_trace(right_cross_aux_trace_pts.data(),
                                            right_cross_aux_trace_dirs.data(),
                                            right_cross_aux_trace_num,
                                            false,
                                            &right_upper_corner,
                                            &right_upper_trace_index))
```

- [ ] **Step 3: 编译验证**

Run:

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/project/out
make -j12
```

Expected: 编译通过。

- [ ] **Step 4: Commit**

```bash
git add project/code/driver/vision/vision_image_processor.cpp
git commit -m "feat: use upper corner template matching in cross2"
```

---

### Task 4: 改善网页 UI 角点与辅助边界绘制

**Files:**
- Modify: `tools/pc_receiver_js/public/index.html`

- [ ] **Step 1: 新增角点颜色表和通用小圆绘制函数**

在现有 `drawCrossLowerCornerPoint(...)` 函数之前插入：

```js
    const cornerPalette = {
      lowerLeft: '#f97316',
      lowerRight: '#facc15',
      upperLeft: '#a855f7',
      upperRight: '#ef4444',
      auxLeft: '#0ea5e9',
      auxRight: '#10b981'
    };

    function drawCornerMarker(ctx, point, color, active) {
      if (!Array.isArray(point) || point.length < 2 || !active) return;
      const x = Number(point[0]);
      const y = Number(point[1]);
      if (!Number.isFinite(x) || !Number.isFinite(y)) return;

      ctx.fillStyle = color;
      ctx.strokeStyle = '#020617';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(x, y, 2.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
```

- [ ] **Step 2: 保留旧函数兼容但缩小圆并删除文字**

把 `drawCrossLowerCornerPoint(...)` 函数体替换为：

```js
    function drawCrossLowerCornerPoint(ctx, point, label, active) {
      const color = label === 'L下角' ? cornerPalette.lowerLeft
        : label === 'R下角' ? cornerPalette.lowerRight
        : label === 'L上角' ? cornerPalette.upperLeft
        : label === 'R上角' ? cornerPalette.upperRight
        : label === 'L辅起' ? cornerPalette.auxLeft
        : label === 'R辅起' ? cornerPalette.auxRight
        : '#f8fafc';
      drawCornerMarker(ctx, point, color, active);
    }
```

- [ ] **Step 3: 让辅助边界按数据存在即绘制，不再只依赖 `route_sub_state === 1`**

在 `drawBinaryTraceOverlay(status)` 中，把：

```js
      const routeSubState = Number(status && status.route_sub_state);
      const showCrossAux = routeSubState === 1;
```

替换为：

```js
      const showCrossAux =
        Array.isArray(status.cross_left_aux_trace) ||
        Array.isArray(status.cross_right_aux_trace) ||
        Array.isArray(status.cross_left_aux_regular) ||
        Array.isArray(status.cross_right_aux_regular);
```

- [ ] **Step 4: 区分辅助 trace 与 regular 的左右颜色**

把辅助边界绘制块：

```js
      if (showCrossAux) {
        drawSeries(binaryCtx, status.cross_left_aux_trace || [], '#e11d48', 2, 1);
        drawSeries(binaryCtx, status.cross_right_aux_trace || [], '#14b8a6', 2, 1);
        drawSeries(binaryCtx, status.cross_left_aux_regular || [], '#fb7185', 2, 1);
        drawSeries(binaryCtx, status.cross_right_aux_regular || [], '#2dd4bf', 2, 1);
      }
```

替换为：

```js
      if (showCrossAux) {
        drawSeries(binaryCtx, status.cross_left_aux_trace || [], '#0ea5e9', 2, 0.8);
        drawSeries(binaryCtx, status.cross_right_aux_trace || [], '#10b981', 2, 0.8);
        drawSeries(binaryCtx, status.cross_left_aux_regular || [], '#7dd3fc', 1.5, 0.8);
        drawSeries(binaryCtx, status.cross_right_aux_regular || [], '#86efac', 1.5, 0.8);
      }
```

- [ ] **Step 5: 角点按上下左右颜色编码绘制**

确认 `drawBinaryTraceOverlay(status)` 中保留以下调用：

```js
      drawCrossLowerCornerPoint(binaryCtx, status.cross_lower_left_corner_point, 'L下角', status.cross_lower_left_corner_found);
      drawCrossLowerCornerPoint(binaryCtx, status.cross_lower_right_corner_point, 'R下角', status.cross_lower_right_corner_found);
      if (showCrossAux) {
        drawCrossLowerCornerPoint(binaryCtx, status.cross_left_aux_transition_point, 'L辅起', status.cross_left_aux_found);
        drawCrossLowerCornerPoint(binaryCtx, status.cross_right_aux_transition_point, 'R辅起', status.cross_right_aux_found);
      }
      drawCrossLowerCornerPoint(binaryCtx, status.cross_left_upper_corner_point, 'L上角', status.cross_left_upper_corner_found);
      drawCrossLowerCornerPoint(binaryCtx, status.cross_right_upper_corner_point, 'R上角', status.cross_right_upper_corner_found);
```

注意：这里下角点 active 使用左右 found，而不是 pair_valid；这样单侧命中时也能看见。

- [ ] **Step 6: UI smoke test**

Run:

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/tools/pc_receiver_js
npm test -- --runInBand
```

Expected: 测试通过。如果该目录没有 npm test，运行：

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/tools/pc_receiver_js
node --check public/index.html
```

Expected: 如果 `node --check` 不能检查 HTML 内联脚本，记录该限制，继续执行手工浏览器验证。

- [ ] **Step 7: Commit**

```bash
git add tools/pc_receiver_js/public/index.html
git commit -m "feat: improve cross corner and aux boundary overlay"
```

---

### Task 5: 用现有回放/模拟数据验证状态字段与可视化

**Files:**
- Test only: `tools/pc_receiver_js/public/index.html`
- Test only: `project/code/driver/vision/vision_image_processor.cpp`

- [ ] **Step 1: 构建车端工程**

Run:

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/project/out
make -j12
```

Expected: 编译通过。

- [ ] **Step 2: 启动 PC receiver**

Run:

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/tools/pc_receiver_js
node server.js
```

Expected: 服务启动并打印本地访问地址，通常是 `http://localhost:3000`。

- [ ] **Step 3: 启动 mock sender**

另一个终端运行：

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/tools/pc_receiver_js
node scripts/mock_board_sender.js
```

Expected: Web 页面能收到状态，左右边界正常绘制。mock 数据未必包含真实上角点，但页面不应报错。

- [ ] **Step 4: 浏览器手工检查**

打开 receiver 页面后确认：

- 二值图上左右主边界仍为蓝/绿。
- 左辅助 trace 为亮蓝，右辅助 trace 为绿色。
- 左辅助 regular 为浅蓝，右辅助 regular 为浅绿。
- 左下角点为橙色小圆，右下角点为黄色小圆。
- 左上角点为紫色小圆，右上角点为红色小圆。
- 角点旁不再绘制文字，远路口四点贴近时仍可分辨颜色。

- [ ] **Step 5: 使用实车或录制帧验证模板命中**

在十字路口较远且下角点刚出现的帧观察状态：

```text
cross_left_aux_found / cross_right_aux_found
cross_left_aux_trace_count / cross_right_aux_trace_count
cross_left_upper_corner_found / cross_right_upper_corner_found
cross_left_upper_corner_index / cross_right_upper_corner_index
cross_left_upper_corner_point / cross_right_upper_corner_point
```

Expected:

- 下角点命中后，对应侧辅助边界能显示。
- 辅助 trace 最大点数受 `cross_aux_trace_max_points = 50` 限制。
- 辅助 trace 向上搜索行数受 `cross_aux_trace_upward_rows_max = 30` 限制。
- 上角点仅在辅助 trace 的 dir 序列满足镜像模板时 found=true。

- [ ] **Step 6: 调参记录**

如果上角点未命中，优先调小平台要求或放宽过渡长度：

```toml
cross_upper_dir4_pre_run_len = 2
cross_upper_transition_max_len = 5
cross_upper_dir6_post_run_len = 2
```

如果误命中，优先调大平台要求或缩短过渡长度：

```toml
cross_upper_dir4_pre_run_len = 4
cross_upper_transition_max_len = 2
cross_upper_dir6_post_run_len = 4
```

- [ ] **Step 7: Commit 验证记录或调参**

如果只改代码不改参数：

```bash
git status --short
```

Expected: 无未提交代码变更。

如果调了 TOML：

```bash
git add project/user/smartcar_config.toml
git commit -m "tune: adjust cross upper corner template thresholds"
```

---

## Self-Review

Spec coverage:

- “先不管十字状态机”：本计划不改 `vision_route_state_machine.*`，只改检测与 UI。
- “下角点检测时同步进行上角点检测”：使用现有下角点后立即构建辅助边界的流程，并替换 CROSS_1/CROSS_2 中上角点检测逻辑。
- “从起始行左右赛道边界起点作为起始点”：现有实现从下角点同 x 向上找白黑转变点后开始辅助边界追踪；如果后续实验要求严格改为 maze 起始行边界起点，应另开小计划调整 `build_cross_aux_boundary()` 入参语义。
- “辅助边界构建相同算法向上找白→黑转变点，再八邻域追踪向上（最多 50 点、30 行）”：现有 `build_cross_aux_boundary()` 已按 `cross_aux_vertical_scan_max_rows`、`cross_aux_trace_max_points`、`cross_aux_trace_upward_rows_max` 实现。
- “找到后在对应边的辅助边界上运行模板匹配”：Task 1-3 实现并替换调用。
- “后续再加入角点精修”：本计划不加入精修。
- “辅助边界和上角点需要在网页 UI 绘制出来”：Task 4-5 覆盖。
- “角点圆缩小，颜色代表上下左右角点，不用文字，辅助边界不同颜色”：Task 4 覆盖。

Known risk:

- 右侧上角点是否真是左侧模板的 `6 -> 4` 镜像需要实车帧验证；如果实验显示右侧也应使用 `4 -> 6`，只需把 `find_cross_upper_corner_from_aux_trace()` 中 `pre_dir/post_dir` 的镜像选择改掉。
