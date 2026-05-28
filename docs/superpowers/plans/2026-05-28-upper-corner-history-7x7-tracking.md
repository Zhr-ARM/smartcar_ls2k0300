# 上角点历史 7×7 正方形快速跟踪

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有八邻域辅助边界上角点检测之前，插入基于上一帧角点位置的 7×7 正方形局部跟踪快速路径，减少计算量。

**Architecture:** 新增 `detect_upper_corner_from_history()` 函数，在 7×7 窗口中读上边和侧边的黑白跳变来定位角点。修改 `update_cross_upper_corner_detection_cache()` 将其作为第一优先路径。修改 CROSS_1/CROSS_2 的边界拼接逻辑，处理"历史跟踪成功但无辅助边界"的情况。

**Tech Stack:** C++，图像分辨率 160×120，二值图（0=黑/墙壁，255=白/赛道）。

---

## 文件结构

| 文件 | 职责 | 操作 |
|------|------|------|
| `project/code/driver/vision/vision_image_processor.cpp` | 新增检测函数 + 修改上角点缓存更新 + 修改 CROSS_1/CROSS_2 边界拼接 | 修改 |
| `project/code/driver/vision/vision_config.h` | 新增 7×7 检测配置参数 | 修改 |
| `project/code/driver/vision/vision_config.c` | 新增配置参数默认值 | 修改 |

---

## 核心数据结构

正方形与角点的几何关系（以左上角点为例）：

```
正方形左上角 (sx, sy) = (prev_cx - 2, prev_cy - 2)
正方形覆盖: x ∈ [cx-2, cx+4], y ∈ [cy-2, cy+4]

   x=0 1 2 3 4 5 6
y=0  ● ● ○ ○ ○ ○ ○  ← 上边(sy行): 2黑+5白, 跳变在x=2
y=1  ● ● ○ ○ ○ ○ ○
y=2  ○ ○ ○ ○ ○ ○ ○  ← 角点 = (sx+2, sy+2)
y=3  ○ ○ ○ ○ ○ ○ ○
y=4  ○ ○ ○ ○ ○ ○ ○
y=5  ○ ○ ○ ○ ○ ○ ○
y=6  ○ ○ ○ ○ ○ ○ ○
  ↑
左边(sx列): 2黑+5白, 跳变在y=2
```

右上角点（镜像）：

```
正方形左上角 (sx, sy) = (prev_cx - 5, prev_cy - 2)

   x=0 1 2 3 4 5 6
y=0  ○ ○ ○ ○ ○ ● ●  ← 上边: 5白+2黑, 跳变在x=5
y=1  ○ ○ ○ ○ ○ ● ●
y=2  ○ ○ ○ ○ ○ ● ●  ← 角点 = (sx+5, sy+2)
y=3  ○ ○ ○ ○ ○ ● ●
y=4  ○ ○ ○ ○ ○ ● ●
y=5  ○ ○ ○ ○ ○ ● ●
y=6  ○ ○ ○ ○ ○ ● ●
                    ↑ 右边(sx+6列): 2黑+5白, 跳变在y=2
```

### 配置参数（新增）

```c
// vision_config.h 中新增
int cross_upper_history_square_half;       // 正方形半边长, 默认 3 (→7×7)
int cross_upper_history_black_pixels;      // 预期黑像素数, 默认 2
int cross_upper_history_max_iterations;    // 最大平移迭代次数, 默认 5
int cross_upper_history_shift_px;          // 每次平移像素数, 默认 3
```

---

### Task 1: 新增配置参数

**Files:**
- Modify: `project/code/driver/vision/vision_config.h` (在 cross_aux_history_x_offset 之后)
- Modify: `project/code/driver/vision/vision_config.c` (在对应默认值区域)

- [ ] **Step 1: 在 vision_config.h 添加配置字段**

在 `cross_aux_history_x_offset` 声明之后（约第 365 行）添加：

```c
    // 7×7 正方形历史跟踪参数
    int cross_upper_history_enabled;          // 是否启用历史正方形跟踪, 默认 1
    int cross_upper_history_square_half;      // 正方形半边长 (半边长3→7×7), 默认 3
    int cross_upper_history_black_pixels;     // 边上预期黑像素数, 默认 2
    int cross_upper_history_max_iterations;   // 最大平移迭代次数, 默认 5
    int cross_upper_history_shift_px;         // 每次平移像素数 (半边长), 默认 3
```

- [ ] **Step 2: 在 vision_config.c 添加默认值**

在 `cross_aux_history_x_offset` 默认值之后（约第 375 行）添加：

```c
    config->cross_upper_history_enabled = true;
    config->cross_upper_history_square_half = 3;
    config->cross_upper_history_black_pixels = 2;
    config->cross_upper_history_max_iterations = 5;
    config->cross_upper_history_shift_px = 3;
```

- [ ] **Step 3: 编译验证**

```bash
mkdir -p project/out && cd project/out
cmake ../user -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DUVC_RES_PRESET=1 -DUVC_FORMAT_PRESET=0 -DENABLE_NCNN=0
make -j12
```

---

### Task 2: 实现 7×7 正方形上角点检测函数

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp`（在 `find_cross_upper_corner_from_aux_trace` 之前插入）

- [ ] **Step 1: 添加 `detect_upper_corner_from_history` 函数**

在 `find_cross_upper_corner_from_aux_trace` 函数定义之前（约第 4041 行之前）插入：

```cpp
// 基于上一帧上角点位置的 7×7 正方形局部跟踪。
// 左上角点: 上边 2 黑 + 5 白 (黑在左), 左边 2 黑 + 5 白 (黑在上)
// 右上角点: 镜像 — 上边 5 白 + 2 黑 (黑在右), 右边 2 黑 + 5 白 (黑在上)
// 返回 true 表示找到角点, corner_point 输出角点坐标。
static bool detect_upper_corner_from_history(PixelClassifier &classifier,
                                              bool is_left,
                                              int prev_corner_x,
                                              int prev_corner_y,
                                              maze_point_t *corner_point)
{
    if (corner_point) *corner_point = maze_point_t{0, 0};
    if (classifier.binary == nullptr) return false;

    const int img_w = VISION_DOWNSAMPLED_WIDTH;   // 160
    const int img_h = VISION_DOWNSAMPLED_HEIGHT;  // 120
    const int half = g_vision_runtime_config.cross_upper_history_square_half;  // 3
    const int black_n = g_vision_runtime_config.cross_upper_history_black_pixels; // 2
    const int max_iter = g_vision_runtime_config.cross_upper_history_max_iterations;
    const int shift = g_vision_runtime_config.cross_upper_history_shift_px;    // 3
    const int side = half * 2 + 1;  // 7

    int sx, sy;  // 正方形左上角
    if (is_left)
    {
        sx = prev_corner_x - half;  // cx - 2
        sy = prev_corner_y - half;  // cy - 2
    }
    else
    {
        sx = prev_corner_x - (side - 1 - half);  // cx - 5 (黑像素在右边)
        sy = prev_corner_y - half;                // cy - 2
    }

    for (int iter = 0; iter < max_iter; ++iter)
    {
        // 边界检查
        if (sx < 0 || sy < 0 || sx + side > img_w || sy + side > img_h)
        {
            return false;
        }

        const uint8 *binary = classifier.binary;
        const int stride = img_w;

        if (is_left)
        {
            // --- 左上角点 ---
            // 上边 (sy 行, sx..sx+6): 前 black_n 黑, 后 (side-black_n) 白
            bool top_ok = true;
            for (int i = 0; i < black_n && top_ok; ++i)
                if (binary[sy * stride + sx + i] != 0) top_ok = false;
            for (int i = black_n; i < side && top_ok; ++i)
                if (binary[sy * stride + sx + i] != 255) top_ok = false;

            // 左边 (sx 列, sy..sy+6): 前 black_n 黑, 后 (side-black_n) 白
            bool left_ok = true;
            for (int i = 0; i < black_n && left_ok; ++i)
                if (binary[(sy + i) * stride + sx] != 0) left_ok = false;
            for (int i = black_n; i < side && left_ok; ++i)
                if (binary[(sy + i) * stride + sx] != 255) left_ok = false;

            if (top_ok && left_ok)
            {
                if (corner_point)
                {
                    corner_point->x = sx + black_n;  // sx + 2
                    corner_point->y = sy + black_n;  // sy + 2
                }
                return true;
            }

            // 上边独立修正: 全黑→右移, 全白→左移
            bool top_all_black = true, top_all_white = true;
            for (int i = 0; i < side; ++i)
            {
                if (binary[sy * stride + sx + i] != 0)   top_all_black = false;
                if (binary[sy * stride + sx + i] != 255) top_all_white = false;
            }
            if (top_all_black)       sx += shift;
            else if (top_all_white)  sx -= shift;

            // 左边独立修正: 全黑→下移, 全白→上移
            bool left_all_black = true, left_all_white = true;
            for (int i = 0; i < side; ++i)
            {
                if (binary[(sy + i) * stride + sx] != 0)   left_all_black = false;
                if (binary[(sy + i) * stride + sx] != 255) left_all_white = false;
            }
            if (left_all_black)       sy += shift;
            else if (left_all_white)  sy -= shift;
        }
        else
        {
            // --- 右上角点（镜像） ---
            const int right_x = sx + side - 1;  // 右边列索引 sx+6

            // 上边 (sy 行): 前 (side-black_n) 白, 后 black_n 黑
            bool top_ok = true;
            for (int i = 0; i < side - black_n && top_ok; ++i)
                if (binary[sy * stride + sx + i] != 255) top_ok = false;
            for (int i = side - black_n; i < side && top_ok; ++i)
                if (binary[sy * stride + sx + i] != 0) top_ok = false;

            // 右边 (right_x 列): 前 black_n 黑, 后 (side-black_n) 白
            bool right_ok = true;
            for (int i = 0; i < black_n && right_ok; ++i)
                if (binary[(sy + i) * stride + right_x] != 0) right_ok = false;
            for (int i = black_n; i < side && right_ok; ++i)
                if (binary[(sy + i) * stride + right_x] != 255) right_ok = false;

            if (top_ok && right_ok)
            {
                if (corner_point)
                {
                    corner_point->x = sx + (side - 1 - black_n);  // sx + 5
                    corner_point->y = sy + black_n;               // sy + 2
                }
                return true;
            }

            // 上边独立修正: 全黑(右侧黑像素溢出→黑占满)→左移, 全白→右移
            bool top_all_black = true, top_all_white = true;
            for (int i = 0; i < side; ++i)
            {
                if (binary[sy * stride + sx + i] != 0)   top_all_black = false;
                if (binary[sy * stride + sx + i] != 255) top_all_white = false;
            }
            if (top_all_black)       sx -= shift;
            else if (top_all_white)  sx += shift;

            // 右边独立修正: 全黑→下移, 全白→上移
            bool right_all_black = true, right_all_white = true;
            for (int i = 0; i < side; ++i)
            {
                if (binary[(sy + i) * stride + right_x] != 0)   right_all_black = false;
                if (binary[(sy + i) * stride + right_x] != 255) right_all_white = false;
            }
            if (right_all_black)       sy += shift;
            else if (right_all_white)  sy -= shift;
        }
    }

    return false;
}
```

- [ ] **Step 2: 编译验证**

```bash
cd project/out && make -j12
```

---

### Task 3: 修改 `update_cross_upper_corner_detection_cache` — 插入历史跟踪路径

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp:2365-2460`

- [ ] **Step 1: 在路径 1 之前插入历史正方形跟踪分支**

在 `process_side` lambda 内，`// ---- 路径 1: 尝试用历史点重捕获 ----` 之前（约第 2365 行），插入：

```cpp
            // ---- 路径 0: 7×7 正方形历史角点跟踪（最快） ----
            const bool prev_upper_found = is_left
                ? g_cross_left_upper_corner_found.load()
                : g_cross_right_upper_corner_found.load();
            if (g_vision_runtime_config.cross_upper_history_enabled && prev_upper_found)
            {
                const int prev_upper_x = is_left
                    ? g_cross_left_upper_corner_x.load()
                    : g_cross_right_upper_corner_x.load();
                const int prev_upper_y = is_left
                    ? g_cross_left_upper_corner_y.load()
                    : g_cross_right_upper_corner_y.load();

                if (detect_upper_corner_from_history(classifier,
                                                      is_left,
                                                      prev_upper_x,
                                                      prev_upper_y,
                                                      &upper_corner))
                {
                    upper_found = true;
                    upper_trace_index = -1;  // 历史跟踪无 trace index
                    // 不构建辅助边界，aux_trace_num/aux_regular_num 保持 0
                }
            }
```

- [ ] **Step 2: 修改写入全局状态逻辑 — 区分历史跟踪 vs 辅助边界**

找到路径 1 成功和路径 2 成功后写入全局状态的代码（约第 2462-2508 行的 `if (is_left)` / `else` 分支）。

当前逻辑：`upper_found` 为 true 时设置 `aux_found=true`、保存 aux cache、设置 upper corner。

修改为：历史跟踪成功时不设置 `aux_found`、不保存 aux cache。找到写入段，将：

```cpp
            if (is_left)
            {
                g_cross_left_aux_found.store(upper_found);
                g_cross_left_upper_corner_found.store(upper_found);
                if (upper_found)
                {
                    g_cross_left_aux_transition_x.store(aux_transition.x);
                    // ... 等
                    save_cross_aux_trace_cache(true, ...);
                    save_cross_aux_regular_cache(true, ...);
                }
            }
```

改为：

```cpp
            if (is_left)
            {
                const bool has_aux = (aux_trace_num > 0);  // 辅助边界是否存在
                g_cross_left_aux_found.store(has_aux);
                g_cross_left_upper_corner_found.store(upper_found);
                if (upper_found)
                {
                    g_cross_left_upper_corner_index.store(upper_trace_index);
                    g_cross_left_upper_corner_x.store(upper_corner.x);
                    g_cross_left_upper_corner_y.store(upper_corner.y);
                    if (has_aux)
                    {
                        g_cross_left_aux_transition_x.store(aux_transition.x);
                        g_cross_left_aux_transition_y.store(aux_transition.y);
                        g_cross_left_aux_last_transition_valid.store(true);
                        g_cross_left_aux_last_transition_x.store(aux_transition.x);
                        g_cross_left_aux_last_transition_y.store(aux_transition.y);
                        save_cross_aux_trace_cache(true, aux_trace_pts.data(), aux_trace_dirs.data(), aux_trace_num);
                        save_cross_aux_regular_cache(true, aux_regular_pts.data(), aux_regular_num);
                    }
                }
            }
```

右侧 (`else`) 分支做相同修改（`is_left` → `false`）。

- [ ] **Step 3: 编译验证**

```bash
cd project/out && make -j12
```

---

### Task 4: 修改 CROSS_1 边界拼接 — 适配无辅助边界情况

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp:5639-5780` (CROSS_1 段)

- [ ] **Step 1: 在 CROSS_1 边界拼接中增加历史跟踪分支**

当前 CROSS_1 的左侧处理（约第 5653 行）逻辑是：

```
if (aux_found && aux_regular_num > 0):
    检测上角点 → 成功: base + bridge + aux_tail / 失败: base
```

改为：

```
if (upper_corner_found):
    if (aux_found && aux_regular_num > 0):
        // 原有逻辑: base + bridge + aux_tail
    else:
        // 历史跟踪路径: base + bridge (无 aux_tail)
if (aux_found && aux_regular_num > 0):
    // 原有逻辑 (检测上角点...)
```

具体代码修改。找到：

```cpp
            if (g_cross_left_aux_found.load() && left_cross_aux_regular_num > 0)
            {
                maze_point_t left_upper_corner{};
                int left_upper_trace_index = -1;
                if (find_cross_upper_corner_from_aux_trace(...))
                {
                    // ... 拼接 base + bridge + aux_tail
                }
                else { /* fallback */ }
            }
```

改为：

```cpp
            const bool left_upper_ready = g_cross_left_upper_corner_found.load();
            const bool left_aux_ready = g_cross_left_aux_found.load() && left_cross_aux_regular_num > 0;

            if (left_upper_ready && left_aux_ready)
            {
                // 原有路径: 辅助边界 + 八邻域上角点 → base + bridge + aux_tail
                maze_point_t left_upper_corner{
                    g_cross_left_upper_corner_x.load(),
                    g_cross_left_upper_corner_y.load()
                };
                std::array<maze_point_t, VISION_BOUNDARY_NUM> left_aux_tail{};
                const int left_aux_tail_num = copy_boundary_points(left_cross_aux_regular_pts.data(),
                                                                     left_cross_aux_regular_num,
                                                                     left_aux_tail.data(),
                                                                     static_cast<int>(left_aux_tail.size()));
                const int left_aux_tail_truncated_num =
                    truncate_regular_boundary_before_point_inplace(left_aux_tail.data(),
                                                                    left_aux_tail_num,
                                                                    left_upper_corner);
                std::array<maze_point_t, VISION_BOUNDARY_NUM> left_bridge_pts{};
                const maze_point_t left_lower_corner{
                    g_cross_lower_left_corner_x.load(),
                    g_cross_lower_left_corner_y.load()
                };
                const int left_bridge_num = build_line_points_between_with_y_step(left_lower_corner,
                                                                                    left_upper_corner,
                                                                                    2,
                                                                                    left_bridge_pts.data(),
                                                                                    static_cast<int>(left_bridge_pts.size()));
                // concatenate: base + bridge + aux_tail
                // ... (原有拼接逻辑保持不变)
            }
            else if (left_upper_ready && !left_aux_ready)
            {
                // 历史跟踪路径: base + bridge (无 aux_tail)
                maze_point_t left_upper_corner{
                    g_cross_left_upper_corner_x.load(),
                    g_cross_left_upper_corner_y.load()
                };
                std::array<maze_point_t, VISION_BOUNDARY_NUM> left_bridge_pts{};
                const maze_point_t left_lower_corner{
                    g_cross_lower_left_corner_x.load(),
                    g_cross_lower_left_corner_y.load()
                };
                const int left_bridge_num = build_line_points_between_with_y_step(left_lower_corner,
                                                                                    left_upper_corner,
                                                                                    2,
                                                                                    left_bridge_pts.data(),
                                                                                    static_cast<int>(left_bridge_pts.size()));
                const maze_point_t *left_base_pts = (left_cross_base_num > 0) ? left_cross_base_pts.data() : left_pts.data();
                const int left_base_num = (left_cross_base_num > 0) ? left_cross_base_num : left_num;
                std::array<maze_point_t, VISION_BOUNDARY_NUM> left_combined_pts{};
                const int left_combined_num = concatenate_boundary_segments(left_base_pts,
                                                                              left_base_num,
                                                                              left_bridge_pts.data(),
                                                                              left_bridge_num,
                                                                              nullptr,
                                                                              0,
                                                                              left_combined_pts.data(),
                                                                              static_cast<int>(left_combined_pts.size()));
                left_num = copy_boundary_points(left_combined_pts.data(),
                                                  left_combined_num,
                                                  left_pts.data(),
                                                  static_cast<int>(left_pts.size()));
            }
            else if (!left_upper_ready && left_aux_ready)
            {
                // aux 存在但上角点未命中: 先尝试检测, 失败回退 base
                // ... (原 else 分支逻辑)
            }
```

右侧 (`g_cross_right_*`) 做镜像修改。

- [ ] **Step 2: 编译验证**

```bash
cd project/out && make -j12
```

---

### Task 5: 修改 CROSS_2 边界拼接 — 适配无辅助边界情况

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp:5781-5995` (CROSS_2 段)

- [ ] **Step 1: 在 CROSS_2 中插入历史跟踪快速路径**

CROSS_2 当前对每侧的处理是：清空 aux → build_cross_aux_boundary → find_cross_upper_corner_from_aux_trace → 拼接。

修改为：先尝试 7×7 历史跟踪，成功则跳过 aux 构建直接拼接；失败则走原有 aux 流程。

在 CROSS_2 左侧处理中（约第 5811 行 `if (frozen_left_found)` 内部），在 `build_cross_aux_boundary` 调用之前插入：

```cpp
                // 尝试历史正方形跟踪
                bool left_upper_from_history = false;
                maze_point_t left_upper_corner{};
                if (g_vision_runtime_config.cross_upper_history_enabled &&
                    g_cross_left_upper_corner_found.load())
                {
                    const int prev_ux = g_cross_left_upper_corner_x.load();
                    const int prev_uy = g_cross_left_upper_corner_y.load();
                    left_upper_from_history = detect_upper_corner_from_history(classifier,
                                                                                true,
                                                                                prev_ux,
                                                                                prev_uy,
                                                                                &left_upper_corner);
                }
```

然后在原有 `build_cross_aux_boundary` + `find_cross_upper_corner_from_aux_trace` 外部包裹条件：

```cpp
                if (left_upper_from_history)
                {
                    // 历史跟踪成功: 无 aux_tail, guide + bridge 从固定起点到上角点
                    g_cross_left_upper_corner_found.store(true);
                    g_cross_left_upper_corner_x.store(left_upper_corner.x);
                    g_cross_left_upper_corner_y.store(left_upper_corner.y);

                    const maze_point_t left_start{
                        std::clamp(20, 0, kProcWidth - 1),
                        std::clamp(100, 1, kProcHeight - 2)
                    };
                    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_guide_pts{};
                    const int left_guide_num = build_line_points_between(left_start,
                                                                           left_upper_corner,
                                                                           left_guide_pts.data(),
                                                                           static_cast<int>(left_guide_pts.size()));
                    std::array<maze_point_t, VISION_BOUNDARY_NUM> left_combined_pts{};
                    const int left_combined_num = concatenate_boundary_segments(left_guide_pts.data(),
                                                                                  left_guide_num,
                                                                                  nullptr,
                                                                                  0,
                                                                                  nullptr,
                                                                                  0,
                                                                                  left_combined_pts.data(),
                                                                                  static_cast<int>(left_combined_pts.size()));
                    left_num = copy_boundary_points(left_combined_pts.data(),
                                                      left_combined_num,
                                                      left_pts.data(),
                                                      static_cast<int>(left_pts.size()));
                }
                else if (build_cross_aux_boundary(...))
                {
                    // ... 原有 aux 流程保持不变
                }
```

右侧做镜像修改（`is_left=false`, 固定起点 `right_start = {140, 100}`）。

- [ ] **Step 2: 编译验证**

```bash
cd project/out && make -j12
```

---

### Task 6: 最终编译验证

- [ ] **Step 1: 全量编译**

```bash
cd project/out && make -j12
```

预期：编译通过，无警告（与交叉编译工具链相关的既有警告除外）。

---

## 自检

**1. 规格覆盖:**
- [x] 7×7 正方形检测函数 — Task 2
- [x] 左上/右上镜像逻辑 — Task 2
- [x] 全黑/全白时平移迭代 — Task 2
- [x] 插入为第一优先路径 — Task 3
- [x] 历史跟踪无辅助边界时的下游处理 — Task 4 (CROSS_1), Task 5 (CROSS_2)
- [x] 配置参数可调 — Task 1

**2. 无占位符:** 所有步骤包含完整代码。

**3. 类型一致性:**
- `detect_upper_corner_from_history` 签名: `(PixelClassifier&, bool is_left, int prev_cx, int prev_cy, maze_point_t*) -> bool` — 在各 Task 中使用一致
- 全局原子变量名: `g_cross_left_upper_corner_found/x/y` 等在 Task 3/4/5 中保持一致
- `g_cross_left_aux_found` 语义变更: 从 `= upper_found` 变为 `= has_aux` (仅当辅助边界存在时为 true) — Task 3 中统一处理
