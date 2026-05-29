# 角点补线（替代路口状态机）实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在边界截断后，用上下角点做几何补线替代已删除的路口状态机补线逻辑，确保 IPM 拿到完整边界。

**Architecture:** 新增一个静态辅助函数 `complete_boundary_with_corners`，对每侧独立判断：上下角点都有→连线插值；只有上角点→竖直向下补到底部；只有下角点→不处理。调用点位于状态机更新后、环岛处理前，直接覆写 `left_pts`/`right_pts` 和对应的 count 变量。

**Tech Stack:** C++，`std::array<maze_point_t, VISION_BOUNDARY_NUM>`、`std::atomic` 全局变量

---

## 文件结构

| 文件 | 职责 |
|------|------|
| `project/code/driver/vision/vision_image_processor.cpp` | 唯一修改文件：新增 `complete_boundary_with_corners` 函数 + 调用点 |

---

### Task 1: 新增 `complete_boundary_with_corners` 辅助函数

**文件:** [vision_image_processor.cpp](project/code/driver/vision/vision_image_processor.cpp)

在 `truncate_boundary_at_cross_lower_corner_inplace` 之后（约第 4439 行后）、`copy_boundary_points` 之前插入新函数。

- [ ] **Step 1: 添加函数实现**

```cpp
// 角点补线：用上下角点做几何补线，替代已删除的路口状态机补线。
// 输出数组按 y 从大到小排列（画面底部在前，索引 0 = 离车最近）。
// 规则：
//   1. 同时有上角点和下角点 → 两点连线，逐行插值
//   2. 仅有上角点           → 从上角点竖直向下补到画面底部
//   3. 仅有下角点或无角点   → 保持原边界不变
static int complete_boundary_with_corners(maze_point_t *pts,
                                          int num,
                                          bool upper_found, int upper_x, int upper_y,
                                          bool lower_found, int lower_x, int lower_y,
                                          int max_pts)
{
    if (pts == nullptr || max_pts <= 0)
    {
        return 0;
    }

    // 1. 同时有上角点和下角点：连线补线（下角点→上角点，底部优先）
    if (upper_found && lower_found)
    {
        const maze_point_t lower_pt = {lower_x, lower_y};
        const maze_point_t upper_pt = {upper_x, upper_y};
        return build_line_points_between(lower_pt, upper_pt, pts, max_pts);
    }

    // 2. 仅有上角点：从上角点竖直向下补到画面底部
    if (upper_found && !lower_found)
    {
        const int start_y = kProcHeight - 2;  // 画面底部（跳过边缘行）
        const int end_y = std::max(1, upper_y);
        int out_num = 0;
        for (int y = start_y; y >= end_y && out_num < max_pts; --y)
        {
            pts[out_num++] = {std::clamp(upper_x, 0, kProcWidth - 1), y};
        }
        return out_num;
    }

    // 3. 仅有下角点或无角点：保持原样
    return num;
}
```

- [ ] **Step 2: 编译验证函数本身无语法错误**

```bash
mkdir -p project/out && cd project/out && cmake ../user -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DUVC_RES_PRESET=1 -DUVC_FORMAT_PRESET=0 -DENABLE_NCNN=0 && make -j12 2>&1 | tail -30
```

期望：编译通过（此时函数尚未被调用，可能有 unused warning，可忽略）。

---

### Task 2: 在 NORMAL/STRAIGHT 截断后调用补线

**文件:** [vision_image_processor.cpp](project/code/driver/vision/vision_image_processor.cpp)

调用点位置：NORMAL/STRAIGHT 截断块（约第 5347 行 `}` 闭合后）、`auto t_maze_trace_end` 之后、`vision_route_state_input_t route_input{}` 之前。

具体在约第 5348 行 `auto t_maze_trace_end = ...` 之后插入。

- [ ] **Step 1: 添加调用代码**

```cpp
    auto t_maze_trace_end = std::chrono::steady_clock::now();

    // 角点补线：在边界截断后，用上下角点做几何补线替代原路口状态机补线。
    // 对每侧独立判断：上下角点都有→连线；仅上角点→竖直向下；仅下角点→不动。
    left_num = complete_boundary_with_corners(left_pts.data(),
                                               left_num,
                                               g_cross_left_upper_corner_found.load(),
                                               g_cross_left_upper_corner_x.load(),
                                               g_cross_left_upper_corner_y.load(),
                                               g_cross_lower_left_corner_found.load(),
                                               g_cross_lower_left_corner_x.load(),
                                               g_cross_lower_left_corner_y.load(),
                                               static_cast<int>(left_pts.size()));
    right_num = complete_boundary_with_corners(right_pts.data(),
                                                right_num,
                                                g_cross_right_upper_corner_found.load(),
                                                g_cross_right_upper_corner_x.load(),
                                                g_cross_right_upper_corner_y.load(),
                                                g_cross_lower_right_corner_found.load(),
                                                g_cross_lower_right_corner_x.load(),
                                                g_cross_lower_right_corner_y.load(),
                                                static_cast<int>(right_pts.size()));

    vision_route_state_input_t route_input{};
```

- [ ] **Step 2: 编译验证**

```bash
cd project/out && make -j12 2>&1 | tail -20
```

期望：编译通过，无 warning（函数被调用后 unused warning 消失）。

---

### Task 3: 全量编译 + 产物检查

- [ ] **Step 1: 清空重新编译**

```bash
rm -rf project/out && mkdir -p project/out && cd project/out && cmake ../user -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DUVC_RES_PRESET=1 -DUVC_FORMAT_PRESET=0 -DENABLE_NCNN=0 && make -j12 2>&1 | tail -20
```

期望：编译通过。

- [ ] **Step 2: 验证产物**

```bash
file project/out/project && ls -lh project/out/project
```

期望：`ELF 64-bit LSB executable, LoongArch`。

---

### Task 4: 审视边界情况

检查以下场景的补线行为是否正确：

- [ ] **场景 A：上下角点都有** — `build_line_points_between(lower, upper, ...)` 接收下角点（y 较大）和上角点（y 较小），函数内部 `for (int y = max_y; y >= min_y; --y)` 从下往上逐行插值，输出顺序为 bottom-to-top，与 `fill_boundary_arrays_from_maze` 的消费顺序一致。

- [ ] **场景 B：仅上角点** — 竖直补线从 `kProcHeight - 2`（画面底部）到 `upper_y`（上角点 y），x 不变。输出 index 0 = 底部点 = 边界起始点，符合"最下方的点才是边界的起始点"的要求。

- [ ] **场景 C：仅下角点** — `return num`，不修改原边界，符合"无操作"的要求。

- [ ] **场景 D：无角点** — `return num`，不修改。

- [ ] **场景 E：角点 x 在图像边界外** — `std::clamp(upper_x, 0, kProcWidth - 1)` 确保不越界。

- [ ] **场景 F：上下角点 y 相等** — `build_line_points_between` 中 `a.y == b.y` 时 `t = 0.0f`，生成单个点，不会除以零。
