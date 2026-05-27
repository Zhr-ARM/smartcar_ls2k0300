# 十字下角点跳变精修 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在方向模板匹配成功后，用规则边界上的 x 跳变点精修角点位置，替代原来从 dir 序列中简单选角点的逻辑。

**Architecture:** 两阶段设计。阶段一：方向模板匹配作为初筛，匹配成功得到一个粗角点位置。阶段二（新增）：规则边界生成后，在粗角点 y 对应的规则边界位置 ±10 窗口内扫描 x 跳变——跳变判据两条（OR）：(1) x 差值的幅度突然变大；(2) x 差值的正负号翻转。**跳变判据的结果最终决定 found/not found 和角点位置**：找到跳变点 → found=true + 精修坐标；找不到跳变点 → found=false（即使模板匹配成功）。pair_valid 在精修后重新计算。

**Tech Stack:** C++ (existing vision_image_processor.cpp), existing config system (vision_config.h / vision_config.c)

---

## 涉及文件

| 文件 | 变更类型 |
|------|----------|
| `project/code/driver/vision/vision_config.h` | 修改 — 新增2个配置字段 |
| `project/code/driver/vision/vision_config.c` | 修改 — 新增2个默认值 |
| `project/code/driver/vision/vision_image_processor.cpp` | 修改 — 新增精修函数 + 调用点 |

---

## 设计细节

### 数据流变化

```
现有流程:
  update_cross_lower_corner_detection_cache (模板匹配 → found + 粗角点)
    → extract_one_point_per_row_from_contour (规则边界)
    → ...
    → truncate_boundary_at_cross_lower_corner_inplace (用粗角点截断)

新流程:
  update_cross_lower_corner_detection_cache (模板匹配 → 暂存 found + 粗角点)
    → extract_one_point_per_row_from_contour (规则边界)
    → [NEW] 跳变精修:
         ├─ 找到跳变点 → 更新角点坐标, found 保持 true
         └─ 找不到跳变点 → found = false (模板匹配结果被覆盖)
    → [NEW] 重新计算 pair_valid
    → ...
    → truncate_boundary_at_cross_lower_corner_inplace (用精修后角点截断, 或 found=false 不截)
```

### 精修函数设计

```cpp
// 在规则边界上精修角点位置
// 返回 true 表示找到跳变点并更新了 corner_x/corner_y
// 返回 false 表示未找到跳变点，保持原值
static bool refine_cross_lower_corner_on_regular_boundary(
    const maze_point_t *regular_pts,  // 规则边界点数组（底→顶）
    int regular_num,                  // 规则边界点数
    int rough_corner_y,               // 模板匹配粗角点的 y
    bool is_left,                     // 是否为左边界
    int *corner_x,                    // [in/out] 角点 x
    int *corner_y);                   // [in/out] 角点 y
```

### 判据1：x 差值幅度跳变

在窗口内扫描 forward（从底到顶，即从近车端到远车端），维护一个 running baseline（取最近若干个 dx 的均值/中位数）。当某个 dx 超过 baseline × ratio 时，该 dx 起点即为跳变点。

具体步骤：
1. 计算窗口内所有相邻点对的 dx_abs[i] = |pts[i+1].x - pts[i].x|
2. 取所有 dx_abs 的中位数作为 baseline
3. 从底向顶扫描，找到第一个满足 `dx_abs[i] > baseline * jump_ratio` 的位置 i
4. 返回 `pts[i]` 作为跳变点

### 判据2：x 差值正负号翻转

在窗口内扫描 forward，计算相邻点对的 signed dx：
- `dx[i] = pts[i+1].x - pts[i].x`
- 跳过 dx=0 的项
- 找到第一个符号翻转的位置（sign(dx[i-1]) != sign(dx[i])），翻转点即为 pts[i]

### 判据优先级

先跑判据1，找到了直接返回。找不到再跑判据2。都找不到返回 false，corner_x/y 保持不变（回退到粗角点）。

### 窗口定位

在规则边界数组中找到 y 与粗角点 y 最接近的点位置，前后各取 jump_window 个点构成窗口。窗口边界 clamp 到 [0, regular_num-1]。

---

### Task 1: 新增配置参数

**Files:**
- Modify: `project/code/driver/vision/vision_config.h:212` (在 `cross_lower_corner_extrapolate_y_span` 之后)
- Modify: `project/code/driver/vision/vision_config.c:286` (在 `cross_lower_corner_extrapolate_y_span` 默认值之后)

- [ ] **Step 1: 在 vision_config.h 的 vision_processor_runtime_config_t 结构体中新增两个字段**

在 `cross_lower_corner_extrapolate_y_span` 行后插入：

```cpp
    // 十字下角点跳变精修：规则边界搜索窗口半宽（点数）。
    int cross_lower_corner_jump_window;
    // 十字下角点跳变精修：x跳变幅度判据的倍率阈值。
    float cross_lower_corner_jump_ratio;
```

- [ ] **Step 2: 在 vision_config.c 的默认值初始化中新增两个字段**

在 `.cross_lower_corner_extrapolate_y_span = 30,` 行后插入：

```c
    // 十字下角点跳变精修：规则边界搜索窗口半宽，默认±10个点。
    .cross_lower_corner_jump_window = 10,
    // 十字下角点跳变精修：dx 超过 baseline 的倍率才认为是跳变。
    .cross_lower_corner_jump_ratio = 2.0f,
```

- [ ] **Step 3: 在 vision_image_processor.cpp 的全局 atomic 区新增运行时可调 atomic**

在 `g_cross_lower_corner_pair_y_diff_max` 的 atomic 声明后（约 line 283）新增：

```cpp
static std::atomic<int> g_cross_lower_corner_jump_window(g_vision_runtime_config.cross_lower_corner_jump_window);
static std::atomic<float> g_cross_lower_corner_jump_ratio(g_vision_runtime_config.cross_lower_corner_jump_ratio);
```

- [ ] **Step 4: 编译验证**

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/project/out && cmake ../user -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DUVC_RES_PRESET=1 -DUVC_FORMAT_PRESET=0 -DENABLE_NCNN=0 && make -j12
```

Expected: 编译通过，无错误。

- [ ] **Step 5: Commit**

```bash
git add project/code/driver/vision/vision_config.h project/code/driver/vision/vision_config.c project/code/driver/vision/vision_image_processor.cpp
git commit -m "feat: add cross lower corner jump refinement config parameters

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 2: 实现判据1 — x 差值幅度跳变检测

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp`

- [ ] **Step 1: 实现幅度跳变检测函数**

在 `pick_cross_lower_corner_index_near_transition` 函数之后（约 line 1816）新增：

```cpp
// 在规则边界窗口内用幅度跳变判据找跳变点。
// 返回 true 表示找到，corner_pt 被填充；false 表示未找到。
static bool find_boundary_jump_by_magnitude(const maze_point_t *pts, int start, int end, maze_point_t *corner_pt)
{
    if (pts == nullptr || corner_pt == nullptr || end - start < 3)
    {
        return false;
    }

    const int window_len = end - start;
    if (window_len < 3)
    {
        return false;
    }

    // 收集所有相邻 dx 绝对值，用于计算 baseline（中位数）
    std::vector<int> dx_abs;
    dx_abs.reserve(window_len - 1);
    for (int i = start; i < end - 1; ++i)
    {
        const int dx = std::abs(pts[i + 1].x - pts[i].x);
        dx_abs.push_back(dx);
    }

    if (dx_abs.empty())
    {
        return false;
    }

    // 排序取中位数作为 stable baseline
    std::sort(dx_abs.begin(), dx_abs.end());
    const int baseline = dx_abs[dx_abs.size() / 2];
    if (baseline <= 0)
    {
        return false;
    }

    const float ratio = g_cross_lower_corner_jump_ratio.load();
    const float threshold = static_cast<float>(baseline) * ratio;

    // 从底向顶扫描，找第一个超过阈值的 dx
    for (int i = start; i < end - 1; ++i)
    {
        const int dx = std::abs(pts[i + 1].x - pts[i].x);
        if (static_cast<float>(dx) > threshold)
        {
            *corner_pt = pts[i];
            return true;
        }
    }

    return false;
}
```

- [ ] **Step 2: 编译验证**

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/project/out && make -j12
```

Expected: 编译通过。

- [ ] **Step 3: Commit**

```bash
git add project/code/driver/vision/vision_image_processor.cpp
git commit -m "feat: add magnitude-based x-jump detection for corner refinement

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 3: 实现判据2 — x 差值正负号翻转检测

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp`

- [ ] **Step 1: 实现符号翻转检测函数**

在判据1函数之后新增：

```cpp
// 在规则边界窗口内用正负号翻转判据找跳变点。
// 返回 true 表示找到，corner_pt 被填充；false 表示未找到。
static bool find_boundary_jump_by_sign_flip(const maze_point_t *pts, int start, int end, maze_point_t *corner_pt)
{
    if (pts == nullptr || corner_pt == nullptr || end - start < 3)
    {
        return false;
    }

    int prev_sign = 0;
    int prev_idx = -1;

    for (int i = start; i < end - 1; ++i)
    {
        const int dx = pts[i + 1].x - pts[i].x;
        if (dx == 0)
        {
            continue;
        }
        const int sign = (dx > 0) ? 1 : -1;

        if (prev_sign != 0 && sign != prev_sign)
        {
            // 符号翻转，pts[i] 是翻转前的点
            *corner_pt = pts[i];
            return true;
        }

        prev_sign = sign;
    }

    return false;
}
```

- [ ] **Step 2: 编译验证**

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/project/out && make -j12
```

Expected: 编译通过。

- [ ] **Step 3: Commit**

```bash
git add project/code/driver/vision/vision_image_processor.cpp
git commit -m "feat: add sign-flip x-jump detection for corner refinement

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 4: 实现精修主函数 + 窗口定位

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp`

- [ ] **Step 1: 实现 `refine_cross_lower_corner_on_regular_boundary` 主函数**

在判据2函数之后新增：

```cpp
// 在规则边界上精修十字下角点位置。
// 先在粗角点 y 附近找对应规则边界点，前后各取 jump_window 个点形成窗口，
// 再依次用幅度跳变、符号翻转判据在窗口内找跳变点。
// 返回 true 表示精修成功（corner_x/corner_y 已更新），false 表示回退到粗角点。
static bool refine_cross_lower_corner_on_regular_boundary(
    const maze_point_t *regular_pts,
    int regular_num,
    int rough_corner_y,
    bool is_left,
    int *corner_x,
    int *corner_y)
{
    if (regular_pts == nullptr || regular_num < 3 || corner_x == nullptr || corner_y == nullptr)
    {
        return false;
    }

    // 1. 在规则边界中找到 y 最接近粗角点 y 的位置
    int center_idx = 0;
    int best_dy = 999999;
    for (int i = 0; i < regular_num; ++i)
    {
        const int dy = std::abs(regular_pts[i].y - rough_corner_y);
        if (dy < best_dy)
        {
            best_dy = dy;
            center_idx = i;
        }
    }

    // 2. 取 ±jump_window 窗口
    const int half_win = std::max(1, g_cross_lower_corner_jump_window.load());
    const int win_start = std::max(0, center_idx - half_win);
    const int win_end = std::min(regular_num, center_idx + half_win + 1);

    if (win_end - win_start < 3)
    {
        return false;
    }

    // 3. 优先判据1（幅度跳变），再判据2（符号翻转）
    maze_point_t jump_pt{0, 0};
    if (find_boundary_jump_by_magnitude(regular_pts, win_start, win_end, &jump_pt) ||
        find_boundary_jump_by_sign_flip(regular_pts, win_start, win_end, &jump_pt))
    {
        *corner_x = jump_pt.x;
        *corner_y = jump_pt.y;
        return true;
    }

    return false;
}
```

- [ ] **Step 2: 编译验证**

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/project/out && make -j12
```

Expected: 编译通过。

- [ ] **Step 3: Commit**

```bash
git add project/code/driver/vision/vision_image_processor.cpp
git commit -m "feat: add refine_cross_lower_corner_on_regular_boundary main function

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 5: 接入精修调用点

**Files:**
- Modify: `project/code/driver/vision/vision_image_processor.cpp:4489-4496`

- [ ] **Step 1: 在规则边界生成后、截断前，插入跳变精修 + found 覆写 + pair_valid 重算**

现有代码（约 line 4489-4496）：

```cpp
        left_num = copy_boundary_points(left_regular_pts.data(),
                                        left_regular_num,
                                        left_pts.data(),
                                        static_cast<int>(left_pts.size()));
        right_num = copy_boundary_points(right_regular_pts.data(),
                                         right_regular_num,
                                         right_pts.data(),
                                         static_cast<int>(right_pts.size()));
```

替换为：

```cpp
        left_num = copy_boundary_points(left_regular_pts.data(),
                                        left_regular_num,
                                        left_pts.data(),
                                        static_cast<int>(left_pts.size()));
        right_num = copy_boundary_points(right_regular_pts.data(),
                                         right_regular_num,
                                         right_pts.data(),
                                         static_cast<int>(right_pts.size()));

        // 十字下角点跳变精修：模板匹配仅初筛，跳变判据最终决定 found 和角点位置
        bool left_jump_refined = false;
        bool right_jump_refined = false;
        if (g_cross_lower_left_corner_found.load())
        {
            int refined_x = g_cross_lower_left_corner_x.load();
            int refined_y = g_cross_lower_left_corner_y.load();
            if (refine_cross_lower_corner_on_regular_boundary(left_regular_pts.data(),
                                                               left_regular_num,
                                                               refined_y,
                                                               true,
                                                               &refined_x,
                                                               &refined_y))
            {
                g_cross_lower_left_corner_x.store(refined_x);
                g_cross_lower_left_corner_y.store(refined_y);
                left_jump_refined = true;
            }
            else
            {
                // 跳变判据未找到 → 覆盖模板匹配结果，置 found=false
                g_cross_lower_left_corner_found.store(false);
            }
        }
        if (g_cross_lower_right_corner_found.load())
        {
            int refined_x = g_cross_lower_right_corner_x.load();
            int refined_y = g_cross_lower_right_corner_y.load();
            if (refine_cross_lower_corner_on_regular_boundary(right_regular_pts.data(),
                                                               right_regular_num,
                                                               refined_y,
                                                               false,
                                                               &refined_x,
                                                               &refined_y))
            {
                g_cross_lower_right_corner_x.store(refined_x);
                g_cross_lower_right_corner_y.store(refined_y);
                right_jump_refined = true;
            }
            else
            {
                g_cross_lower_right_corner_found.store(false);
            }
        }

        // 重新计算 pair_valid：必须两边跳变精修都成功 + y 差在阈值内
        {
            const int y_diff_max = std::max(0, g_cross_lower_corner_pair_y_diff_max.load());
            const bool pair_valid =
                left_jump_refined &&
                right_jump_refined &&
                std::abs(g_cross_lower_left_corner_y.load() - g_cross_lower_right_corner_y.load()) <= y_diff_max;
            g_cross_lower_corner_pair_valid.store(pair_valid);
        }
```

注意：`left_regular_pts` 和 `right_regular_pts` 在 line 4477-4488 的 `extract_one_point_per_row_from_contour` 调用中已经填充完毕，所以这里可以直接使用。`pair_valid` 原本在 `update_cross_lower_corner_detection_cache` 内基于粗角点计算，此处用精修后的坐标重新计算并覆盖。

- [ ] **Step 2: 编译验证**

```bash
cd /home/lucy/loongcar/smartcar_ls2k0300/project/out && make -j12
```

Expected: 编译通过。

- [ ] **Step 3: Commit**

```bash
git add project/code/driver/vision/vision_image_processor.cpp
git commit -m "feat: wire cross lower corner jump refinement into processing pipeline

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## 自检清单

### 1. Spec coverage

| 需求 | 对应 Task | 状态 |
|------|-----------|------|
| 模板匹配作为初筛 | Task 5 — `g_cross_lower_*_corner_found` 来自 `update_...cache`，作为进入精修的前置条件 | OK |
| 跳变判据决定最终 found | Task 5 — 精修失败时 `store(false)` 覆盖模板匹配的 found | OK |
| 跳变判据决定最终角点位置 | Task 5 — 精修成功时 `store(refined_x/y)` 覆盖粗角点 | OK |
| pair_valid 基于精修后坐标重算 | Task 5 — 在精修块末尾重新计算并 store | OK |
| 在规则边界中找粗角点对应位置 | Task 4 — `center_idx` 通过最小 dy 定位 | OK |
| ±10 点窗口搜索 | Task 4 — `half_win` 从配置读取，默认 10 | OK |
| 判据1：x差值突然变大 | Task 2 — baseline 中位数 × ratio 阈值 | OK |
| 判据2：差值正负号翻转 | Task 3 — sign(dx) 翻转检测 | OK |
| 优先判据1，再判据2，OR 关系 | Task 4 — `||` 短路 | OK |
| 配置可调 | Task 1 — jump_window + jump_ratio | OK |

### 2. Placeholder scan

无 TBD、TODO、placeholder。

### 3. Type consistency

- `maze_point_t` — 现有类型，{int x, int y}，全链路一致使用
- `cross_lower_corner_detection_t` — 现有类型，不变
- 新函数签名中 `maze_point_t*` / `int*` 类型在声明和调用处一致
- atomic `g_cross_lower_corner_jump_window` (int) / `g_cross_lower_corner_jump_ratio` (float) 在声明、默认值和 load 处类型一致
