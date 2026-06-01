# 简化位置环 — 删除 IIR 滤波、死区、动态 Kp Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 位置环输入直接用原始像素误差 × 固定 Kp，删除 IIR 低通滤波、死区+小误差降增益、三段分段动态 Kp 及其所有相关参数。

**Architecture:** 在 line_follow_thread.cpp 中删除 `g_filtered_error`、`compute_control_error_state()`、`compute_piecewise_linear_abs_gain()`，位置环 PID 直接吃 `raw_error_px`。Profile 结构体用单一 `position_kp` 替换 8 个动态 Kp 字段。toml 参数同步精简。

**Tech Stack:** C++ (LoongArch 交叉编译), TOML

---

### Task 1: 简化 line_follow_thread.cpp 位置环逻辑

**Files:**
- Modify: `project/code/app/line_follow_thread/line_follow_thread.cpp`

删除三个处理函数，位置环直接用原始误差。

- [ ] **Step 1: 删除 `compute_piecewise_linear_abs_gain` 函数**

找到 `float compute_piecewise_linear_abs_gain(...)` 及其完整函数体（约 179-201 行），整块删除。

- [ ] **Step 2: 删除 `compute_control_error_state` 函数**

找到 `ControlErrorState compute_control_error_state(float filtered_error_px)` 及其完整函数体（约 204-219 行），整块删除。同时删除 `ControlErrorState` 结构体定义。

- [ ] **Step 3: 删除 `g_filtered_error` 全局变量及相关滤波代码**

找到 `float g_filtered_error = 0.0f;` 声明，删除。在 `reset_line_follow_runtime_state()` 中删除对应的重置行 `g_filtered_error = 0.0f;`。

- [ ] **Step 4: 修改 `update_filtered_vision_inputs_if_new_frame` — 删除 IIR 滤波**

函数中删除以下行（约 607-614 行）：
```cpp
const float error_filter_tau_seconds = ...;
const float error_filter_alpha = ...;
g_filtered_error = apply_iir_filter(g_filtered_error, raw_error_px, error_filter_alpha);
```
保留 `g_line_error_px.store(raw_error_px);` 和跟踪点夹角滤波部分。

- [ ] **Step 5: 简化主循环中的位置环计算**

找到约 685-735 行，将：
```cpp
const ControlErrorState error_state = compute_control_error_state(g_filtered_error);

const float dynamic_kp =
    compute_piecewise_linear_abs_gain(route_profile.position_dynamic_kp_base, ...);

...

configure_line_follow_controllers_for_profile(route_profile, dynamic_kp,
                                               route_profile.position_kd,
                                               route_profile.yaw_rate_kp);

g_position_output_state = update_pid_output_state_if_needed(vision_updated,
                                                            error_state.control_error_px,
                                                            ...);
```

替换为：
```cpp
const float pos_error_px = g_line_error_px.load();

configure_line_follow_controllers_for_profile(route_profile,
                                              route_profile.position_kp,
                                              route_profile.position_kd,
                                              route_profile.yaw_rate_kp);

bool pos_dummy_has_time = false;
std::chrono::steady_clock::time_point pos_dummy_time;
g_position_output_state = update_pid_output_state_if_needed(vision_updated,
                                                            pos_error_px,
                                                            POSITION_PID_DT_SECONDS,
                                                            pos_dummy_time,
                                                            pos_dummy_has_time,
                                                            std::max(route_profile.position_max_output, 0.0f),
                                                            position_pid1,
                                                            g_position_output_state);
```

- [ ] **Step 6: 更新 debug status 字段**

将 `g_pid_debug_status.raw_error_px`、`filtered_error_px`、`abs_filtered_error_px`、`control_error_px` 全部改为 `pos_error_px`；`dynamic_position_kp` 改为 `route_profile.position_kp`。

---

### Task 2: 精简 `pid_tuning.h` Profile 结构体

**Files:**
- Modify: `project/code/driver/pid/pid_tuning.h`

- [ ] **Step 7: 替换位置环动态 Kp 字段**

找到 Profile 结构体中的位置环字段（约 98-112 行），将 8 个动态 Kp 字段：
```cpp
float position_dynamic_kp_quad_a;
float position_dynamic_kp_base;
float position_dynamic_kp_min;
float position_dynamic_kp_max;
float position_dynamic_kp_low_error_threshold_px;
float position_dynamic_kp_mid_a;
float position_dynamic_kp_mid_error_threshold_px;
float position_dynamic_kp_high_a;
```

替换为单一字段：
```cpp
float position_kp;
```

- [ ] **Step 8: 删除 `line_follow` namespace 中的滤波/死区参数**

删除：
```cpp
extern float kErrorFilterAlpha;
extern float kErrorDeadzonePx;
extern float kErrorLowGainLimitPx;
extern float kErrorLowGain;
```

- [ ] **Step 9: 删除 `is_dynamic_kp_range_valid` 和 `is_position_kp_piecewise_range_valid` 声明**

这两个验证函数不再需要。

---

### Task 3: 精简 `pid_tuning.cpp` 默认值和验证函数

**Files:**
- Modify: `project/code/driver/pid/pid_tuning.cpp`

- [ ] **Step 10: 删除 `line_follow` namespace 中的 4 个变量定义**

删除：
```cpp
float kErrorFilterAlpha = 0.95f;
float kErrorDeadzonePx = 0.6f;
float kErrorLowGainLimitPx = 3.0f;
float kErrorLowGain = 0.70f;
```

- [ ] **Step 11: 更新 `kNormalProfile` 初始化**

将第一组值从 8 个合并为 2 个（position_kp + position_ki, position_kd, max_integral, max_output = 1+4 个）：
```
之前:
  350.0f,                                              // base_speed
  3.0f, 2.1f, 0.0f, 50.0f, 3.0f, 4.6f, 10.0f, 5.6f,  // 8 dynamic Kp values
  0.0f, 0.15f, 0.0f, 210.0f,                           // position_ki, kd, max_integral, max_output

之后:
  350.0f,                                              // base_speed
  6.0f,                                                // position_kp
  0.0f, 0.15f, 0.0f, 210.0f,                           // position_ki, kd, max_integral, max_output
```

- [ ] **Step 12: 删除 `is_dynamic_kp_range_valid` 和 `is_position_kp_piecewise_range_valid` 函数体**

---

### Task 4: 精简 `smartcar_config.cpp` 配置解析

**Files:**
- Modify: `project/code/driver/config/smartcar_config.cpp`

- [ ] **Step 13: 更新 `load_route_profile` 函数**

将 8 行 `require_float(...position_dynamic_kp_*)` 替换为 1 行：
```cpp
require_float(values, consumed, prefix + ".position_kp", &profile->position_kp, error_message) &&
```

- [ ] **Step 14: 删除 `REQUIRE_PID_FLOAT` 对 4 个滤波/死区参数的解析**

找到 `kErrorFilterAlpha`、`kErrorDeadzonePx`、`kErrorLowGainLimitPx`、`kErrorLowGain` 的 `REQUIRE_PID_FLOAT` 行并删除。

- [ ] **Step 15: 删除 profile 验证中对已删除函数的调用**

找到对 `is_dynamic_kp_range_valid` 和 `is_position_kp_piecewise_range_valid` 的调用并删除。

- [ ] **Step 16: 删除 PidSnapshot 中相关字段**

从 snapshot 结构体和 capture/restore 函数中删除 `error_filter_alpha`、`error_deadzone_px`、`error_low_gain_limit_px`、`error_low_gain` 的代码。

---

### Task 5: 精简 `smartcar_config.toml`

**Files:**
- Modify: `project/user/smartcar_config.toml`

- [ ] **Step 17: 删除 `[pid.line_follow]` 中的滤波和死区参数**

删除这 4 行：
```toml
error_filter_alpha = 0.99
error_deadzone_px = 0.5
error_low_gain_limit_px = 1.0
error_low_gain = 0.4
```

- [ ] **Step 18: 替换 `[pid.route_line_follow.normal]` 中的动态 Kp 参数**

删除这 8 行位置环动态 Kp 参数，替换为：
```toml
position_kp = 6.0
```

---

### Task 6: 清理残留引用并编译验证

**Files:**
- Modify: `project/code/driver/vision/vision_transport.cpp`（如有引用）
- Modify: `project/code/app/line_follow_thread/line_follow_thread.h`（如有引用）

- [ ] **Step 19: 全局搜索残留引用**

```bash
grep -rn "kErrorFilterAlpha\|kErrorDeadzone\|kErrorLowGain\|position_dynamic_kp\|is_dynamic_kp_range\|is_position_kp_piecewise\|ControlErrorState\|compute_control_error_state\|compute_piecewise_linear_abs_gain\|g_filtered_error" project/code/
```

逐个清理。

- [ ] **Step 20: 编译验证**

```bash
cd project/out && cmake ../user -DUVC_RES_PRESET=1 -DUVC_FORMAT_PRESET=0 -DENABLE_NCNN=0 && make -j12
```

Expected: `[100%] Built target project`，无错误无警告。

- [ ] **Step 21: Commit**

```bash
git add -A
git commit -m "refactor: simplify position loop to fixed Kp PID

Remove IIR lowpass filter, deadzone, low-gain reduction, and
piecewise-linear dynamic Kp from the position loop. Raw pixel
error now feeds directly into a fixed-Kp positional PID."
```
