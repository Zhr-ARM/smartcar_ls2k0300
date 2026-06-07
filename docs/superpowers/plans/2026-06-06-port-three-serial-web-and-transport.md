# 移植三串控制 Web 前端 + 清理逐飞客户端/本地复算 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**目标:** 将 三串控制 分支的完整 Web 前端移植到 回退逐飞主摄，同时清理逐飞 seekfree_assistant 客户端发送链路和 wasm 本地复算模块。

**架构:** 三条独立轨道 —— A) Web 前端文件移植（纯前端，无后端依赖）；B) C++ 逐飞客户端发送链路删除（vision_thread → vision_pipeline → vision_transport → seekfree_assistant 整条调用链）；C) server.js 清理 wasm/local_compute 残余引用。

**技术栈:** C++ (vision_transport/pipeline/thread), Node.js (server.js), HTML/CSS/JS (前端)

---

## 前置梳理：当前分支 vs 三串控制分支 文件状态

| 文件 | 回退逐飞主摄 | 三串控制 | 操作 |
|------|------------|---------|------|
| `tools/pc_receiver_js/public/index.html` | 旧版宽布局 | 控制台风格重构 | **覆盖** |
| `tools/pc_receiver_js/public/shared_receiver_core.js` | 旧版 | +145 行改动 | **覆盖** |
| `tools/pc_receiver_js/public/project_control.js` | 不存在 | 411 行新文件 | **新建** |
| `tools/pc_receiver_js/public/receiver_frame_source.js` | 不存在 | 62 行新文件 | **新建** |
| `tools/pc_receiver_js/public/receiver_recording.js` | 不存在 | 129 行新文件 | **新建** |
| `tools/pc_receiver_js/public/playback.html` | 旧版 | 大幅改动 | **覆盖** |
| `tools/pc_receiver_js/public/playback_app.js` | 旧版 | 大幅改动 | **覆盖** |
| `tools/pc_receiver_js/public/config.html` | 旧版 | +1 行 | **覆盖** |
| `tools/pc_receiver_js/public/local_compute.html` | 存在 | 已删除 | **删除** |
| `tools/pc_receiver_js/public/local_compute_app.js` | 存在 | 已删除 | **删除** |
| `tools/pc_receiver_js/public/pipeline_worker.js` | 存在 | 已删除 | **删除** |
| `tools/pc_receiver_js/public/wasm/` | 存在 | 已删除 | **删除** |
| `tools/pc_receiver_js/server.js` | 含 wasm sync/local_compute 路由 | 已清理 | **增量修改** |
| `tools/pc_receiver_local_compute/` | 存在 | 已删除 | **删除整个目录** |
| `project/code/driver/vision/vision_transport.h` | 含 VISION_SEND_BINARY 枚举 + client sender API | 已清理 | **删除 client sender API** |
| `project/code/driver/vision/vision_transport.cpp` | 含 seekfree_assistant 发送代码 | 已清理 | **删除 client sender 实现** |
| `project/code/driver/vision/vision_pipeline.h` | 含 send_mode/max_fps/enabled 透传 API | 已清理 | **删除透传 API 声明** |
| `project/code/driver/vision/vision_pipeline.cpp` | 含透传实现 | 已清理 | **删除透传实现** |
| `project/code/app/vision_thread/vision_thread.h` | 含 VISION_THREAD_SEND_* 枚举 + 外层 API | 已清理 | **删除外层 send API** |
| `project/code/app/vision_thread/vision_thread.cpp` | 含 wrapper 实现 + last_send_time 调用 | 已清理 | **删除 wrapper + 修复 send_us 引用** |
| `project/code/driver/vision/vision_assistant_udp.h` | 存在 | ? | **删除（无人调用）** |
| `project/code/driver/vision/vision_assistant_udp.cpp` | 存在 | ? | **删除（无人调用）** |

---

## Track A: Web 前端移植

### Task A1: 从 三串控制 提取全部前端文件

- [ ] **Step 1: 批量提取三串控制分支的前端文件到临时目录**

```bash
mkdir -p /tmp/three_serial_frontend
for f in index.html shared_receiver_core.js project_control.js \
         receiver_frame_source.js receiver_recording.js \
         playback.html playback_app.js config.html; do
  git show 三串控制:tools/pc_receiver_js/public/$f > "/tmp/three_serial_frontend/$f" 2>/dev/null && \
    echo "$f OK ($(wc -c < /tmp/three_serial_frontend/$f) bytes)" || \
    echo "$f SKIP (not found)"
done
```

- [ ] **Step 2: 备份当前前端文件**

```bash
mkdir -p tools/pc_receiver_js/public/.bak
cp tools/pc_receiver_js/public/*.html tools/pc_receiver_js/public/.bak/ 2>/dev/null || true
cp tools/pc_receiver_js/public/*.js tools/pc_receiver_js/public/.bak/ 2>/dev/null || true
echo "Backup created in tools/pc_receiver_js/public/.bak/"
```

- [ ] **Step 3: 覆盖/新建前端文件**

```bash
cp /tmp/three_serial_frontend/index.html tools/pc_receiver_js/public/index.html
cp /tmp/three_serial_frontend/shared_receiver_core.js tools/pc_receiver_js/public/shared_receiver_core.js
cp /tmp/three_serial_frontend/project_control.js tools/pc_receiver_js/public/project_control.js
cp /tmp/three_serial_frontend/receiver_frame_source.js tools/pc_receiver_js/public/receiver_frame_source.js
cp /tmp/three_serial_frontend/receiver_recording.js tools/pc_receiver_js/public/receiver_recording.js
cp /tmp/three_serial_frontend/playback.html tools/pc_receiver_js/public/playback.html
cp /tmp/three_serial_frontend/playback_app.js tools/pc_receiver_js/public/playback_app.js
cp /tmp/three_serial_frontend/config.html tools/pc_receiver_js/public/config.html
echo "Done"
```

- [ ] **Step 4: 删除本地复算相关前端文件**

```bash
rm -f tools/pc_receiver_js/public/local_compute.html
rm -f tools/pc_receiver_js/public/local_compute_app.js
rm -f tools/pc_receiver_js/public/pipeline_worker.js
rm -rf tools/pc_receiver_js/public/wasm
echo "Removed local_compute frontend files"
```

- [ ] **Step 5: 验证前端文件完整性**

```bash
echo "=== Required files ===" && \
for f in index.html shared_receiver_core.js project_control.js \
         receiver_frame_source.js receiver_recording.js \
         playback.html playback_app.js config.html; do
  [ -f "tools/pc_receiver_js/public/$f" ] && echo "  OK $f" || echo "  MISSING $f"
done && \
echo "=== Should NOT exist ===" && \
for f in local_compute.html local_compute_app.js pipeline_worker.js; do
  [ -f "tools/pc_receiver_js/public/$f" ] && echo "  STILL EXISTS $f (should be deleted)" || echo "  OK $f (removed)"
done && \
[ -d "tools/pc_receiver_js/public/wasm" ] && echo "  STILL EXISTS wasm/ (should be deleted)" || echo "  OK wasm/ (removed)"
```

- [ ] **Step 6: 提交**

```bash
git add tools/pc_receiver_js/public/
git commit -m "feat: 移植三串控制分支 Web 前端，删除本地复算前端文件

- 覆盖 index.html 为控制台风格布局
- 新增 project_control.js / receiver_frame_source.js / receiver_recording.js
- 更新 shared_receiver_core.js / playback.html / playback_app.js / config.html
- 删除 local_compute.html / local_compute_app.js / pipeline_worker.js / wasm/

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Track B: C++ 逐飞客户端发送链路清理

### 调用链分析（自顶向下）

```
vision_thread.h / vision_thread.cpp
  └─ vision_thread_set_send_mode() → vision_pipeline_set_send_mode()
  └─ vision_thread_get_last_send_time_us() → vision_transport_get_last_send_time_us()

vision_pipeline.h / vision_pipeline.cpp
  └─ vision_pipeline_set_send_mode() → vision_transport_set_send_mode()
  └─ vision_pipeline_set_send_max_fps() → vision_transport_set_send_max_fps()
  └─ vision_pipeline_set_send_enabled() → vision_transport_set_send_enabled()

vision_transport.h / vision_transport.cpp
  └─ g_send_mode, g_send_enabled, g_last_send_mode, g_send_max_fps
  └─ config_camera_send_packet() → seekfree_assistant_camera_information_config()
  └─ refresh_camera_boundary_packet() → seekfree_assistant_camera_boundary_config()
  └─ vision_transport_send_step() → seekfree_assistant_camera_send()
```

### Task B1: 清理 vision_transport.h —— 删除 client sender API 声明

**文件:** `project/code/driver/vision/vision_transport.h`

- [ ] **Step 1: 删除 VISION_SEND_BINARY / VISION_SEND_GRAY 枚举**

删除:
```cpp
typedef enum
{
    VISION_SEND_BINARY = 0, // 发送二值图（黑白）。
    VISION_SEND_GRAY = 1    // 发送灰度图（可叠加边线）。
} vision_send_mode_enum;
```

- [ ] **Step 2: 删除 client sender 相关的 8 个函数声明**

删除以下声明块:
```cpp
// 作用：读取最近一次"客户端发送"耗时（us）。
uint32 vision_transport_get_last_send_time_us();

// 作用：设置客户端发送图像模式。
void vision_transport_set_send_mode(vision_send_mode_enum mode);
vision_send_mode_enum vision_transport_get_send_mode();

// 作用：设置/读取客户端发送限频（0 表示不限）。
void vision_transport_set_send_max_fps(uint32 max_fps);
uint32 vision_transport_get_send_max_fps();

// 作用：开关客户端发送。
void vision_transport_set_send_enabled(bool enabled);
bool vision_transport_is_send_enabled();
```

### Task B2: 清理 vision_transport.cpp —— 删除 client sender 实现

**文件:** `project/code/driver/vision/vision_transport.cpp`

- [ ] **Step 1: 删除 client sender 全局变量块**

删除:
```cpp
// ---------- client sender ----------
static constexpr uint32 kClientDefaultMaxFps = 30;
static constexpr uint32 kClientMaxFpsUpper = 240;
// 客户端发送模式与开关。
static std::atomic<int> g_send_mode(VISION_SEND_BINARY);
static std::atomic<bool> g_send_enabled(true);
// 最近一次已配置模式（避免重复配置底层发送结构）。
static std::atomic<int> g_last_send_mode(-1);
// 最近一次客户端发送耗时（us）。
static std::atomic<uint32> g_last_send_time_us(0);
// 最近一次客户端发送时间戳（用于限频）。
static std::atomic<uint64> g_last_send_tick_us(0);
// 客户端发送最大 FPS（0 表示不限）。
static std::atomic<uint32> g_send_max_fps(kClientDefaultMaxFps);
```

- [ ] **Step 2: 删除 `vision_sender_sanitize_mode()` 函数**

删除:
```cpp
static vision_send_mode_enum vision_sender_sanitize_mode(vision_send_mode_enum mode)
{
    if (mode == VISION_SEND_BINARY)
    {
        return VISION_SEND_BINARY;
    }
    return VISION_SEND_GRAY;
}
```

- [ ] **Step 3: 删除 `mode_enable_boundary_packet()` 函数**

删除:
```cpp
static bool mode_enable_boundary_packet(vision_send_mode_enum mode)
{
    return mode == VISION_SEND_GRAY;
}
```

- [ ] **Step 4: 删除 `config_camera_send_packet()` 函数**

删除整个函数（约 40 行，包含 seekfree_assistant_camera_information_config 和 seekfree_assistant_camera_boundary_config 调用）。

- [ ] **Step 5: 删除 `refresh_camera_boundary_packet()` 函数**

删除整个函数（约 25 行）。

- [ ] **Step 6: 删除 `vision_transport_send_step()` 中的 client sender 代码块**

在 `vision_transport_send_step()` 函数中，删除以 `g_last_send_mode.store(-1)` 开始到函数末尾的 client sender 部分（包含 `g_send_max_fps` 限频检查、`config_camera_send_packet`、`refresh_camera_boundary_packet`、`seekfree_assistant_camera_send` 调用）。

函数末尾只保留：
```cpp
void vision_transport_send_step()
{
    send_tcp_status();
    send_udp_image();
}
```

- [ ] **Step 7: 删除 6 个 getter/setter 函数实现**

删除:
```cpp
uint32 vision_transport_get_last_send_time_us() { ... }
void vision_transport_set_send_mode(vision_send_mode_enum mode) { ... }
vision_send_mode_enum vision_transport_get_send_mode() { ... }
void vision_transport_set_send_max_fps(uint32 max_fps) { ... }
uint32 vision_transport_get_send_max_fps() { ... }
void vision_transport_set_send_enabled(bool enabled) { ... }
bool vision_transport_is_send_enabled() { ... }
```

### Task B3: 清理 vision_pipeline.h 和 vision_pipeline.cpp —— 删除透传 API

**文件:** `project/code/driver/vision/vision_pipeline.h`

- [ ] **Step 1: 删除 send 透传声明**

删除:
```cpp
// 作用：发送配置透传给 transport。
void vision_pipeline_set_send_mode(vision_send_mode_enum mode);
vision_send_mode_enum vision_pipeline_get_send_mode();
void vision_pipeline_set_send_max_fps(uint32 max_fps);
uint32 vision_pipeline_get_send_max_fps();
void vision_pipeline_set_send_enabled(bool enabled);
bool vision_pipeline_is_send_enabled();
```

**文件:** `project/code/driver/vision/vision_pipeline.cpp`

- [ ] **Step 2: 删除 send 透传实现**

删除:
```cpp
void vision_pipeline_set_send_mode(vision_send_mode_enum mode)
{
    vision_transport_set_send_mode(mode);
}
vision_send_mode_enum vision_pipeline_get_send_mode()
{
    return vision_transport_get_send_mode();
}
void vision_pipeline_set_send_max_fps(uint32 max_fps)
{
    vision_transport_set_send_max_fps(max_fps);
}
uint32 vision_pipeline_get_send_max_fps()
{
    return vision_transport_get_send_max_fps();
}
void vision_pipeline_set_send_enabled(bool enabled)
{
    vision_transport_set_send_enabled(enabled);
}
bool vision_pipeline_is_send_enabled()
{
    return vision_transport_is_send_enabled();
}
```

### Task B4: 清理 vision_thread.h 和 vision_thread.cpp —— 删除外层 send API

**文件:** `project/code/app/vision_thread/vision_thread.h`

- [ ] **Step 1: 删除 send 枚举和外层 API 声明**

删除:
```cpp
typedef enum
{
    VISION_THREAD_SEND_BINARY = 0,
    VISION_THREAD_SEND_GRAY = 1
} vision_thread_send_mode_enum;

void vision_thread_set_send_mode(vision_thread_send_mode_enum mode);
vision_thread_send_mode_enum vision_thread_get_send_mode();
void vision_thread_set_send_max_fps(uint32 max_fps);
uint32 vision_thread_get_send_max_fps();
void vision_thread_set_client_sender_enabled(bool enabled);
bool vision_thread_client_sender_enabled();
```

**文件:** `project/code/app/vision_thread/vision_thread.cpp`

- [ ] **Step 2: 删除 `vision_thread_sanitize_send_mode()` 静态函数**

删除:
```cpp
static vision_thread_send_mode_enum vision_thread_sanitize_send_mode(vision_thread_send_mode_enum mode)
{
    if (mode == VISION_THREAD_SEND_BINARY)
    {
        return VISION_THREAD_SEND_BINARY;
    }
    return VISION_THREAD_SEND_GRAY;
}
```

- [ ] **Step 3: 修复 `send_us_this_frame` 引用**

将:
```cpp
uint32 send_us_this_frame = 0;

if (!vision_pipeline_process_step())
{
    // ...error handling...
}

vision_pipeline_send_step();
send_us_this_frame = vision_transport_get_last_send_time_us();
```

改为:
```cpp
if (!vision_pipeline_process_step())
{
    // ...error handling...
}

vision_pipeline_send_step();
```

并删除:
```cpp
perf_acc.send_us += send_us_this_frame;
```

- [ ] **Step 4: 删除 6 个 wrapper 函数实现**

删除 `vision_thread_set_send_mode`, `vision_thread_get_send_mode`, `vision_thread_set_send_max_fps`, `vision_thread_get_send_max_fps`, `vision_thread_set_client_sender_enabled`, `vision_thread_client_sender_enabled`。

### Task B5: 删除 vision_assistant_udp 文件

- [ ] **Step 1: 确认无人调用**

```bash
grep -rn "vision_assistant_udp" project/code/ --include="*.cpp" --include="*.h" 2>/dev/null
# 应只有 vision_assistant_udp.cpp 自身 include 其 .h
```

- [ ] **Step 2: 删除文件**

```bash
rm project/code/driver/vision/vision_assistant_udp.cpp
rm project/code/driver/vision/vision_assistant_udp.h
```

### Task B6: 编译验证

- [ ] **Step 1: 编译检查**

```bash
cd project/code && make -j$(nproc) 2>&1 | tail -30
```

如果出现编译错误（如其他文件引用了被删除的符号），根据错误定位并修复。

- [ ] **Step 2: 提交**

```bash
git add project/code/driver/vision/vision_transport.h \
        project/code/driver/vision/vision_transport.cpp \
        project/code/driver/vision/vision_pipeline.h \
        project/code/driver/vision/vision_pipeline.cpp \
        project/code/app/vision_thread/vision_thread.h \
        project/code/app/vision_thread/vision_thread.cpp
git rm project/code/driver/vision/vision_assistant_udp.cpp \
        project/code/driver/vision/vision_assistant_udp.h
git commit -m "feat: 删除逐飞 seekfree_assistant 客户端发送链路

移除 vision_thread → vision_pipeline → vision_transport → seekfree_assistant
整条客户端发送调用链，仅保留 UDP/TCP 网页端发送。

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Track C: server.js 清理 + tools 目录清理

### Task C1: 清理 server.js 中的 wasm/local_compute 引用

**文件:** `tools/pc_receiver_js/server.js`

- [ ] **Step 1: 删除 wasm_sync_meta require**

删除:
```javascript
const { summarizeSyncStatus } = require('../pc_receiver_local_compute/wasm_sync_meta.js');
```

- [ ] **Step 2: 删除 WASM 相关常量**

删除:
```javascript
const WASM_DIR = path.join(PUBLIC_DIR, 'wasm');
const WASM_SYNC_METADATA_PATH = path.join(WASM_DIR, 'vision_pipeline.sync.json');
```

- [ ] **Step 3: 删除 cachedWasmSyncStatus 变量及初始化**

删除:
```javascript
let cachedWasmSyncStatus = null;
let cachedWasmSyncStatusAtMs = 0;
```

以及初始化代码中调用 `summarizeSyncStatus()` 的部分。

- [ ] **Step 4: 删除 local_compute 路由**

删除:
```javascript
if (pathname === '/local_compute.html') {
  serveFile(res, path.join(PUBLIC_DIR, 'local_compute.html'), 'text/html; charset=utf-8');
  return;
}
if (pathname === '/local_compute_app.js') {
  serveFile(res, path.join(PUBLIC_DIR, 'local_compute_app.js'), 'application/javascript; charset=utf-8');
  return;
}
```

- [ ] **Step 5: 删除 WASM 文件服务路由**

删除 `/wasm/` 路径的文件服务逻辑。

- [ ] **Step 6: 删除 `/api/wasm_sync_status` 路由**

删除:
```javascript
if (pathname === '/api/wasm_sync_status') { ... }
```

- [ ] **Step 7: HTTP_PORT 改为 9090**（与三串控制对齐）

```javascript
const HTTP_PORT = Number(process.env.HTTP_PORT || 9090);
```

- [ ] **Step 8: 新增 `tcpStatusEvents` 数组**（在 udpFrameEvents 声明之后）

```javascript
const tcpStatusEvents = [];
```

- [ ] **Step 9: 新增 `backendRecording` 和 `projectControlState`**（在 boardConnectionStore 之后）

```javascript
let backendRecording = null;
let projectControlState = {
  last_action: 'idle',
  last_message: '',
  last_error: '',
  last_updated_at_ms: 0
};
```

- [ ] **Step 10: 验证 server.js 语法**

```bash
node --check tools/pc_receiver_js/server.js
```

### Task C2: 删除整个 pc_receiver_local_compute 目录

- [ ] **Step 1: 删除**

```bash
rm -rf tools/pc_receiver_local_compute/
```

- [ ] **Step 2: 验证 server.js 仍能正常 check**

```bash
node --check tools/pc_receiver_js/server.js
```

### Task C3: 提交

```bash
git add tools/pc_receiver_js/server.js
git rm -r tools/pc_receiver_local_compute/
git commit -m "feat: 清理 wasm 本地复算模块和服务端引用

- 删除 tools/pc_receiver_local_compute/ 整个目录
- server.js: 移除 wasm_sync_meta、WASM_DIR、local_compute 路由、wasm 文件服务
- server.js: HTTP 端口改为 9090，新增 tcpStatusEvents/projectControlState

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## 执行顺序

三个 Track 互相独立，可以按任意顺序执行。建议:

1. **先 Track A**（Web 前端）—— 无编译依赖，可立即验证
2. **再 Track B**（C++ 清理）—— 需要编译验证
3. **最后 Track C**（server.js + 目录清理）—— 需要 node 语法检查

---

## 注意事项

1. **缺失数据字段不补 API** —— 三串控制的 index.html 引用的 TCP 状态字段（如 `pid_common_cascade_mode`、`board_debug_*` 等）在 回退逐飞主摄 上没有数据来源，页面卡片将显示空白/黑色/默认值。这是预期行为，不需要修改 C++ 端。

2. **编译验证关键点** —— Task B 删除 `vision_transport_get_last_send_time_us()` 后，vision_thread.cpp 中的 `send_us` 引用必须同步删除，否则编译失败。

3. **server.js 的 wasm_sync_meta require** —— 必须在删除 `tools/pc_receiver_local_compute/` 目录之前先从 server.js 中移除引用，否则 node --check 会报找不到模块。

4. **HTTP 端口变为 9090** —— 如果其他工具/书签硬编码了 `localhost:8080`，需要更新。
