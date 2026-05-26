# UDP Unified Vision Transport Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the PC receiver's split UDP-image/TCP-status live stream with one UDP-only per-sample bundle so images, boundaries, centerlines, and telemetry update atomically from the same vision result, while fully removing the obsolete SeekFree assistant sender path.

**Architecture:** Keep the existing image encoders and JSON status builder, but route both into a unified UDP bundle format keyed by `bundle_id`. Each send slot captures the latest processed result once, emits a manifest JSON item, a status JSON item, and enabled image items over UDP chunks; the PC receiver publishes a display update only when the manifest, status, and all manifest-required image modes for that bundle have arrived.

**Tech Stack:** C++17, OpenCV `imencode`, existing `zf_driver_udp`, Node.js `dgram`/HTTP/WebSocket receiver, TOML config, existing `project/user/build.sh` and `tools/pc_receiver_js` smoke tests.

---

## Feasibility And Expected Effect

This is feasible with moderate risk. The current lower-side path already sends image chunks over UDP in `project/code/driver/vision/vision_transport.cpp`, and the PC receiver already reassembles UDP chunks in `tools/pc_receiver_js/server.js`. The main change is not a new transport stack; it is replacing "image over UDP plus status over TCP" with "typed UDP items inside one bundle".

The SeekFree assistant path can be removed instead of preserved. That simplifies the transport module because there is no longer a need to configure `seekfree_assistant_camera_information_config()`, refresh `seekfree_assistant_camera_boundary_config()`, or run a separate assistant UDP callback in parallel with the web receiver.

Expected effect:
- Frame/status consistency improves from best-effort timing alignment to manifest-enforced bundle-level alignment.
- Latency should drop slightly because TCP head-of-line blocking and JSON socket buffering disappear.
- Packet loss behavior changes: TCP status was reliable; UDP status becomes lossy. For live debugging this is usually acceptable because the next bundle supersedes the old one.
- At 60 FPS, single gray JPEG plus full boundary/status JSON should be realistic on the current network. At 110 FPS, CPU will likely be limited by OpenCV image encoding and JSON construction before raw UDP send overhead.

CPU/load expectation:
- Removing TCP saves one socket path and TCP bookkeeping, but that is small compared with JPEG/PNG encode and large JSON string construction.
- Unified UDP adds a little header work and bundle bookkeeping; this is negligible.
- Removing the SeekFree assistant path saves one extra per-frame send gate and boundary packet refresh path. If `client_sender_enabled` is already false, runtime CPU improvement is small, but code complexity and accidental double-send risk drop noticeably.
- If status JSON is built only at send slots instead of every vision loop, CPU is similar to current gated path.
- The real CPU win comes from choosing `RAW_MINIMAL`, sending only one image mode, and limiting FPS to 30-60 for debugging. Sending gray JPEG at 100 quality at 110 FPS is likely much heavier than the TCP part ever was.

Recommended operating profile:
- Start with `vision.runtime.web.max_fps = 60`, `send_gray_jpeg = true`, `gray_image_format = 0`, other image streams false.
- Use `data_profile = 1` for normal live driving; switch to full debug only while tuning routes.
- Raise to 90/110 only after measuring `perf_total_us`, `cpu_usage_percent`, `rx_udp_mbps`, and actual drop rate.

## File Structure

- Modify `project/code/driver/vision/vision_transport.h`
  - Rename comments from UDP/TCP to unified UDP where public behavior changes.
  - Remove client/SeekFree assistant sender APIs: `vision_transport_set_send_max_fps()`, `vision_transport_set_send_enabled()`, and their getters.
  - Keep `vision_transport_set_send_mode()` / `vision_transport_get_send_mode()` as local display mode state because `screen_display_thread` still reads it.
  - Keep web UDP APIs only.

- Modify `project/code/driver/vision/vision_transport.cpp`
  - Add bundle/chunk wire structs.
  - Refactor status JSON construction into `build_web_status_json()`.
  - Send manifest JSON, status JSON, and enabled image frames as typed UDP bundle items from the same send slot.
  - Remove live dependency on `tcp_client_send_data()` for web status; no TCP fallback remains in the live web path.
  - Delete the `seekfree_assistant_camera_*` configuration/refresh/send path and its atomics.

- Delete `project/code/driver/vision/vision_assistant_udp.h`
  - Remove the standalone SeekFree assistant UDP callback interface.

- Delete `project/code/driver/vision/vision_assistant_udp.cpp`
  - Remove the standalone SeekFree assistant UDP socket implementation.

- Modify `project/user/main.cpp`
  - Remove `vision_assistant_udp_init()` setup, cleanup, and related logging.
  - Remove `vision_thread_set_send_max_fps()` and `vision_thread_set_client_sender_enabled()` calls.

- Modify `project/code/app/vision_thread/vision_thread.h`
  - Remove client sender setter/getter declarations.

- Modify `project/code/app/vision_thread/vision_thread.cpp`
  - Remove client sender setter/getter wrappers.

- Modify `project/code/driver/vision/vision_pipeline.h`
  - Remove client sender setter/getter declarations.

- Modify `project/code/driver/vision/vision_pipeline.cpp`
  - Remove client sender setter/getter wrappers.

- Modify `project/code/driver/vision/vision_config.h`
  - Remove `send_max_fps`, `client_sender_enabled`, `assistant_udp_enabled`, `assistant_server_ip`, and `assistant_server_port`.

- Modify `project/code/driver/config/smartcar_config.cpp`
  - Remove parsing/diff/application of `vision.runtime.send_max_fps`, `vision.runtime.client_sender_enabled`, and `vision.runtime.assistant.*`.
  - Keep parsing `tcp_enabled` and `meta_port` as inert compatibility fields for the web UI/config editor during this pass; runtime live status no longer uses TCP.

- Modify `project/user/smartcar_config.toml`
  - Set the live profile used for testing: unified UDP enabled, TCP disabled or ignored, FPS set deliberately.
  - Remove top-level `send_max_fps`, `client_sender_enabled`, and the full `[vision.runtime.assistant]` section.

- Modify `project/user/sync_connection_preset_to_toml.js`
  - Stop requiring `assistant_receiver_ip`.
  - Stop editing `[vision.runtime.assistant]`.

- Modify `project/user/connection_presets.json`
  - Remove `assistant_receiver_ip` from each preset.

- Modify `tools/pc_receiver_js/server.js`
  - Add unified UDP bundle parser while keeping the old image parser for compatibility during transition.
  - Reassemble by `(bundle_id, item_type, mode)`.
  - Update `latestStatus` and `latestByMode` atomically when a bundle is complete enough.

- Modify `tools/pc_receiver_js/public/shared_receiver_core.js` and `tools/pc_receiver_js/public/index.html` only if the frontend assumes independent status/frame events.
  - Prefer keeping API shape stable: `/api/status`, `/api/frame/gray`, and WebSocket `status`/`frame` events continue to exist.

- Add `tools/pc_receiver_js/scripts/test_unified_udp_bundle.js`
  - Unit-style receiver parser test with synthetic status and chunked image packets.

## Wire Format

Use a new magic to avoid confusing the existing frame parser. The header is exactly 32 bytes on both C++ and JS sides:

```cpp
static constexpr uint32 kBundleMagic = 0x56535542; // VSUB
static constexpr uint16 kBundleHeaderSize = 32;

#pragma pack(push, 1)
struct udp_bundle_chunk_header_t
{
    uint32 magic;
    uint32 bundle_id;
    uint16 item_idx;
    uint16 item_total;
    uint16 chunk_idx;
    uint16 chunk_total;
    uint16 payload_len;
    uint8 item_type;     // 1=manifest_json, 2=status_json, 3=image
    uint8 image_mode;    // 0=binary, 1=gray, 2=rgb, 3=roi64, 255=not image
    uint16 width;
    uint16 height;
    uint8 image_format;  // existing 0=JPEG, 1=PNG, 2=BMP, 255=not image
    uint8 flags;         // reserved for future use, send 0 for now
    uint16 reserved;
    uint32 reserved2;
};
#pragma pack(pop)
static_assert(sizeof(udp_bundle_chunk_header_t) == kBundleHeaderSize,
              "udp bundle header size must match JS receiver");
```

Item types:
- `1=manifest_json`
- `2=status_json`
- `3=image`

Manifest JSON format:

```json
{"required_image_modes":[1],"optional_image_modes":[3]}
```

Bundle publish rule:
- Manifest JSON and status JSON are required.
- Enabled primary image streams are listed in `required_image_modes`: gray/binary/rgb according to config.
- ROI64 is listed in `optional_image_modes` only when valid because it may be absent when no ROI exists.
- PC receiver publishes a bundle when manifest, status, and every manifest-required image mode are complete.
- If a newer complete bundle arrives, older incomplete bundles are discarded.
- If an optional ROI64 chunk is lost, publish the bundle without updating ROI64.

---

### Task 1: Remove SeekFree Assistant Sender Path

**Files:**
- Modify: `project/code/driver/vision/vision_transport.h`
- Modify: `project/code/driver/vision/vision_transport.cpp`
- Delete: `project/code/driver/vision/vision_assistant_udp.h`
- Delete: `project/code/driver/vision/vision_assistant_udp.cpp`
- Modify: `project/code/app/vision_thread/vision_thread.h`
- Modify: `project/code/app/vision_thread/vision_thread.cpp`
- Modify: `project/code/driver/vision/vision_pipeline.h`
- Modify: `project/code/driver/vision/vision_pipeline.cpp`
- Modify: `project/user/main.cpp`
- Modify: `project/code/driver/vision/vision_config.h`
- Modify: `project/code/driver/vision/vision_config.c`
- Modify: `project/code/driver/config/smartcar_config.cpp`
- Modify: `project/user/smartcar_config.toml`
- Modify: `project/user/sync_connection_preset_to_toml.js`
- Modify: `project/user/connection_presets.json`

- [ ] **Step 1: Remove assistant source files from the build**

Search for explicit source lists:

```bash
rg -n "vision_assistant_udp|vision_transport|vision_thread" project/user/CMakeLists.txt project -g 'CMakeLists.txt'
```

If `project/code/driver/vision/vision_assistant_udp.cpp` is listed explicitly, delete that list entry. If sources are globbed, no CMake edit is needed.

- [ ] **Step 2: Delete assistant UDP implementation files**

```bash
rm project/code/driver/vision/vision_assistant_udp.h project/code/driver/vision/vision_assistant_udp.cpp
```

Expected: `rg -n "vision_assistant_udp" project/code project/user` only reports references that will be removed in later steps.

- [ ] **Step 3: Remove assistant setup from `main.cpp`**

In `project/user/main.cpp`, delete:

```cpp
#include "driver/vision/vision_assistant_udp.h"
```

Delete cleanup call:

```cpp
vision_assistant_udp_cleanup();
```

Delete the whole assistant init block:

```cpp
// 初始化逐飞助手独立 UDP 通道（不与网页端共用 IP/端口）。
if (g_vision_runtime_config.assistant_udp_enabled)
{
    if (!vision_assistant_udp_init(g_vision_runtime_config.assistant_server_ip,
                                   g_vision_runtime_config.assistant_server_port))
    {
        printf("[ASSISTANT_UDP] init failed ip=%s port=%u\r\n",
               g_vision_runtime_config.assistant_server_ip,
               static_cast<unsigned int>(g_vision_runtime_config.assistant_server_port));
    }
    else
    {
        printf("[ASSISTANT_UDP] ready=1 ip=%s port=%u\r\n",
               g_vision_runtime_config.assistant_server_ip,
               static_cast<unsigned int>(g_vision_runtime_config.assistant_server_port));
    }
}
else
{
    printf("[ASSISTANT_UDP] disabled\r\n");
}
```

Delete client sender runtime application:

```cpp
vision_thread_set_send_max_fps(g_vision_runtime_config.send_max_fps);
vision_thread_set_client_sender_enabled(g_vision_runtime_config.client_sender_enabled);
```

Remove `vision_thread_get_send_max_fps()` and `vision_thread_client_sender_enabled()` arguments from the startup `printf`, replacing them with web-only fields:

```cpp
printf("[VISION_CONFIG] web_enabled=%d web_max_fps=%u web_tcp=%d\r\n",
       g_vision_runtime_config.udp_web_enabled ? 1 : 0,
       static_cast<unsigned int>(g_vision_runtime_config.udp_web_max_fps),
       g_vision_runtime_config.udp_web_tcp_enabled ? 1 : 0);
```

- [ ] **Step 4: Remove client sender API from `vision_thread` and `vision_pipeline`**

Delete these declarations from `project/code/app/vision_thread/vision_thread.h`:

```cpp
void vision_thread_set_send_max_fps(uint32 max_fps);
uint32 vision_thread_get_send_max_fps();
void vision_thread_set_client_sender_enabled(bool enabled);
bool vision_thread_client_sender_enabled();
```

Delete these definitions from `project/code/app/vision_thread/vision_thread.cpp`:

```cpp
void vision_thread_set_send_max_fps(uint32 max_fps)
{
    vision_pipeline_set_send_max_fps(max_fps);
}

uint32 vision_thread_get_send_max_fps()
{
    return vision_pipeline_get_send_max_fps();
}

void vision_thread_set_client_sender_enabled(bool enabled)
{
    vision_pipeline_set_send_enabled(enabled);
}

bool vision_thread_client_sender_enabled()
{
    return vision_pipeline_is_send_enabled();
}
```

Delete these declarations from `project/code/driver/vision/vision_pipeline.h`:

```cpp
void vision_pipeline_set_send_max_fps(uint32 max_fps);
uint32 vision_pipeline_get_send_max_fps();
void vision_pipeline_set_send_enabled(bool enabled);
bool vision_pipeline_is_send_enabled();
```

Delete these definitions from `project/code/driver/vision/vision_pipeline.cpp`:

```cpp
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

- [ ] **Step 5: Remove SeekFree sender code from `vision_transport`**

In `project/code/driver/vision/vision_transport.cpp`, remove:

```cpp
static constexpr uint32 kClientDefaultMaxFps = 30;
static constexpr uint32 kClientMaxFpsUpper = 240;
static std::atomic<bool> g_send_enabled(true);
static std::atomic<int> g_last_send_mode(-1);
static std::atomic<uint64> g_last_send_tick_us(0);
static std::atomic<uint32> g_send_max_fps(kClientDefaultMaxFps);
```

Delete these helper functions:

```cpp
static bool mode_enable_boundary_packet(vision_send_mode_enum mode);
static void config_camera_send_packet(vision_send_mode_enum mode);
static void refresh_camera_boundary_packet(vision_send_mode_enum mode);
```

Replace the old sender sanitizer with a local-display-only sanitizer:

```cpp
static vision_send_mode_enum vision_transport_sanitize_display_mode(vision_send_mode_enum mode)
{
    if (mode == VISION_SEND_BINARY)
    {
        return VISION_SEND_BINARY;
    }
    return VISION_SEND_GRAY;
}
```

In `vision_transport_send_step()`, delete the first send-gate block that calls:

```cpp
config_camera_send_packet(mode);
refresh_camera_boundary_packet(mode);
seekfree_assistant_camera_send();
```

Keep the web UDP `try_acquire_udp_send_slot()` block.

Keep these public functions because the screen display still needs a mode selector:

```cpp
void vision_transport_set_send_mode(vision_send_mode_enum mode)
{
    g_send_mode.store(static_cast<int>(vision_transport_sanitize_display_mode(mode)));
}

vision_send_mode_enum vision_transport_get_send_mode()
{
    return static_cast<vision_send_mode_enum>(g_send_mode.load());
}
```

Delete these public functions from `vision_transport.cpp` and their declarations from `vision_transport.h`:

```cpp
void vision_transport_set_send_max_fps(uint32 max_fps);
uint32 vision_transport_get_send_max_fps();
void vision_transport_set_send_enabled(bool enabled);
bool vision_transport_is_send_enabled();
```

- [ ] **Step 6: Remove assistant fields from runtime config**

In `project/code/driver/vision/vision_config.h`, remove:

```cpp
uint32 send_max_fps;
bool client_sender_enabled;
bool assistant_udp_enabled;
const char *assistant_server_ip;
uint16 assistant_server_port;
```

In `project/code/driver/vision/vision_config.c`, remove the matching initializer entries:

```cpp
.send_max_fps = 60,
.client_sender_enabled = false,
.assistant_udp_enabled = false,
.assistant_server_ip = "172.21.79.129",
.assistant_server_port = 8899,
```

- [ ] **Step 7: Remove config parser fields**

In `project/code/driver/config/smartcar_config.cpp`, remove `assistant_server_ip` from `StringStorage`:

```cpp
std::string assistant_server_ip;
```

Remove assignment:

```cpp
g_vision_runtime_config.assistant_server_ip = g_string_storage.assistant_server_ip.c_str();
```

Remove diff tracking for:

```cpp
vision.runtime.assistant.enabled
vision.runtime.assistant.server_port
vision.runtime.assistant.server_ip
```

Remove application calls:

```cpp
vision_transport_set_send_max_fps(g_vision_runtime_config.send_max_fps);
vision_thread_set_send_max_fps(g_vision_runtime_config.send_max_fps);
vision_thread_set_client_sender_enabled(g_vision_runtime_config.client_sender_enabled);
```

Remove required parsing for:

```cpp
vision.runtime.send_max_fps
vision.runtime.client_sender_enabled
vision.runtime.assistant.enabled
vision.runtime.assistant.server_ip
vision.runtime.assistant.server_port
```

Keep `vision.runtime.send_mode`; `screen_display_thread` still uses it to choose binary/gray display mode.

- [ ] **Step 8: Clean TOML and preset sync**

In `project/user/smartcar_config.toml`, remove:

```toml
send_max_fps = 60
client_sender_enabled = false

[vision.runtime.assistant]
enabled = false
server_ip = "192.168.3.9"
server_port = 8899
```

In `project/user/sync_connection_preset_to_toml.js`, remove all `assistantReceiverIp`, `replacedAssistant`, and `[vision.runtime.assistant]` update logic. The script should only update `vision.runtime.web.server_ip`.

In `project/user/connection_presets.json`, remove every:

```json
"assistant_receiver_ip": "..."
```

- [ ] **Step 9: Verify assistant path is gone**

Run:

```bash
rg -n "vision_assistant_udp|seekfree_assistant_camera|seekfree_assistant_interface|client_sender_enabled|assistant_udp|assistant_server|vision_thread_set_send_max_fps|vision_transport_set_send_max_fps" project/code project/user
```

Expected: no matches, except unrelated comments if intentionally kept.

- [ ] **Step 10: Build**

Run:

```bash
cd project/user && ./build.sh
```

Expected: build succeeds without missing symbol errors.

- [ ] **Step 11: Commit**

```bash
git add project/code project/user
git add -u project/code/driver/vision/vision_assistant_udp.h project/code/driver/vision/vision_assistant_udp.cpp
git commit -m "refactor: remove seekfree assistant vision sender"
```

### Task 2: Add PC-Side Unified UDP Parser Test

**Files:**
- Modify: `tools/pc_receiver_js/server.js`
- Create: `tools/pc_receiver_js/scripts/test_unified_udp_bundle.js`
- Modify: `tools/pc_receiver_js/package.json`

- [ ] **Step 1: Export parser helpers from `server.js`**

At the bottom of `tools/pc_receiver_js/server.js`, wrap startup code so tests can import helpers without binding ports:

```js
function startAll() {
  startUdpReceiver();
  startTcpReceiver();
  startHttpServer();
  startWebSocketServer();
}

if (require.main === module) {
  startAll();
}

module.exports = {
  parseHeader,
  onUdpMessage,
  latestByMode,
  getLatestStatus: () => latestStatus,
  buildTransportTelemetry
};
```

- [ ] **Step 2: Run import test and verify current behavior**

Run:

```bash
node -e "const s=require('./tools/pc_receiver_js/server.js'); console.log(typeof s.onUdpMessage)"
```

Expected: prints `function` and does not print any `listening on` messages.

- [ ] **Step 3: Create failing unified bundle test**

Create `tools/pc_receiver_js/scripts/test_unified_udp_bundle.js`:

```js
const assert = require('node:assert/strict');
const receiver = require('../server.js');

const MAGIC = 0x56535542;
const HEADER_SIZE = 32;

function packet({
  bundleId,
  itemIdx,
  itemTotal,
  chunkIdx,
  chunkTotal,
  itemType,
  imageMode = 255,
  width = 0,
  height = 0,
  imageFormat = 255,
  flags = 0,
  payload
}) {
  const body = Buffer.from(payload);
  const buf = Buffer.alloc(HEADER_SIZE + body.length);
  buf.writeUInt32BE(MAGIC, 0);
  buf.writeUInt32BE(bundleId >>> 0, 4);
  buf.writeUInt16BE(itemIdx, 8);
  buf.writeUInt16BE(itemTotal, 10);
  buf.writeUInt16BE(chunkIdx, 12);
  buf.writeUInt16BE(chunkTotal, 14);
  buf.writeUInt16BE(body.length, 16);
  buf.writeUInt8(itemType, 18);
  buf.writeUInt8(imageMode, 19);
  buf.writeUInt16BE(width, 20);
  buf.writeUInt16BE(height, 22);
  buf.writeUInt8(imageFormat, 24);
  buf.writeUInt8(flags, 25);
  buf.writeUInt16BE(0, 26);
  buf.writeUInt32BE(0, 28);
  body.copy(buf, HEADER_SIZE);
  return buf;
}

receiver.onUdpMessage(packet({
  bundleId: 7,
  itemIdx: 2,
  itemTotal: 3,
  chunkIdx: 0,
  chunkTotal: 1,
  itemType: 3,
  imageMode: 1,
  width: 160,
  height: 120,
  imageFormat: 0,
  payload: Buffer.from([0xff, 0xd8, 0xff, 0xd9])
}));

assert.notEqual(receiver.latestByMode[1].frameId, 7, 'image must not publish before matching status arrives');

receiver.onUdpMessage(packet({
  bundleId: 7,
  itemIdx: 1,
  itemTotal: 3,
  chunkIdx: 0,
  chunkTotal: 1,
  itemType: 2,
  payload: Buffer.from(JSON.stringify({
    ts_ms: 123,
    udp_web_max_fps: 60,
    left_boundary: [[1, 2]],
    right_boundary: [[3, 4]]
  }) + '\n')
}));

assert.notEqual(receiver.latestByMode[1].frameId, 7, 'status alone still must not publish before manifest arrives');

receiver.onUdpMessage(packet({
  bundleId: 7,
  itemIdx: 0,
  itemTotal: 3,
  chunkIdx: 0,
  chunkTotal: 1,
  itemType: 1,
  payload: Buffer.from(JSON.stringify({
    required_image_modes: [1],
    optional_image_modes: []
  }) + '\n')
}));

assert.equal(receiver.latestByMode[1].frameId, 7);
assert.equal(receiver.latestByMode[1].width, 160);
assert.equal(receiver.getLatestStatus().ts_ms, 123);
assert.deepEqual(receiver.getLatestStatus().left_boundary, [[1, 2]]);
console.log('unified udp bundle parser ok');
```

- [ ] **Step 4: Run test to verify it fails**

Run:

```bash
node tools/pc_receiver_js/scripts/test_unified_udp_bundle.js
```

Expected: FAIL because `VSUB` packets are ignored by the old `onUdpMessage()`.

- [ ] **Step 5: Add npm script**

In `tools/pc_receiver_js/package.json`, add:

```json
"test:unified-udp": "node scripts/test_unified_udp_bundle.js"
```

- [ ] **Step 6: Commit**

```bash
git add tools/pc_receiver_js/server.js tools/pc_receiver_js/scripts/test_unified_udp_bundle.js tools/pc_receiver_js/package.json
git commit -m "test: cover unified udp bundle receive path"
```

### Task 3: Implement PC-Side Bundle Reassembly

**Files:**
- Modify: `tools/pc_receiver_js/server.js`
- Test: `tools/pc_receiver_js/scripts/test_unified_udp_bundle.js`

- [ ] **Step 1: Add bundle constants and state**

Near existing UDP constants:

```js
const BUNDLE_MAGIC = 0x56535542; // VSUB
const BUNDLE_HEADER_SIZE = 32;
const ITEM_TYPE_MANIFEST_JSON = 1;
const ITEM_TYPE_STATUS_JSON = 2;
const ITEM_TYPE_IMAGE = 3;
const IMAGE_MODE_NONE = 255;
const IMAGE_FORMAT_NONE = 255;

const inflightBundles = new Map();
let latestBundleId = -1;
```

- [ ] **Step 2: Add parser and publish helpers**

Add below `parseHeader()`:

```js
function parseBundleHeader(buf) {
  if (buf.length < BUNDLE_HEADER_SIZE) return null;
  return {
    magic: buf.readUInt32BE(0),
    bundleId: buf.readUInt32BE(4),
    itemIdx: buf.readUInt16BE(8),
    itemTotal: buf.readUInt16BE(10),
    chunkIdx: buf.readUInt16BE(12),
    chunkTotal: buf.readUInt16BE(14),
    payloadLen: buf.readUInt16BE(16),
    itemType: buf.readUInt8(18),
    imageMode: buf.readUInt8(19),
    width: buf.readUInt16BE(20),
    height: buf.readUInt16BE(22),
    imageFormat: buf.readUInt8(24),
    flags: buf.readUInt8(25)
  };
}

function bundleKeyForItem(hdr) {
  return `${hdr.itemIdx}:${hdr.itemType}:${hdr.imageMode}`;
}

function isBundleNewer(nextBundleId) {
  if (latestBundleId < 0) return true;
  return isFrameNewer(latestBundleId, nextBundleId) || isLikelyFrameCounterReset(latestBundleId, nextBundleId);
}

function parseJsonItem(item) {
  if (!item || !item.complete || !item.payload) return null;
  try {
    return JSON.parse(item.payload.toString('utf8').trim());
  } catch (_) {
    return null;
  }
}

function publishBundle(bundle) {
  if (!isBundleNewer(bundle.bundleId)) return;
  const completedItems = Array.from(bundle.items.values()).filter((item) => item.complete);
  const manifestItem = completedItems.find((item) => item.itemType === ITEM_TYPE_MANIFEST_JSON);
  const statusItem = completedItems.find((item) => item.itemType === ITEM_TYPE_STATUS_JSON);
  const manifest = parseJsonItem(manifestItem);
  const status = parseJsonItem(statusItem);
  if (!manifest || !status) return;
  const requiredModes = Array.isArray(manifest.required_image_modes)
    ? manifest.required_image_modes.map((value) => Number(value)).filter((value) => Number.isInteger(value))
    : [];
  for (const mode of requiredModes) {
    const imageItem = completedItems.find((item) => item.itemType === ITEM_TYPE_IMAGE && item.imageMode === mode);
    if (!imageItem) return;
  }
  latestStatus = status;
  latestBundleId = bundle.bundleId >>> 0;
  broadcastWs('status', latestStatus);
  const nowMs = Date.now();
  for (const item of completedItems) {
    if (item.itemType !== ITEM_TYPE_IMAGE) continue;
    if (!(item.imageMode in latestByMode)) continue;
    const format = sanitizeImageFormat(item.imageFormat);
    const wireBytes = item.payload.length + (item.chunkTotal * BUNDLE_HEADER_SIZE);
    latestByMode[item.imageMode] = {
      image: item.payload,
      frameId: bundle.bundleId >>> 0,
      updatedAtMs: nowMs,
      width: item.width,
      height: item.height,
      mode: item.imageMode,
      format,
      wireBytes
    };
    broadcastWs('frame', {
      mode: item.imageMode,
      frameId: bundle.bundleId >>> 0,
      width: item.width,
      height: item.height,
      format
    });
    udpFrameEvents.push({ ts: nowMs, mode: item.imageMode, wireBytes });
  }
}
```

- [ ] **Step 3: Add bundle message handler**

Add before `onUdpMessage()`:

```js
function onUnifiedUdpMessage(msg) {
  const hdr = parseBundleHeader(msg);
  if (!hdr || hdr.magic !== BUNDLE_MAGIC) return false;
  if (hdr.itemTotal === 0 || hdr.itemIdx >= hdr.itemTotal) return true;
  if (hdr.chunkTotal === 0 || hdr.chunkIdx >= hdr.chunkTotal) return true;
  if (BUNDLE_HEADER_SIZE + hdr.payloadLen > msg.length) return true;
  if (!isBundleNewer(hdr.bundleId)) return true;

  let bundle = inflightBundles.get(hdr.bundleId);
  if (!bundle) {
    bundle = {
      bundleId: hdr.bundleId >>> 0,
      itemTotal: hdr.itemTotal,
      items: new Map(),
      ts: Date.now()
    };
    inflightBundles.set(hdr.bundleId, bundle);
  }

  const key = bundleKeyForItem(hdr);
  let item = bundle.items.get(key);
  if (!item) {
    item = {
      itemType: hdr.itemType,
      imageMode: hdr.imageMode,
      imageFormat: hdr.imageFormat,
      width: hdr.width,
      height: hdr.height,
      chunkTotal: hdr.chunkTotal,
      chunks: new Map(),
      complete: false,
      payload: null
    };
    bundle.items.set(key, item);
  }

  item.chunks.set(hdr.chunkIdx, msg.subarray(BUNDLE_HEADER_SIZE, BUNDLE_HEADER_SIZE + hdr.payloadLen));
  bundle.ts = Date.now();

  if (item.chunks.size === item.chunkTotal) {
    const ordered = [];
    for (let i = 0; i < item.chunkTotal; i += 1) {
      const chunk = item.chunks.get(i);
      if (!chunk) return true;
      ordered.push(chunk);
    }
    item.payload = Buffer.concat(ordered);
    item.complete = true;
  }

  publishBundle(bundle);
  if (latestBundleId === (hdr.bundleId >>> 0)) inflightBundles.delete(hdr.bundleId);
  return true;
}
```

- [ ] **Step 4: Route new packets before old parser**

At the top of `onUdpMessage(msg)`:

```js
  udpByteEvents.push({ ts: Date.now(), bytes: msg.length });
  if (onUnifiedUdpMessage(msg)) return;
```

Then remove the existing first `udpByteEvents.push(...)` line from the old parser body so bytes are not double-counted.

- [ ] **Step 5: Clean stale bundles**

In `cleanupInflight()` add:

```js
  for (const [bundleId, entry] of inflightBundles.entries()) {
    if (now - entry.ts > 300) {
      inflightBundles.delete(bundleId);
    }
  }
```

- [ ] **Step 6: Run receiver tests**

Run:

```bash
npm --prefix tools/pc_receiver_js run test:unified-udp
```

Expected: PASS and prints `unified udp bundle parser ok`.

- [ ] **Step 7: Commit**

```bash
git add tools/pc_receiver_js/server.js tools/pc_receiver_js/scripts/test_unified_udp_bundle.js tools/pc_receiver_js/package.json
git commit -m "feat: receive unified udp vision bundles"
```

### Task 4: Refactor Status JSON Builder On Lower Side

**Files:**
- Modify: `project/code/driver/vision/vision_transport.cpp`

- [ ] **Step 1: Extract `build_web_status_json()`**

Change:

```cpp
static void send_tcp_status()
{
    if (!g_tcp_enabled.load() || !g_tcp_ready)
    {
        return;
    }
```

To:

```cpp
static bool build_web_status_json(std::string *json_out)
{
    if (json_out == nullptr)
    {
        return false;
    }
```

At both places where the old function sends TCP:

```cpp
        line += "}";
        line += "\n";
        tcp_client_send_data(reinterpret_cast<const uint8 *>(line.data()), static_cast<uint32>(line.size()));
        return;
```

Replace with:

```cpp
        line += "}";
        line += "\n";
        *json_out = line;
        return true;
```

At the final send site replace:

```cpp
    tcp_client_send_data(reinterpret_cast<const uint8 *>(line.data()), static_cast<uint32>(line.size()));
```

With:

```cpp
    *json_out = line;
    return true;
```

- [ ] **Step 2: Remove TCP status sender**

Delete the old `send_tcp_status()` function entirely after `build_web_status_json()` exists. The live web path must have no TCP status fallback.

Run:

```bash
rg -n "send_tcp_status|tcp_client_send_data" project/code/driver/vision/vision_transport.cpp
```

Expected: no matches.

- [ ] **Step 3: Build**

Run:

```bash
cd project/user && ./build.sh
```

Expected: build succeeds with no missing return warnings for `build_web_status_json`.

- [ ] **Step 4: Commit**

```bash
git add project/code/driver/vision/vision_transport.cpp
git commit -m "refactor: extract web status json builder"
```

### Task 5: Add Lower-Side Unified UDP Bundle Sender

**Files:**
- Modify: `project/code/driver/vision/vision_transport.cpp`

- [ ] **Step 1: Add constants and item model**

Near existing UDP constants:

```cpp
constexpr uint32 kBundleMagic = 0x56535542;
constexpr uint32 kBundleHeaderSize = 32;
constexpr uint8 kBundleItemManifestJson = 1;
constexpr uint8 kBundleItemStatusJson = 2;
constexpr uint8 kBundleItemImage = 3;
constexpr uint8 kBundleImageModeNone = 255;
constexpr uint8 kBundleImageFormatNone = 255;

#pragma pack(push, 1)
struct udp_bundle_chunk_header_t
{
    uint32 magic;
    uint32 bundle_id;
    uint16 item_idx;
    uint16 item_total;
    uint16 chunk_idx;
    uint16 chunk_total;
    uint16 payload_len;
    uint8 item_type;
    uint8 image_mode;
    uint16 width;
    uint16 height;
    uint8 image_format;
    uint8 flags;
    uint16 reserved;
    uint32 reserved2;
};
#pragma pack(pop)
static_assert(sizeof(udp_bundle_chunk_header_t) == kBundleHeaderSize,
              "udp bundle header size must match JS receiver");

struct udp_bundle_item_t
{
    uint8 item_type = 0;
    uint8 image_mode = kBundleImageModeNone;
    uint16 width = 0;
    uint16 height = 0;
    uint8 image_format = kBundleImageFormatNone;
    const uint8 *data = nullptr;
    uint32 size = 0;
};
```

- [ ] **Step 2: Replace old frame sender with manifest builder and bundle send helper**

Add near the UDP send helpers:

```cpp
static std::string build_udp_bundle_manifest_json(const std::vector<uint8> &required_modes,
                                                  const std::vector<uint8> &optional_modes)
{
    std::string json;
    json.reserve(96);
    json += "{\"required_image_modes\":[";
    for (size_t i = 0; i < required_modes.size(); ++i)
    {
        if (i > 0)
        {
            json += ",";
        }
        json += std::to_string(static_cast<int>(required_modes[i]));
    }
    json += "],\"optional_image_modes\":[";
    for (size_t i = 0; i < optional_modes.size(); ++i)
    {
        if (i > 0)
        {
            json += ",";
        }
        json += std::to_string(static_cast<int>(optional_modes[i]));
    }
    json += "]}\n";
    return json;
}
```

Delete the old `send_udp_frame()` helper after there are no remaining callers. Add this bundle sender after the manifest builder:

```cpp
static void send_udp_bundle(const std::vector<udp_bundle_item_t> &items)
{
    if (!g_udp_ready || items.empty())
    {
        return;
    }

    const uint32 bundle_id = g_udp_frame_id.fetch_add(1);
    const uint32 max_chunk_data = kMaxUdpPayload - kBundleHeaderSize;
    std::vector<uint8> packet;
    packet.resize(kMaxUdpPayload);

    static uint32 window_frames = 0;
    static uint64 window_start_us = 0;
    const uint64 now = now_us();
    if (window_start_us == 0)
    {
        window_start_us = now;
    }
    for (const udp_bundle_item_t &item : items)
    {
        if (item.item_type == kBundleItemImage && item.image_mode <= 2)
        {
            ++window_frames;
        }
    }
    const uint64 elapsed_us = now - window_start_us;
    if (elapsed_us >= 1000000ULL)
    {
        const uint32 fps = static_cast<uint32>(
            (static_cast<uint64>(window_frames) * 1000000ULL + elapsed_us / 2ULL) / elapsed_us);
        g_udp_tx_fps.store(fps);
        window_frames = 0;
        window_start_us = now;
    }

    for (uint16 item_idx = 0; item_idx < static_cast<uint16>(items.size()); ++item_idx)
    {
        const udp_bundle_item_t &item = items[item_idx];
        if (item.data == nullptr || item.size == 0)
        {
            continue;
        }

        const uint32 chunk_total_u32 = (item.size + max_chunk_data - 1U) / max_chunk_data;
        const uint16 chunk_total = static_cast<uint16>(std::min<uint32>(chunk_total_u32, 65535U));
        for (uint16 chunk_idx = 0; chunk_idx < chunk_total; ++chunk_idx)
        {
            const uint32 offset = static_cast<uint32>(chunk_idx) * max_chunk_data;
            const uint32 remain = item.size - offset;
            const uint32 payload_len = (remain > max_chunk_data) ? max_chunk_data : remain;

            udp_bundle_chunk_header_t hdr{};
            hdr.magic = htonl(kBundleMagic);
            hdr.bundle_id = htonl(bundle_id);
            hdr.item_idx = htons(item_idx);
            hdr.item_total = htons(static_cast<uint16>(items.size()));
            hdr.chunk_idx = htons(chunk_idx);
            hdr.chunk_total = htons(chunk_total);
            hdr.payload_len = htons(static_cast<uint16>(payload_len));
            hdr.item_type = item.item_type;
            hdr.image_mode = item.image_mode;
            hdr.width = htons(item.width);
            hdr.height = htons(item.height);
            hdr.image_format = item.image_format;
            hdr.flags = 0;
            hdr.reserved = 0;
            hdr.reserved2 = 0;

            std::memcpy(packet.data(), &hdr, sizeof(hdr));
            std::memcpy(packet.data() + sizeof(hdr), item.data + offset, payload_len);
            udp_send_data(packet.data(), payload_len + sizeof(hdr));
        }
    }
}
```

- [ ] **Step 3: Replace split send in `vision_transport_send_step()`**

Inside `if (try_acquire_udp_send_slot())`, replace the current per-image `send_udp_frame(...)` calls plus `send_tcp_status()` with:

```cpp
        std::string status_json;
        std::vector<uint8> gray_image;
        std::vector<uint8> binary_image;
        std::vector<uint8> rgb_image;
        std::vector<uint8> roi64_image;
        int gray_width = 0;
        int gray_height = 0;
        int binary_width = 0;
        int binary_height = 0;
        int rgb_width = 0;
        int rgb_height = 0;
        int roi64_width = 0;
        int roi64_height = 0;
        uint8 gray_mode = 0;
        uint8 binary_mode = 0;
        uint8 rgb_mode = 0;
        uint8 roi64_mode = 0;
        std::vector<uint8> required_image_modes;
        std::vector<uint8> optional_image_modes;
        std::vector<udp_bundle_item_t> bundle_items;
        bool required_image_failed = false;

        if (!g_udp_enabled.load() || !build_web_status_json(&status_json) || status_json.empty())
        {
            return;
        }

        if (g_udp_enabled.load() && g_vision_runtime_config.udp_web_send_gray_jpeg)
        {
            const vision_web_image_format_enum format =
                sanitize_web_image_format(g_vision_runtime_config.udp_web_gray_image_format);
            if (build_gray_image(&gray_image, &gray_width, &gray_height, &gray_mode))
            {
                required_image_modes.push_back(gray_mode);
                bundle_items.push_back({kBundleItemImage, gray_mode, static_cast<uint16>(gray_width), static_cast<uint16>(gray_height),
                                        static_cast<uint8>(format), gray_image.data(), static_cast<uint32>(gray_image.size())});
            }
            else
            {
                required_image_failed = true;
            }
        }

        if (g_udp_enabled.load() && g_vision_runtime_config.udp_web_send_binary_jpeg)
        {
            const vision_web_image_format_enum format =
                sanitize_web_image_format(g_vision_runtime_config.udp_web_binary_image_format);
            if (build_binary_image(&binary_image, &binary_width, &binary_height, &binary_mode))
            {
                required_image_modes.push_back(binary_mode);
                bundle_items.push_back({kBundleItemImage, binary_mode, static_cast<uint16>(binary_width), static_cast<uint16>(binary_height),
                                        static_cast<uint8>(format), binary_image.data(), static_cast<uint32>(binary_image.size())});
            }
            else
            {
                required_image_failed = true;
            }
        }

        if (g_udp_enabled.load() && g_vision_runtime_config.udp_web_send_rgb_jpeg)
        {
            const vision_web_image_format_enum format =
                sanitize_web_image_format(g_vision_runtime_config.udp_web_rgb_image_format);
            if (build_rgb_image(&rgb_image, &rgb_width, &rgb_height, &rgb_mode))
            {
                required_image_modes.push_back(rgb_mode);
                bundle_items.push_back({kBundleItemImage, rgb_mode, static_cast<uint16>(rgb_width), static_cast<uint16>(rgb_height),
                                        static_cast<uint8>(format), rgb_image.data(), static_cast<uint32>(rgb_image.size())});
            }
            else
            {
                required_image_failed = true;
            }
        }

        if (g_udp_enabled.load())
        {
            const vision_web_image_format_enum format =
                sanitize_web_image_format(g_vision_runtime_config.udp_web_rgb_image_format);
            if (build_roi64_image(&roi64_image, &roi64_width, &roi64_height, &roi64_mode))
            {
                optional_image_modes.push_back(roi64_mode);
                bundle_items.push_back({kBundleItemImage, roi64_mode, static_cast<uint16>(roi64_width), static_cast<uint16>(roi64_height),
                                        static_cast<uint8>(format), roi64_image.data(), static_cast<uint32>(roi64_image.size())});
            }
        }

        if (required_image_failed)
        {
            return;
        }

        std::string manifest_json = build_udp_bundle_manifest_json(required_image_modes, optional_image_modes);
        bundle_items.insert(bundle_items.begin(),
                            {kBundleItemStatusJson,
                             kBundleImageModeNone,
                             0,
                             0,
                             kBundleImageFormatNone,
                             reinterpret_cast<const uint8 *>(status_json.data()),
                             static_cast<uint32>(status_json.size())});
        bundle_items.insert(bundle_items.begin(),
                            {kBundleItemManifestJson,
                             kBundleImageModeNone,
                             0,
                             0,
                             kBundleImageFormatNone,
                             reinterpret_cast<const uint8 *>(manifest_json.data()),
                             static_cast<uint32>(manifest_json.size())});
        send_udp_bundle(bundle_items);
```

- [ ] **Step 4: Build**

Run:

```bash
! rg -n "send_udp_frame|send_tcp_status|tcp_client_send_data" project/code/driver/vision/vision_transport.cpp
cd project/user && ./build.sh
```

Expected: `rg` finds no matches, and build succeeds.

- [ ] **Step 5: Commit**

```bash
git add project/code/driver/vision/vision_transport.cpp
git commit -m "feat: send web status and images in udp bundles"
```

### Task 6: Disable TCP For Web Live Path In Config

**Files:**
- Modify: `project/user/smartcar_config.toml`

- [ ] **Step 1: Change testing profile**

In `project/user/smartcar_config.toml`, set:

```toml
[vision.runtime.web]
enabled = true
max_fps = 60
send_gray_jpeg = true
gray_image_format = 0
send_binary_jpeg = false
binary_image_format = 0
send_rgb_jpeg = false
rgb_image_format = 1
data_profile = 1
tcp_enabled = false
server_ip = "192.168.3.9"
video_port = 10000
meta_port = 0
```

- [ ] **Step 2: Keep boundary fields available in minimal profile**

If `data_profile = 1` does not include the fields needed for boundary display, set `data_profile = 0` for route debugging, but keep `tcp_enabled = false` and `meta_port = 0`.

- [ ] **Step 3: Build and run receiver**

Run:

```bash
cd project/user && ./build.sh
npm --prefix tools/pc_receiver_js run test:unified-udp
```

Expected: build succeeds and JS test passes.

- [ ] **Step 4: Commit**

```bash
git add project/user/smartcar_config.toml
git commit -m "config: use unified udp web transport"
```

### Task 7: Live Validation And Load Measurement

**Files:**
- Modify only if measurements expose a defect.

- [ ] **Step 1: Start PC receiver**

Run:

```bash
HTTP_PORT=9090 UDP_PORT=10000 TCP_PORT=10001 npm --prefix tools/pc_receiver_js start
```

Expected: receiver logs UDP and HTTP listeners. TCP listener may still start for compatibility, but no live status should depend on it.

- [ ] **Step 2: Run board and observe telemetry**

Open:

```text
http://localhost:9090
```

Expected:
- `rx_udp_gray_fps` approaches configured `max_fps`.
- `capture_thread_fps` remains near camera processing FPS.
- Boundary overlays update with the same visual frame; no obvious one-frame lag when moving the car or waving a marker.

- [ ] **Step 3: Compare FPS profiles**

Test these values in `project/user/smartcar_config.toml`:

```toml
max_fps = 30
max_fps = 60
max_fps = 90
max_fps = 110
```

For each run, record:
- `cpu_usage_percent`
- `perf_total_us`
- `rx_udp_mbps`
- `rx_udp_gray_fps`
- Whether boundaries visually stay aligned.

- [ ] **Step 4: Accept/reject criteria**

Accept the unified UDP path if:
- At 60 FPS, receiver frame rate is stable within 10 percent of target.
- CPU usage does not increase more than 5 percentage points compared with split UDP/TCP at the same image format and FPS.
- Boundary/image mismatch is no longer reproducible in normal live operation.

Do not run 110 FPS as default unless:
- CPU has at least 20 percent idle margin.
- `rx_udp_gray_fps` is stable above 100.
- No visible packet-loss stutter occurs during steering/motor load.

- [ ] **Step 5: Commit any measurement notes**

If a note file is useful, create `docs/vision-unified-udp-measurements.md` with the measured table and commit:

```bash
git add docs/vision-unified-udp-measurements.md
git commit -m "docs: record unified udp transport measurements"
```

## Self-Review

Spec coverage:
- UDP-only transport: covered by Tasks 3, 4, 5, and 6.
- SeekFree assistant cleanup: covered by Task 1.
- Frame-rate control: keeps existing `udp_web_max_fps` send slot and validates 30/60/90/110 in Task 7.
- Send latest result at each transmission flag point: Task 5 builds manifest, status, and images inside one acquired send slot.
- Boundary/image consistency: bundle publish rule requires manifest, status, and every manifest-required image mode before PC updates.
- CPU comparison: feasibility section and Task 7 define metrics and accept/reject criteria.

Placeholder scan:
- No `TBD`, `TODO`, or undefined follow-up step remains.

Type consistency:
- `bundle_id` maps to PC-side `bundleId`.
- `item_type` values match `ITEM_TYPE_MANIFEST_JSON`, `ITEM_TYPE_STATUS_JSON`, and `ITEM_TYPE_IMAGE`.
- `image_mode` keeps existing mode values `0..3`.
- `kBundleHeaderSize` is 32 bytes in both C++ and JS; C++ has a `static_assert` to catch drift.
