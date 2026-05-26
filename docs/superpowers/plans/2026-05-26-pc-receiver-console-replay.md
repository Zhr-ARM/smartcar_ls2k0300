# PC Receiver Console Replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild `tools/pc_receiver_js` into a polished single-console UI and replace the separate video playback UI with source-data recording that replays through the same live receiver page.

**Architecture:** Keep `index.html` as the single operator console, but split reusable browser logic into focused modules under `public/` so live mode and replay mode share rendering. Store recordings as timestamped status snapshots plus source image frames per mode (`gray`, `binary`, `rgb`, `roi64`) instead of canvas videos. The server remains compatible with existing saved canvas-video recordings while adding source-frame save/load endpoints.

**Tech Stack:** Node.js HTTP server, browser HTML/CSS/JS, Canvas 2D, Playwright, existing `npm run test:mock:pipeline` fixture workflow.

---

## File Structure

- Modify `tools/pc_receiver_js/server.js`
  - Add source-frame recording persistence to `writeRecordingFiles()`.
  - Extend `/api/recordings/load` to return source frame metadata and frame URLs.
  - Keep old `videos` metadata and `/api/recordings/file` support for backward compatibility.
- Modify `tools/pc_receiver_js/public/index.html`
  - Replace the current visual treatment with a more refined industrial dashboard.
  - Collapse the middle state area to two main state cards: element state and bypass/target-board state.
  - Rebalance the right rail into compact telemetry, medium-density speed summary, and readable PID summary.
  - Add replay mode controls into the same page.
- Create `tools/pc_receiver_js/public/receiver_frame_source.js`
  - Own live frame fetching and replay frame lookup.
  - Expose a tiny API that returns image URLs/Blobs for the existing canvas renderers.
- Create `tools/pc_receiver_js/public/receiver_recording.js`
  - Own browser-side source-data recording, persistence payload creation, saved-folder loading, and replay clock state.
- Modify `tools/pc_receiver_js/public/playback.html`
  - Convert to a compatibility redirect/loading page that opens `index.html?replay=<folder>` or transfers legacy payload to `index.html`.
- Modify `tools/pc_receiver_js/public/playback_app.js`
  - Stop maintaining duplicate UI rendering; keep only compatibility handoff code or remove script usage from `playback.html`.
- Modify `tools/pc_receiver_js/scripts/smoke_mock_pipeline.js`
  - Add optional assertions for source recording save/load if needed by the test task.
- Create `tools/pc_receiver_js/scripts/test_source_recording.js`
  - Unit-style Node test for source-frame payload persistence and `/api/recordings/load` shape.

## Task 1: Source Recording Persistence Contract

**Files:**
- Modify: `tools/pc_receiver_js/server.js`
- Create: `tools/pc_receiver_js/scripts/test_source_recording.js`
- Test: `tools/pc_receiver_js/scripts/test_source_recording.js`

- [ ] **Step 1: Write the persistence test**

Create `tools/pc_receiver_js/scripts/test_source_recording.js` with:

```js
const assert = require('node:assert/strict');
const fs = require('node:fs');
const http = require('node:http');
const path = require('node:path');
const { spawn } = require('node:child_process');

const ROOT = path.resolve(__dirname, '..');
const folder = `source_recording_test_${Date.now()}`;
const httpPort = 19390;
const imageB64 = Buffer.from('fake-jpeg-bytes').toString('base64');

function requestJson(method, pathname, body) {
  return new Promise((resolve, reject) => {
    const text = body ? JSON.stringify(body) : '';
    const req = http.request({
      host: '127.0.0.1',
      port: httpPort,
      method,
      path: pathname,
      headers: {
        'Content-Type': 'application/json',
        'Content-Length': Buffer.byteLength(text)
      }
    }, (res) => {
      const chunks = [];
      res.on('data', (chunk) => chunks.push(chunk));
      res.on('end', () => {
        const raw = Buffer.concat(chunks).toString('utf8');
        try {
          resolve({ statusCode: res.statusCode, json: raw ? JSON.parse(raw) : {} });
        } catch (err) {
          reject(err);
        }
      });
    });
    req.on('error', reject);
    if (text) req.write(text);
    req.end();
  });
}

async function waitForServer() {
  for (let i = 0; i < 40; i += 1) {
    try {
      const result = await requestJson('GET', '/api/status');
      if (result.statusCode === 200) return;
    } catch (_) {}
    await new Promise((resolve) => setTimeout(resolve, 100));
  }
  throw new Error('server did not start');
}

(async () => {
  const child = spawn(process.execPath, ['server.js'], {
    cwd: ROOT,
    env: {
      ...process.env,
      BIND_HOST: '127.0.0.1',
      UDP_PORT: '19300',
      TCP_PORT: '19301',
      HTTP_PORT: String(httpPort)
    },
    stdio: ['ignore', 'pipe', 'pipe']
  });

  try {
    await waitForServer();
    const save = await requestJson('POST', '/api/recordings/save', {
      folder,
      recorded_at_ms: 1000,
      duration_ms: 120,
      frame_count: 2,
      statuses: [
        { client_ts_ms: 1000, status: { route_main_state: 0, target_board_state: 1 } },
        { client_ts_ms: 1120, status: { route_main_state: 2, target_board_state: 3 } }
      ],
      source_frames: {
        gray: [
          { client_ts_ms: 1000, frame_id: 1, width: 160, height: 120, mime: 'image/jpeg', data_b64: imageB64 },
          { client_ts_ms: 1120, frame_id: 2, width: 160, height: 120, mime: 'image/jpeg', data_b64: imageB64 }
        ],
        binary: [
          { client_ts_ms: 1000, frame_id: 1, width: 160, height: 120, mime: 'image/jpeg', data_b64: imageB64 }
        ]
      },
      session_meta: { recording_kind: 'source_frames' }
    });
    assert.equal(save.statusCode, 200);
    assert.equal(save.json.ok, true);

    const load = await requestJson('GET', `/api/recordings/load?folder=${encodeURIComponent(folder)}`);
    assert.equal(load.statusCode, 200);
    assert.equal(load.json.ok, true);
    assert.equal(load.json.status.frame_count, 2);
    assert.equal(load.json.meta.source_frames.gray.length, 2);
    assert.match(load.json.source_frames.gray[0].url, /^\/api\/recordings\/file\?/);
    assert.equal(load.json.source_frames.gray[0].client_ts_ms, 1000);
    assert.equal(load.json.source_frames.binary[0].mode, 'binary');

    console.log('[test_source_recording] PASS');
  } finally {
    child.kill('SIGTERM');
    fs.rmSync(path.join(ROOT, 'recordings', folder), { recursive: true, force: true });
  }
})().catch((err) => {
  console.error(err);
  process.exit(1);
});
```

- [ ] **Step 2: Run the test and verify it fails**

Run:

```bash
cd tools/pc_receiver_js
node scripts/test_source_recording.js
```

Expected: FAIL because `source_frames` is not persisted or returned by `/api/recordings/load`.

- [ ] **Step 3: Implement source-frame persistence**

In `tools/pc_receiver_js/server.js`, update `writeRecordingFiles()` to write `source_frames/<mode>/<index>.<ext>` files and `meta.source_frames`. Add helpers near `extByMime()`:

```js
function imageExtByMime(mime) {
  if (mime === 'image/png') return '.png';
  if (mime === 'image/bmp') return '.bmp';
  if (mime === 'image/jpeg' || mime === 'image/jpg') return '.jpg';
  return '.bin';
}

function normalizeSourceMode(mode) {
  const text = String(mode || '').trim().toLowerCase();
  if (text === 'gray' || text === 'binary' || text === 'rgb' || text === 'roi64') return text;
  return '';
}
```

Inside `writeRecordingFiles()`, after video persistence and before `meta`, add:

```js
  const sourceFramesInput = payload.source_frames && typeof payload.source_frames === 'object'
    ? payload.source_frames
    : {};
  const sourceFrameMeta = {};
  for (const [rawMode, frames] of Object.entries(sourceFramesInput)) {
    const mode = normalizeSourceMode(rawMode);
    if (!mode || !Array.isArray(frames)) continue;
    const modeDir = path.join(folderPath, 'source_frames', mode);
    fs.mkdirSync(modeDir, { recursive: true });
    sourceFrameMeta[mode] = [];
    frames.forEach((frame, index) => {
      if (!frame || typeof frame.data_b64 !== 'string' || !frame.data_b64) return;
      const mime = typeof frame.mime === 'string' && frame.mime ? frame.mime : 'image/jpeg';
      const ext = imageExtByMime(mime);
      const seq = String(index).padStart(6, '0');
      const fileName = `${seq}${ext}`;
      fs.writeFileSync(path.join(modeDir, fileName), Buffer.from(frame.data_b64, 'base64'));
      sourceFrameMeta[mode].push({
        file: `source_frames/${mode}/${fileName}`,
        mode,
        mime,
        client_ts_ms: Number(frame.client_ts_ms) || 0,
        frame_id: Number.isFinite(Number(frame.frame_id)) ? Number(frame.frame_id) : index,
        width: Number(frame.width) || 0,
        height: Number(frame.height) || 0
      });
    });
  }
```

Add `source_frames: sourceFrameMeta` to `meta`.

- [ ] **Step 4: Return source frames from load endpoint**

In `/api/recordings/load`, after building `videos`, add:

```js
      const sourceFrames = {};
      if (meta && meta.source_frames && typeof meta.source_frames === 'object') {
        for (const [mode, frames] of Object.entries(meta.source_frames)) {
          if (!Array.isArray(frames)) continue;
          sourceFrames[mode] = frames
            .filter((item) => item && typeof item.file === 'string' && item.file)
            .map((item) => ({
              mode: item.mode || mode,
              client_ts_ms: Number(item.client_ts_ms) || 0,
              frame_id: Number.isFinite(Number(item.frame_id)) ? Number(item.frame_id) : -1,
              width: Number(item.width) || 0,
              height: Number(item.height) || 0,
              mime: item.mime || 'image/jpeg',
              url: `/api/recordings/file?folder=${encodeURIComponent(folder)}&name=${encodeURIComponent(item.file)}`
            }));
        }
      }
```

Change the response to:

```js
      sendJson(res, 200, { ok: true, folder, status, meta, videos, source_frames: sourceFrames });
```

Update `/api/recordings/file` validation to allow nested `source_frames/mode/file.jpg` names:

```js
      const rawName = String(reqUrl.searchParams.get('name') || '');
      const normalizedName = rawName.replace(/\\/g, '/');
      if (!folder || !normalizedName || normalizedName.includes('..') || normalizedName.startsWith('/')) {
        res.writeHead(400, { 'Content-Type': 'text/plain; charset=utf-8' });
        res.end('bad request');
        return;
      }
      if (!/^[A-Za-z0-9._/-]+$/.test(normalizedName)) {
        res.writeHead(400, { 'Content-Type': 'text/plain; charset=utf-8' });
        res.end('bad file name');
        return;
      }
      const filePath = path.join(folderPath, normalizedName);
      if (!filePath.startsWith(`${folderPath}${path.sep}`)) {
        res.writeHead(400, { 'Content-Type': 'text/plain; charset=utf-8' });
        res.end('bad file path');
        return;
      }
```

- [ ] **Step 5: Run the test and verify it passes**

Run:

```bash
cd tools/pc_receiver_js
node scripts/test_source_recording.js
```

Expected: PASS and output `[test_source_recording] PASS`.

- [ ] **Step 6: Commit**

```bash
git add tools/pc_receiver_js/server.js tools/pc_receiver_js/scripts/test_source_recording.js
git commit -m "feat: persist source-frame recordings"
```

## Task 2: Unified Frame Source and Replay Clock

**Files:**
- Create: `tools/pc_receiver_js/public/receiver_frame_source.js`
- Create: `tools/pc_receiver_js/public/receiver_recording.js`
- Modify: `tools/pc_receiver_js/public/index.html`

- [ ] **Step 1: Create frame source module**

Create `tools/pc_receiver_js/public/receiver_frame_source.js`:

```js
(() => {
  function nowUrl(url) {
    const join = url.includes('?') ? '&' : '?';
    return `${url}${join}t=${Date.now()}`;
  }

  function nearestFrame(frames, playbackMs) {
    if (!Array.isArray(frames) || frames.length < 1) return null;
    let best = frames[0];
    let bestDiff = Number.POSITIVE_INFINITY;
    for (const frame of frames) {
      const diff = Math.abs((Number(frame.client_ts_ms) || 0) - playbackMs);
      if (diff < bestDiff) {
        best = frame;
        bestDiff = diff;
      }
    }
    return best;
  }

  function createReceiverFrameSource(receiverCore) {
    let mode = 'live';
    let replayFrames = {};
    let replayStartMs = 0;
    let replayNowMs = 0;

    return {
      setLive() {
        mode = 'live';
        replayFrames = {};
        replayStartMs = 0;
        replayNowMs = 0;
      },
      setReplay(recording, playbackMs) {
        mode = 'replay';
        replayFrames = (recording && recording.source_frames) || {};
        replayStartMs = Number(recording && recording.recorded_at_ms) || 0;
        replayNowMs = Number(playbackMs) || 0;
      },
      setReplayTime(playbackMs) {
        replayNowMs = Number(playbackMs) || 0;
      },
      urlForMode(frameMode) {
        if (mode === 'live') return receiverCore.frameUrlForMode(frameMode);
        const frames = replayFrames[frameMode] || [];
        const frame = nearestFrame(frames, replayStartMs + replayNowMs);
        return frame && frame.url ? nowUrl(frame.url) : '';
      },
      frameMetaForMode(frameMode) {
        if (mode === 'live') return null;
        const frames = replayFrames[frameMode] || [];
        return nearestFrame(frames, replayStartMs + replayNowMs);
      },
      isReplay() {
        return mode === 'replay';
      }
    };
  }

  window.ReceiverFrameSource = { createReceiverFrameSource };
})();
```

- [ ] **Step 2: Create recording/replay module**

Create `tools/pc_receiver_js/public/receiver_recording.js`:

```js
(() => {
  const FRAME_MODES = ['gray', 'binary', 'rgb', 'roi64'];

  async function blobToBase64(blob) {
    const arrayBuffer = await blob.arrayBuffer();
    const bytes = new Uint8Array(arrayBuffer);
    let binary = '';
    for (let i = 0; i < bytes.length; i += 1) binary += String.fromCharCode(bytes[i]);
    return btoa(binary);
  }

  async function fetchFrameForRecording(mode, receiverCore) {
    const response = await fetch(receiverCore.frameUrlForMode(mode), { cache: 'no-store' });
    if (!response.ok) return null;
    const blob = await response.blob();
    return {
      mode,
      client_ts_ms: Date.now(),
      frame_id: -1,
      width: 0,
      height: 0,
      mime: blob.type || 'image/jpeg',
      data_b64: await blobToBase64(blob)
    };
  }

  function createSourceRecorder(receiverCore, getStatus) {
    let state = null;

    return {
      active() {
        return !!(state && state.active);
      },
      start() {
        state = {
          active: true,
          startedAtMs: Date.now(),
          statusFrames: [],
          sourceFrames: { gray: [], binary: [], rgb: [], roi64: [] }
        };
      },
      async captureTick() {
        if (!state || !state.active) return;
        const status = getStatus();
        state.statusFrames.push({ client_ts_ms: Date.now(), status: JSON.parse(JSON.stringify(status || {})) });
        await Promise.all(FRAME_MODES.map(async (mode) => {
          const frame = await fetchFrameForRecording(mode, receiverCore);
          if (frame) state.sourceFrames[mode].push(frame);
        }));
      },
      stop() {
        if (!state) return null;
        state.active = false;
        const finished = state;
        state = null;
        return finished;
      }
    };
  }

  function buildPersistableSourceRecording(recording) {
    const savedAtMs = Date.now();
    const startedAtMs = Number(recording && recording.startedAtMs) || savedAtMs;
    const statusFrames = Array.isArray(recording && recording.statusFrames) ? recording.statusFrames : [];
    return {
      recorded_at_ms: startedAtMs,
      duration_ms: Math.max(0, savedAtMs - startedAtMs),
      frame_count: statusFrames.length,
      statuses: statusFrames,
      source_frames: (recording && recording.sourceFrames) || {},
      session_meta: {
        recording_kind: 'source_frames',
        recorded_views: [
          { key: 'gray', title: '灰度源图' },
          { key: 'binary', title: '二值源图' },
          { key: 'rgb', title: 'RGB 源图' },
          { key: 'roi64', title: 'ROI64 源图' }
        ]
      }
    };
  }

  function statusAt(recordingStatus, playbackMs) {
    const frames = recordingStatus && Array.isArray(recordingStatus.statuses) ? recordingStatus.statuses : [];
    if (frames.length < 1) return {};
    const startMs = Number(recordingStatus.recorded_at_ms) || Number(frames[0].client_ts_ms) || 0;
    const target = startMs + playbackMs;
    let best = frames[0];
    let bestDiff = Number.POSITIVE_INFINITY;
    for (const frame of frames) {
      const diff = Math.abs((Number(frame.client_ts_ms) || 0) - target);
      if (diff < bestDiff) {
        best = frame;
        bestDiff = diff;
      }
    }
    return best.status || {};
  }

  window.ReceiverRecording = {
    createSourceRecorder,
    buildPersistableSourceRecording,
    statusAt
  };
})();
```

- [ ] **Step 3: Load modules in index**

In `tools/pc_receiver_js/public/index.html`, add before the existing inline script:

```html
  <script src="/receiver_frame_source.js"></script>
  <script src="/receiver_recording.js"></script>
```

- [ ] **Step 4: Replace direct frame URLs with frame source**

Near existing `const receiverCore = window.SharedReceiverCore;`, add:

```js
    const frameSource = window.ReceiverFrameSource.createReceiverFrameSource(receiverCore);
```

In `pullFrames()`, replace:

```js
      rgbImg.src = receiverCore.frameUrlForMode('rgb');
```

with:

```js
      const rgbUrl = frameSource.urlForMode('rgb');
      if (rgbUrl) rgbImg.src = rgbUrl;
```

Replace the ROI line with:

```js
        const roiUrl = frameSource.urlForMode('roi64');
        if (roiUrl) roi64Img.src = roiUrl;
```

In `fetchGrayFrameDirect()`, replace `receiverCore.frameUrlForMode('gray')` with:

```js
        const grayUrl = frameSource.urlForMode('gray');
        if (!grayUrl) throw new Error('gray frame unavailable');
        const response = await fetch(grayUrl, { cache: 'no-store' });
```

- [ ] **Step 5: Run smoke test**

Run:

```bash
cd tools/pc_receiver_js
npm run test:mock:pipeline
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add tools/pc_receiver_js/public/index.html tools/pc_receiver_js/public/receiver_frame_source.js tools/pc_receiver_js/public/receiver_recording.js
git commit -m "feat: add unified receiver frame source"
```

## Task 3: Single-Page Source Replay Mode

**Files:**
- Modify: `tools/pc_receiver_js/public/index.html`
- Modify: `tools/pc_receiver_js/public/playback.html`
- Modify: `tools/pc_receiver_js/public/playback_app.js`

- [ ] **Step 1: Add replay controls to the top toolbar**

In `index.html`, add these buttons/labels beside the recording buttons:

```html
<button class="btn" id="replayToggleBtn" disabled>播放源数据</button>
<button class="btn" id="replayRestartBtn" disabled>回到开头</button>
<button class="btn" id="replayExitBtn" disabled>退出回放</button>
<div class="meta-pill replay-time-pill" id="replayTimeLabel">实时模式</div>
```

- [ ] **Step 2: Add replay state variables**

Near recording state declarations:

```js
    let replayRecording = null;
    let replayPlaying = false;
    let replayStartedAtMs = 0;
    let replayPlaybackMs = 0;
    let replayTimer = null;
```

Add DOM refs:

```js
    const replayToggleBtn = document.getElementById('replayToggleBtn');
    const replayRestartBtn = document.getElementById('replayRestartBtn');
    const replayExitBtn = document.getElementById('replayExitBtn');
    const replayTimeLabel = document.getElementById('replayTimeLabel');
```

- [ ] **Step 3: Add replay rendering functions**

Add below `refreshSavedFolders()`:

```js
    function replayDurationMs() {
      return Math.max(0, Number(replayRecording && replayRecording.status && replayRecording.status.duration_ms) || 0);
    }

    function formatMs(ms) {
      const total = Math.max(0, Number(ms) || 0) / 1000;
      const minutes = Math.floor(total / 60);
      const seconds = Math.floor(total % 60);
      const centi = Math.floor((total - Math.floor(total)) * 100);
      return `${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}.${String(centi).padStart(2, '0')}`;
    }

    function updateReplayUi() {
      const isReplay = !!replayRecording;
      replayToggleBtn.disabled = !isReplay;
      replayRestartBtn.disabled = !isReplay;
      replayExitBtn.disabled = !isReplay;
      replayToggleBtn.textContent = replayPlaying ? '暂停源数据' : '播放源数据';
      replayTimeLabel.textContent = isReplay
        ? `回放 ${formatMs(replayPlaybackMs)} / ${formatMs(replayDurationMs())}`
        : '实时模式';
    }

    function renderReplayFrame() {
      if (!replayRecording) return;
      const status = window.ReceiverRecording.statusAt(replayRecording.status, replayPlaybackMs);
      frameSource.setReplay(replayRecording, replayPlaybackMs);
      latestStatus = status;
      const renderStatus = getRenderStatus();
      updateTransportPanel(status);
      renderStatusPanels(renderStatus);
      renderPidStatusPanel(status);
      renderInferFrame(renderStatus || {});
      drawIpmBoundaries(renderStatus);
      drawTraceDirCharts(renderStatus);
      pullFrames();
      updateReplayUi();
    }

    function stopReplayTimer() {
      if (replayTimer) clearInterval(replayTimer);
      replayTimer = null;
    }

    function setReplayPlaying(nextPlaying) {
      replayPlaying = !!nextPlaying;
      stopReplayTimer();
      if (replayPlaying) {
        replayStartedAtMs = Date.now() - replayPlaybackMs;
        replayTimer = setInterval(() => {
          replayPlaybackMs = Math.min(replayDurationMs(), Date.now() - replayStartedAtMs);
          if (replayPlaybackMs >= replayDurationMs()) replayPlaying = false;
          renderReplayFrame();
          if (!replayPlaying) stopReplayTimer();
        }, 80);
      }
      updateReplayUi();
    }

    async function loadSourceReplayFolder(folder) {
      const j = await receiverCore.fetchJsonNoStore(`/api/recordings/load?folder=${encodeURIComponent(folder)}`);
      if (!j.ok) throw new Error(j.error || 'load failed');
      replayRecording = j;
      replayPlaybackMs = 0;
      frameSource.setReplay(replayRecording, replayPlaybackMs);
      setReplayPlaying(false);
      renderReplayFrame();
    }

    function exitReplayMode() {
      setReplayPlaying(false);
      replayRecording = null;
      replayPlaybackMs = 0;
      frameSource.setLive();
      updateReplayUi();
    }
```

- [ ] **Step 4: Gate live polling in replay mode**

At the top of `pullStatus()` and `pullFrames()`, add:

```js
      if (replayRecording) {
        renderReplayFrame();
        return;
      }
```

In `renderReplayFrame()`, call a new helper `pullFramesForCurrentSource()` instead of `pullFrames()` if recursion occurs. Extract existing `pullFrames()` body into `pullFramesForCurrentSource()`, then make `pullFrames()` only gate replay and delegate.

- [ ] **Step 5: Load replay from URL**

After event listener setup:

```js
    const pageParams = new URLSearchParams(window.location.search || '');
    const replayFolderFromUrl = pageParams.get('replay');
    if (replayFolderFromUrl) {
      loadSourceReplayFolder(replayFolderFromUrl).catch((err) => {
        recordingMeta.textContent = `加载源数据回放失败：${err}`;
      });
    }
```

- [ ] **Step 6: Wire replay controls**

Add:

```js
    replayToggleBtn.addEventListener('click', () => {
      setReplayPlaying(!replayPlaying);
    });
    replayRestartBtn.addEventListener('click', () => {
      replayPlaybackMs = 0;
      setReplayPlaying(false);
      renderReplayFrame();
    });
    replayExitBtn.addEventListener('click', () => {
      exitReplayMode();
    });
```

- [ ] **Step 7: Redirect playback page**

Replace `playback.html` body with a minimal compatibility shell:

```html
<body>
  <main class="handoff">
    <h1>正在打开统一回放界面</h1>
    <p id="handoffText">请稍候...</p>
  </main>
  <script src="/playback_app.js"></script>
</body>
```

Replace `playback_app.js` with:

```js
(() => {
  const params = new URLSearchParams(window.location.search || '');
  const folder = params.get('folder');
  if (folder) {
    window.location.replace(`/?replay=${encodeURIComponent(folder)}`);
    return;
  }
  const text = document.getElementById('handoffText');
  if (text) text.textContent = '旧的临时回放数据无法直接迁移，请先保存录制后再打开源数据回放。';
})();
```

- [ ] **Step 8: Run tests**

Run:

```bash
cd tools/pc_receiver_js
node scripts/test_source_recording.js
npm run test:mock:pipeline
```

Expected: both PASS.

- [ ] **Step 9: Commit**

```bash
git add tools/pc_receiver_js/public/index.html tools/pc_receiver_js/public/playback.html tools/pc_receiver_js/public/playback_app.js
git commit -m "feat: replay recordings in unified console"
```

## Task 4: Switch Recording From Canvas Video to Source Data

**Files:**
- Modify: `tools/pc_receiver_js/public/index.html`
- Test: `tools/pc_receiver_js/scripts/test_source_recording.js`

- [ ] **Step 1: Replace recorder state**

In `index.html`, replace `recordingState` use with:

```js
    const sourceRecorder = window.ReceiverRecording.createSourceRecorder(receiverCore, () => latestStatus);
    let recordingState = null;
```

- [ ] **Step 2: Change startRecording**

Replace `startRecording()` with:

```js
    async function startRecording() {
      if (sourceRecorder.active()) return;
      sourceRecorder.start();
      recordingState = {
        active: true,
        startedAtMs: Date.now(),
        statusFrames: []
      };
      updateRecordingUi();
    }
```

- [ ] **Step 3: Capture source frames on status tick**

In `pullStatus()`, replace the old `recordingState.statusFrames.push(...)` block with:

```js
        if (sourceRecorder.active()) {
          sourceRecorder.captureTick().catch((err) => {
            recordingMeta.textContent = `源数据录制采样失败：${err}`;
          });
        }
```

- [ ] **Step 4: Change stopRecording**

Replace `stopRecording()` with:

```js
    async function stopRecording() {
      if (!sourceRecorder.active()) return;
      const finished = sourceRecorder.stop();
      recordingState = null;
      const payload = window.ReceiverRecording.buildPersistableSourceRecording(finished);
      lastRecording = {
        savedAtMs: Date.now(),
        startedAtMs: payload.recorded_at_ms,
        durationMs: payload.duration_ms,
        frameCount: payload.frame_count,
        durationSec: (payload.duration_ms / 1000).toFixed(1),
        statusFrames: payload.statuses,
        sourceFrames: payload.source_frames,
        sourcePayload: payload
      };
      updateRecordingUi();
      if (confirm('源数据录制完成，是否立即保存并用统一界面回放？')) {
        await saveLastRecording();
        if (lastRecording.folder) await loadSourceReplayFolder(lastRecording.folder);
      }
    }
```

- [ ] **Step 5: Change persistence payload**

Replace `buildPersistableRecordingPayload(recording)` with:

```js
    async function buildPersistableRecordingPayload(recording) {
      if (recording && recording.sourcePayload) return recording.sourcePayload;
      return {
        recorded_at_ms: Number(recording && recording.startedAtMs) || Date.now(),
        duration_ms: Number(recording && recording.durationMs) || 0,
        frame_count: Number(recording && recording.frameCount) || 0,
        statuses: Array.isArray(recording && recording.statusFrames) ? recording.statusFrames : [],
        source_frames: (recording && recording.sourceFrames) || {},
        session_meta: { recording_kind: 'source_frames' }
      };
    }
```

- [ ] **Step 6: Change playLastRecording**

Replace `playLastRecording()` with:

```js
    async function playLastRecording() {
      if (!lastRecording) return;
      if (!lastRecording.folder) {
        await saveLastRecording();
      }
      if (!lastRecording.folder) throw new Error('保存源数据录制失败');
      await loadSourceReplayFolder(lastRecording.folder);
    }
```

- [ ] **Step 7: Keep old video helpers unused but harmless**

Do not delete `createCanvasRecorder`, `listRecordableCanvases`, and video playback helpers in this task unless all references are removed and smoke tests pass. Removing them is a later cleanup.

- [ ] **Step 8: Run tests**

Run:

```bash
cd tools/pc_receiver_js
node scripts/test_source_recording.js
npm run test:mock:pipeline
```

Expected: both PASS.

- [ ] **Step 9: Commit**

```bash
git add tools/pc_receiver_js/public/index.html
git commit -m "feat: record receiver source data"
```

## Task 5: State Machine Simplification and Right Rail Rebalance

**Files:**
- Modify: `tools/pc_receiver_js/public/index.html`
- Modify: `tools/pc_receiver_js/public/shared_receiver_core.js`

- [ ] **Step 1: Add target-board formatter**

In `shared_receiver_core.js`, add before the export object:

```js
  function formatTargetBoardState(state) {
    const n = Number(state);
    if (n === 0) return '未检测';
    if (n === 1) return '候选确认';
    if (n === 2) return '绕行准备';
    if (n === 3) return '绕行中';
    if (n === 4) return '绕行结束';
    if (!Number.isFinite(n)) return '未知';
    return `未知(${n})`;
  }
```

Add it to `window.SharedReceiverCore`.

- [ ] **Step 2: Replace middle state DOM**

In `index.html`, replace the always-visible `straightFocusPanel`, `crossFocusPanel`, `speedFocusPanel`, large condition blocks, clues, and counters in the middle column with:

```html
<div class="state-machine-pair">
  <div class="machine-card machine-card-primary" id="elementMachineCard">
    <div class="machine-label">元素状态机</div>
    <div class="machine-state" id="routeMainStateValue">--</div>
    <div class="machine-sub" id="routeSubStateValue">--</div>
    <div class="machine-meta">
      <span>偏向 <strong id="routePreferredSourceValue">--</strong></span>
      <span>编码器 <strong id="routeEncoderValue">--</strong></span>
    </div>
  </div>
  <div class="machine-card" id="bypassMachineCard">
    <div class="machine-label">绕行状态机</div>
    <div class="machine-state" id="targetBoardStateValue">--</div>
    <div class="machine-sub" id="targetBoardActiveValue">--</div>
    <div class="machine-meta">
      <span>确认 <strong id="targetBoardConfirmValue">--</strong></span>
      <span>偏移 <strong id="targetBoardOffsetValue">--</strong></span>
    </div>
  </div>
</div>
<details class="detail-panel state-detail-panel">
  <summary>状态机判定细节</summary>
  <div class="route-state-judgment" id="routeStateJudgment">waiting...</div>
  <div class="route-state-grid">
    <div class="route-state-block">
      <div class="route-state-block-title">下一步状态</div>
      <div class="route-state-next" id="routeNextStateValue">--</div>
      <div class="route-state-lines" id="routeNextReason"></div>
    </div>
    <div class="route-state-block">
      <div class="route-state-block-title">触发条件与当前判定</div>
      <div class="route-state-lines" id="routeConditionLines"></div>
    </div>
    <div class="route-state-block">
      <div class="route-state-block-title">原图标志层（当前值）</div>
      <div class="route-state-lines" id="routeRawFlagLines"></div>
    </div>
    <div class="route-state-block">
      <div class="route-state-block-title">状态判断（实际 / 标准）</div>
      <div class="route-state-lines" id="routeJudgeMetricLines"></div>
    </div>
  </div>
  <div class="route-state-clues" id="routeStateClues"></div>
  <div class="route-state-counters" id="routeStateCounters"></div>
</details>
```

Keep hidden placeholder elements for removed IDs that scripts still reference until render functions are simplified:

```html
<div class="compat-hidden" hidden>
  <div id="straightFocusPanel"><span id="straightFocusBadge"></span><span id="straightFocusCenterlineCount"></span><span id="straightFocusLastIndex"></span><span id="straightFocusErrorSum"></span><span id="straightFocusErrorMax"></span><span id="straightFocusDetail"></span></div>
  <div id="crossFocusPanel"><span id="crossFocusBadge"></span><span id="crossFocusLeftRows"></span><span id="crossFocusRightRows"></span><span id="crossFocusGap"></span><span id="crossFocusAux"></span><span id="crossFocusDetail"></span></div>
  <div id="speedFocusPanel"><span id="speedFocusBadge"></span><span id="speedFocusCenterlineCount"></span><span id="speedFocusDesiredSpeed"></span><span id="speedFocusMinCount"></span><span id="speedFocusErrorSum"></span><span id="speedFocusErrorThreshold"></span><span id="speedFocusDetail"></span></div>
</div>
```

- [ ] **Step 3: Render bypass state**

Add DOM refs:

```js
    const targetBoardStateValue = document.getElementById('targetBoardStateValue');
    const targetBoardActiveValue = document.getElementById('targetBoardActiveValue');
    const targetBoardConfirmValue = document.getElementById('targetBoardConfirmValue');
    const targetBoardOffsetValue = document.getElementById('targetBoardOffsetValue');
```

In `renderRouteStatePanel(status)`, after `renderRouteStatePanelWithRefs(...)`, add:

```js
      targetBoardStateValue.textContent = receiverCore.formatTargetBoardState(status && status.target_board_state);
      targetBoardActiveValue.textContent = isBool01(status && status.target_board_active) ? '激活' : '未激活';
      targetBoardConfirmValue.textContent = hasValue(status && status.target_board_confirm_count) ? String(status.target_board_confirm_count) : '--';
      targetBoardOffsetValue.textContent = hasValue(status && status.target_board_offset_px) ? String(status.target_board_offset_px) : '--';
```

- [ ] **Step 4: Rebalance right rail DOM**

Replace right rail order with:

```html
<div class="transport-panel dense-telemetry" id="transportPanel">...</div>
<div class="card speed-summary-card" id="slowdownCard">...</div>
<details class="detail-panel pid-summary-panel" open>...</details>
```

Use six compact telemetry cells in a `3 x 2` grid, speed summary with only selected branch, desired/applied speed, yaw-rate, scale, point count, and PID summary with top 10 common rows visible plus left/right columns collapsed.

- [ ] **Step 5: Apply visual design CSS**

Add CSS variables and replace the current blue-card look:

```css
:root {
  --console-bg: #080a0d;
  --console-panel: #10141a;
  --console-panel-2: #151b22;
  --console-line: #2a343f;
  --console-text: #eef3f4;
  --console-muted: #8c9aa5;
  --console-cyan: #33d6c4;
  --console-amber: #f2b84b;
  --console-red: #ef5b5b;
  --console-green: #5fc878;
}
body {
  background: var(--console-bg);
  color: var(--console-text);
  font-family: "Noto Sans Mono CJK SC", "Noto Sans SC", "Microsoft YaHei", sans-serif;
}
.card,
.transport-panel,
.detail-panel {
  background: var(--console-panel);
  border: 1px solid var(--console-line);
  border-radius: 6px;
  box-shadow: none;
}
.machine-card {
  background: linear-gradient(180deg, #161c22 0%, #0f1419 100%);
  border: 1px solid var(--console-line);
  border-left: 4px solid var(--console-cyan);
  border-radius: 6px;
  padding: 14px;
}
.machine-state {
  font-size: 30px;
  font-weight: 800;
  line-height: 1.1;
}
```

- [ ] **Step 6: Run Playwright visual check**

Run the isolated fixture replay and capture:

```bash
cd tools/pc_receiver_js
BIND_HOST=127.0.0.1 UDP_PORT=19000 TCP_PORT=19001 HTTP_PORT=19090 npm run dev
npm run mock:board -- --udp-port 19000 --tcp-port 19001 --fixture recordings/live_fixture_20260525T090208Z --duration 30
```

Then use Playwright to save:

```text
/tmp/smartcar-frontend-1920x1200-redesign.png
```

Expected: the middle column has exactly two primary state cards visible; details are folded; right rail telemetry is denser than speed/PID; no overlap or console errors.

- [ ] **Step 7: Commit**

```bash
git add tools/pc_receiver_js/public/index.html tools/pc_receiver_js/public/shared_receiver_core.js
git commit -m "feat: simplify console state overview"
```

## Task 6: Final QA and Cleanup

**Files:**
- Modify if needed: `tools/pc_receiver_js/public/index.html`
- Modify if needed: `tools/pc_receiver_js/public/playback.html`
- Modify if needed: `tools/pc_receiver_js/public/playback_app.js`

- [ ] **Step 1: Run source recording test**

```bash
cd tools/pc_receiver_js
node scripts/test_source_recording.js
```

Expected: PASS.

- [ ] **Step 2: Run mock pipeline**

```bash
cd tools/pc_receiver_js
npm run test:mock:pipeline
```

Expected: PASS with latest fixture or `recordings/live_fixture_20260525T090208Z`.

- [ ] **Step 3: Run live visual QA**

Start receiver and mock board on isolated ports, then run Playwright at `1920x1200`.

Expected screenshot path:

```text
/tmp/smartcar-frontend-1920x1200-final-live.png
```

Acceptance checks:
- Gray, binary, and IPM canvases render source data.
- Middle state area shows two primary state machine cards only.
- State details, dir charts, raw status, and PID deep details remain accessible through folded panels.
- Right rail has compact telemetry and readable speed/PID summary.
- Chinese text is readable, not boxes.
- No JavaScript console errors.

- [ ] **Step 4: Run source replay visual QA**

Record a short source-data session in the browser, save it, load `/?replay=<folder>`, then capture:

```text
/tmp/smartcar-frontend-1920x1200-final-replay.png
```

Acceptance checks:
- The same `index.html` console renders replay status and images.
- Replay controls update time and pause/play state.
- Live polling does not overwrite replay frames while replay mode is active.
- Exiting replay returns to live mode.

- [ ] **Step 5: Remove dead duplicate playback UI only after compatibility is proven**

If `playback.html?folder=<folder>` redirects to `/?replay=<folder>` and old saved recordings still list/load without server errors, remove unused large duplicated render functions from `playback_app.js`. Keep the small redirect handoff.

- [ ] **Step 6: Final commit**

```bash
git add tools/pc_receiver_js
git commit -m "chore: finalize unified receiver console QA"
```

## Assumptions and Defaults

- “元素状态机” maps to existing `route_main_state`, `route_sub_state`, `route_preferred_source`, and `route_encoder_since_enter`.
- “绕行状态机” maps to existing target-board fields: `target_board_state`, `target_board_active`, `target_board_confirm_count`, `target_board_no_red_count`, and `target_board_offset_px`.
- Existing saved video recordings remain loadable/listable, but the new default recording format is `source_frames`.
- Replay uses recorded source frames and recorded status snapshots; it does not re-feed UDP/TCP into the server.
- Mobile/tablet layout remains usable but the primary acceptance viewport is `1920x1200`.

## Self-Review

- Spec coverage: UI aesthetics, two state-machine panels, right-rail density, unified replay UI, source-data recording, compatibility, and QA are covered by Tasks 1-6.
- Placeholder scan: no TBD/TODO/later placeholders are present.
- Type consistency: `source_frames`, `status.statuses`, `target_board_*`, and `route_*` names match inspected fixture/server fields.
