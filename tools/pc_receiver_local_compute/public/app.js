const rawCanvas = document.getElementById('rawCanvas');
const rawCtx = rawCanvas.getContext('2d');
const overlayCanvas = document.getElementById('overlayCanvas');
const overlayCtx = overlayCanvas.getContext('2d');
const frameModeSelect = document.getElementById('frameMode');
const toggleOverlay = document.getElementById('toggleOverlay');
const runOnceBtn = document.getElementById('runOnceBtn');
const metaText = document.getElementById('metaText');
const rawStatusList = document.getElementById('rawStatusList');
const resultText = document.getElementById('resultText');

const frameImg = new Image();
const frameBitmapCanvas = document.createElement('canvas');
const frameBitmapCtx = frameBitmapCanvas.getContext('2d', { willReadFrequently: true });

let latestStatus = {};
let overlayEnabled = true;
let lastFrameMeta = null;
let workerBusy = false;

const worker = new Worker('/pipeline_worker.js');
worker.onmessage = (ev) => {
  workerBusy = false;
  const payload = ev.data || {};
  renderOverlayResult(payload);
};

function frameUrlForMode(mode) {
  if (mode === 'rgb') return `/api/frame_rgb.jpg?t=${Date.now()}`;
  if (mode === 'binary') return `/api/frame_binary.jpg?t=${Date.now()}`;
  return `/api/frame_gray.jpg?t=${Date.now()}`;
}

function fmt(value) {
  if (typeof value === 'number') return Number.isFinite(value) ? value.toFixed(3) : String(value);
  return String(value);
}

function renderRawStatus(status) {
  const captureThreadFps = status.capture_thread_fps ?? status.capture_fps;
  const visionProcessFps = status.vision_process_fps ?? status.process_fps;
  const udpTxFps = status.udp_tx_fps ?? status.send_fps;

  const rows = [
    ['ts_ms', status.ts_ms],
    ['capture_thread_fps', captureThreadFps],
    ['vision_process_fps', visionProcessFps],
    ['udp_tx_fps', udpTxFps],
    ['left_current_count', status.left_current_count],
    ['right_current_count', status.right_current_count],
    ['left_filtered_count', status.left_filtered_count],
    ['right_filtered_count', status.right_filtered_count],
    ['left_target_count', status.left_target_count],
    ['right_target_count', status.right_target_count],
    ['base_speed', status.base_speed],
    ['line_error', status.line_error],
    ['otsu_threshold', status.otsu_threshold]
  ];

  rawStatusList.innerHTML = '';
  for (const [name, value] of rows) {
    const item = document.createElement('div');
    item.className = 'status-item';
    item.innerHTML = `<strong>${name}</strong>: ${value === undefined ? 'N/A' : fmt(value)}`;
    rawStatusList.appendChild(item);
  }
}

function drawFrameToCanvases() {
  const w = frameImg.naturalWidth || 160;
  const h = frameImg.naturalHeight || 120;
  if (rawCanvas.width !== w) rawCanvas.width = w;
  if (rawCanvas.height !== h) rawCanvas.height = h;
  if (overlayCanvas.width !== w) overlayCanvas.width = w;
  if (overlayCanvas.height !== h) overlayCanvas.height = h;
  if (frameBitmapCanvas.width !== w) frameBitmapCanvas.width = w;
  if (frameBitmapCanvas.height !== h) frameBitmapCanvas.height = h;
  rawCtx.drawImage(frameImg, 0, 0, w, h);
  frameBitmapCtx.drawImage(frameImg, 0, 0, w, h);
  overlayCtx.drawImage(frameImg, 0, 0, w, h);
}

function requestLocalRecompute() {
  if (workerBusy) return;
  try {
    const imageData = frameBitmapCtx.getImageData(0, 0, frameBitmapCanvas.width, frameBitmapCanvas.height);
    workerBusy = true;
    worker.postMessage({
      imageData,
      status: latestStatus,
      mode: frameModeSelect.value
    });
  } catch (err) {
    resultText.textContent = `本地复算失败：${err}`;
  }
}

function renderOverlayResult(payload) {
  overlayCtx.drawImage(frameImg, 0, 0, overlayCanvas.width, overlayCanvas.height);
  if (overlayEnabled && Array.isArray(payload.overlayPoints)) {
    overlayCtx.fillStyle = '#facc15';
    for (const point of payload.overlayPoints) {
      if (!Array.isArray(point) || point.length < 2) continue;
      overlayCtx.beginPath();
      overlayCtx.arc(point[0], point[1], 1.5, 0, Math.PI * 2);
      overlayCtx.fill();
    }
  }
  if (frameModeSelect.value === 'binary') {
    drawBoundaryStraightLabel(overlayCtx, isActiveFlag(latestStatus.left_boundary_straight), 'L直边', 6, 14, 'left');
    drawBoundaryStraightLabel(overlayCtx, isActiveFlag(latestStatus.right_boundary_straight), 'R直边', overlayCanvas.width - 6, 14, 'right');
  }

  const info = {
    summary: payload.summary || 'worker ready',
    limitations: payload.limitations || [],
    recommendation: payload.recommendation || ''
  };
  resultText.textContent = JSON.stringify(info, null, 2);
}

function drawBoundaryStraightLabel(ctx, active, label, anchorX, anchorY, side) {
  if (!active) return;
  ctx.save();
  ctx.font = '11px "Noto Sans SC", "Microsoft YaHei", sans-serif';
  ctx.textBaseline = 'middle';
  const padX = 6;
  const boxH = 16;
  const textW = ctx.measureText(label).width;
  const boxW = Math.ceil(textW + padX * 2);
  const x = side === 'right' ? Math.max(0, anchorX - boxW) : Math.min(overlayCanvas.width - boxW, anchorX);
  const y = Math.max(0, Math.min(overlayCanvas.height - boxH, anchorY - boxH / 2));
  ctx.fillStyle = 'rgba(34, 197, 94, 0.88)';
  ctx.fillRect(x, y, boxW, boxH);
  ctx.strokeStyle = '#052e16';
  ctx.lineWidth = 1;
  ctx.strokeRect(x, y, boxW, boxH);
  ctx.fillStyle = '#f8fafc';
  ctx.fillText(label, x + padX, y + boxH / 2);
  ctx.restore();
}

function isActiveFlag(value) {
  if (value === true || value === 1) return true;
  if (typeof value === 'string') {
    const v = value.trim().toLowerCase();
    return v === '1' || v === 'true';
  }
  return false;
}

async function pullStatus() {
  try {
    const r = await fetch('/api/status', { cache: 'no-store' });
    latestStatus = await r.json();
    renderRawStatus(latestStatus);
  } catch (_) {
    // ignore
  }
}

async function pullFrameMeta() {
  try {
    const r = await fetch('/api/frame_meta', { cache: 'no-store' });
    lastFrameMeta = await r.json();
    const slot = lastFrameMeta[frameModeSelect.value] || {};
    metaText.textContent = `mode=${frameModeSelect.value} frameId=${slot.frameId ?? -1} size=${slot.width || 0}x${slot.height || 0}`;
  } catch (_) {
    metaText.textContent = 'frame meta unavailable';
  }
}

function pullFrame() {
  frameImg.src = frameUrlForMode(frameModeSelect.value);
}

frameImg.onload = () => {
  drawFrameToCanvases();
  requestLocalRecompute();
};

frameImg.onerror = () => {
  rawCtx.fillStyle = '#000';
  rawCtx.fillRect(0, 0, rawCanvas.width, rawCanvas.height);
  overlayCtx.fillStyle = '#000';
  overlayCtx.fillRect(0, 0, overlayCanvas.width, overlayCanvas.height);
};

frameModeSelect.addEventListener('change', () => {
  pullFrameMeta();
  pullFrame();
});

toggleOverlay.addEventListener('change', () => {
  overlayEnabled = !!toggleOverlay.checked;
  requestLocalRecompute();
});

runOnceBtn.addEventListener('click', () => {
  requestLocalRecompute();
});

setInterval(pullStatus, 120);
setInterval(pullFrameMeta, 500);
setInterval(pullFrame, 80);
pullStatus();
pullFrameMeta();
pullFrame();
