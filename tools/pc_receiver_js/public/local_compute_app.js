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
let workerBusy = false;

const worker = new Worker('/pipeline_worker.js');
worker.onmessage = (ev) => {
  workerBusy = false;
  const payload = ev.data || {};
  renderOverlayResult(payload);
};

function renderRawStatus(status) {
  SharedReceiverCore.renderStatusList(rawStatusList, [
    ['web_data_profile', status.web_data_profile],
    ['ts_ms', status.ts_ms],
    ['left_current_count', status.left_current_count],
    ['right_current_count', status.right_current_count],
    ['left_filtered_count', status.left_filtered_count],
    ['right_filtered_count', status.right_filtered_count],
    ['left_target_count', status.left_target_count],
    ['right_target_count', status.right_target_count],
    ['base_speed', status.base_speed],
    ['adjusted_base_speed', status.adjusted_base_speed],
    ['otsu_threshold', status.otsu_threshold]
  ]);
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

  const info = {
    summary: payload.summary || 'worker ready',
    limitations: payload.limitations || [],
    recommendation: payload.recommendation || ''
  };
  resultText.textContent = JSON.stringify(info, null, 2);
}

async function pullStatus() {
  try {
    latestStatus = await SharedReceiverCore.fetchJsonNoStore('/api/status');
    renderRawStatus(latestStatus);
  } catch (_) {
    // ignore
  }
}

async function pullFrameMeta() {
  try {
    const meta = await SharedReceiverCore.fetchJsonNoStore('/api/frame_meta');
    const slot = meta[frameModeSelect.value] || {};
    metaText.textContent =
      `mode=${frameModeSelect.value} frameId=${slot.frameId ?? -1} size=${slot.width || 0}x${slot.height || 0}`;
  } catch (_) {
    metaText.textContent = 'frame meta unavailable';
  }
}

function pullFrame() {
  frameImg.src = SharedReceiverCore.frameUrlForMode(frameModeSelect.value);
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
