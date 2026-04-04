const grayCanvas = document.getElementById('grayCanvas');
const grayCtx = grayCanvas.getContext('2d');
const binaryCanvas = document.getElementById('binaryCanvas');
const binaryCtx = binaryCanvas.getContext('2d');
const ipmCanvas = document.getElementById('ipmCanvas');
const ipmCtx = ipmCanvas.getContext('2d');
const ipmRawCanvas = document.getElementById('ipmRawCanvas');
const ipmRawCtx = ipmRawCanvas.getContext('2d');
const curvatureCanvas = document.getElementById('curvatureCanvas');
const curvatureCtx = curvatureCanvas.getContext('2d');
const leftBoundaryCurvatureCanvas = document.getElementById('leftBoundaryCurvatureCanvas');
const leftBoundaryCurvatureCtx = leftBoundaryCurvatureCanvas.getContext('2d');
const rightBoundaryCurvatureCanvas = document.getElementById('rightBoundaryCurvatureCanvas');
const rightBoundaryCurvatureCtx = rightBoundaryCurvatureCanvas.getContext('2d');
const leftBoundaryAngleCosCanvas = document.getElementById('leftBoundaryAngleCosCanvas');
const leftBoundaryAngleCosCtx = leftBoundaryAngleCosCanvas.getContext('2d');
const rightBoundaryAngleCosCanvas = document.getElementById('rightBoundaryAngleCosCanvas');
const rightBoundaryAngleCosCtx = rightBoundaryAngleCosCanvas.getContext('2d');

const grayPixelInfo = document.getElementById('grayPixelInfo');
const binaryPixelInfo = document.getElementById('binaryPixelInfo');
const ipmPixelInfo = document.getElementById('ipmPixelInfo');
const ipmRawPixelInfo = document.getElementById('ipmRawPixelInfo');

const toggleOverlay = document.getElementById('toggleOverlay');
const toggleGrayBoundaries = document.getElementById('toggleGrayBoundaries');
const toggleMeanCenterline = document.getElementById('toggleMeanCenterline');
const toggleBoundaryPointMode = document.getElementById('toggleBoundaryPointMode');
const toggleCenterlineCurvatureChart = document.getElementById('toggleCenterlineCurvatureChart');
const toggleBoundaryCurvatureCharts = document.getElementById('toggleBoundaryCurvatureCharts');
const toggleBoundaryAngleCosCharts = document.getElementById('toggleBoundaryAngleCosCharts');
const curvatureYMinInput = document.getElementById('curvatureYMinInput');
const curvatureYMaxInput = document.getElementById('curvatureYMaxInput');
const runOnceBtn = document.getElementById('runOnceBtn');

const metaText = document.getElementById('metaText');
const statusSummaryText = document.getElementById('statusSummaryText');
const resultText = document.getElementById('resultText');
const compareText = document.getElementById('compareText');
const fullModeBanner = document.getElementById('fullModeBanner');
const ipmCenterlineSourceLabel = document.getElementById('ipmCenterlineSourceLabel');

const curvatureCard = document.getElementById('curvatureCard');
const leftBoundaryCurvatureCard = document.getElementById('leftBoundaryCurvatureCard');
const rightBoundaryCurvatureCard = document.getElementById('rightBoundaryCurvatureCard');
const leftBoundaryAngleCosCard = document.getElementById('leftBoundaryAngleCosCard');
const rightBoundaryAngleCosCard = document.getElementById('rightBoundaryAngleCosCard');

const grayImg = new Image();
const frameBitmapCanvas = document.createElement('canvas');
const frameBitmapCtx = frameBitmapCanvas.getContext('2d', { willReadFrequently: true });

let latestStatus = {};
let latestFrameMeta = {};
let latestPayload = null;
let workerBusy = false;
let overlayEnabled = true;
let boundaryPointMode = false;
let showGrayBoundaries = true;
let showMeanCenterline = true;
let showCenterlineCurvatureChart = true;
let showBoundaryCurvatureCharts = false;
let showBoundaryAngleCosCharts = true;
let curvatureYMin = -0.3;
let curvatureYMax = 0.3;

const worker = new Worker('/pipeline_worker.js');
worker.onmessage = (ev) => {
  workerBusy = false;
  latestPayload = ev.data || {};
  renderAll();
};

function hasPointArray(value) {
  return Array.isArray(value) && value.length > 0;
}

function getSelectedBoardIpmCenterline(status) {
  if (Array.isArray(status.ipm_centerline_selected_shift)) return status.ipm_centerline_selected_shift;
  return [];
}

function getSelectedBoardSrcCenterline(status) {
  if (Array.isArray(status.src_centerline_selected_shift)) return status.src_centerline_selected_shift;
  return [];
}

function buildMeanCenterline(pointsLeft, pointsRight) {
  const left = Array.isArray(pointsLeft) ? pointsLeft : [];
  const right = Array.isArray(pointsRight) ? pointsRight : [];
  const n = Math.min(left.length, right.length);
  const out = [];
  for (let i = 0; i < n; i += 1) {
    const lp = left[i];
    const rp = right[i];
    if (!Array.isArray(lp) || !Array.isArray(rp) || lp.length < 2 || rp.length < 2) continue;
    out.push([
      Math.round((Number(lp[0] || 0) + Number(rp[0] || 0)) * 0.5),
      Math.round((Number(lp[1] || 0) + Number(rp[1] || 0)) * 0.5)
    ]);
  }
  return out;
}

function applyProfileUi(status) {
  SharedReceiverCore.setElementVisible(fullModeBanner, SharedReceiverCore.isFullDebugProfile(status.web_data_profile));
}

function resizeCanvasToImage(canvas, image) {
  const w = image.naturalWidth || 160;
  const h = image.naturalHeight || 120;
  if (canvas.width !== w) canvas.width = w;
  if (canvas.height !== h) canvas.height = h;
}

function drawGrayFrame() {
  resizeCanvasToImage(grayCanvas, grayImg);
  resizeCanvasToImage(binaryCanvas, grayImg);
  resizeCanvasToImage(frameBitmapCanvas, grayImg);
  grayCtx.drawImage(grayImg, 0, 0, grayCanvas.width, grayCanvas.height);
  frameBitmapCtx.drawImage(grayImg, 0, 0, frameBitmapCanvas.width, frameBitmapCanvas.height);
}

function drawLocalGrayOverlay(payload) {
  grayCtx.drawImage(grayImg, 0, 0, grayCanvas.width, grayCanvas.height);
  if (!overlayEnabled || !payload) return;

  if (showGrayBoundaries) {
    SharedReceiverCore.drawSeries(grayCtx, payload.leftBoundary || [], '#ff4d4f', 2, 1, boundaryPointMode);
    SharedReceiverCore.drawSeries(grayCtx, payload.rightBoundary || [], '#22c55e', 2, 1, boundaryPointMode);
  }
  if (showMeanCenterline) {
    SharedReceiverCore.drawSeries(grayCtx, payload.centerline || [], '#facc15', 2, 1, boundaryPointMode);
  }
  SharedReceiverCore.drawSeries(grayCtx, payload.leftAuxiliaryLine || [], '#60a5fa', 1, 1, true);
  SharedReceiverCore.drawSeries(grayCtx, payload.rightAuxiliaryLine || [], '#a78bfa', 1, 1, true);
  SharedReceiverCore.drawPointSet(grayCtx, payload.leftBoundaryCorner || [], '#facc15', 2);
  SharedReceiverCore.drawPointSet(grayCtx, payload.rightBoundaryCorner || [], '#fb923c', 2);
}

function drawBinaryFrame(payload) {
  const threshold = Number(payload && Number.isFinite(payload.otsuThreshold) ? payload.otsuThreshold : payload?.workerThreshold);
  const img = frameBitmapCtx.getImageData(0, 0, frameBitmapCanvas.width, frameBitmapCanvas.height);
  const data = img.data;
  const th = Number.isFinite(threshold) ? Math.round(threshold) : Math.round(Number(latestStatus.otsu_threshold) || 127);
  for (let i = 0; i < data.length; i += 4) {
    const gray = data[i];
    const v = gray >= th ? 255 : 0;
    data[i] = v;
    data[i + 1] = v;
    data[i + 2] = v;
    data[i + 3] = 255;
  }
  binaryCtx.putImageData(img, 0, 0);
}

function fillBlack(canvas, ctx) {
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
}

function drawIpmScene(canvas, ctx, leftPts, rightPts, centerPts, leftCorners, rightCorners, trackPoint, trackValid) {
  fillBlack(canvas, ctx);
  SharedReceiverCore.drawSeries(ctx, leftPts || [], '#ff4d4f', 2, 1, boundaryPointMode);
  SharedReceiverCore.drawSeries(ctx, rightPts || [], '#22c55e', 2, 1, boundaryPointMode);
  SharedReceiverCore.drawSeries(ctx, centerPts || [], '#facc15', 2, 1, boundaryPointMode);
  SharedReceiverCore.drawPointSet(ctx, leftCorners || [], '#facc15', 2);
  SharedReceiverCore.drawPointSet(ctx, rightCorners || [], '#fb923c', 2);
  if (trackValid && Array.isArray(trackPoint) && trackPoint.length >= 2) {
    ctx.fillStyle = '#38bdf8';
    ctx.beginPath();
    ctx.arc(Number(trackPoint[0] || 0), Number(trackPoint[1] || 0), 3, 0, Math.PI * 2);
    ctx.fill();
  }
}

function renderIpmViews(payload) {
  fillBlack(ipmCanvas, ipmCtx);
  fillBlack(ipmRawCanvas, ipmRawCtx);

  if (!payload) return;

  drawIpmScene(
    ipmCanvas,
    ipmCtx,
    payload.ipmLeftBoundary,
    payload.ipmRightBoundary,
    payload.ipmCenterline,
    payload.ipmLeftBoundaryCorner,
    payload.ipmRightBoundaryCorner,
    payload.ipm_track_point,
    payload.ipm_track_valid
  );

  const rawLeft = hasPointArray(latestStatus.ipm_raw_left_boundary) ? latestStatus.ipm_raw_left_boundary : (payload.ipmLeftBoundary || []);
  const rawRight = hasPointArray(latestStatus.ipm_raw_right_boundary) ? latestStatus.ipm_raw_right_boundary : (payload.ipmRightBoundary || []);
  const rawCenter = hasPointArray(getSelectedBoardIpmCenterline(latestStatus)) ? getSelectedBoardIpmCenterline(latestStatus) : (payload.ipmCenterline || []);
  const rawLeftCorner = hasPointArray(latestStatus.ipm_left_boundary_corner) ? latestStatus.ipm_left_boundary_corner : (payload.ipmLeftBoundaryCorner || []);
  const rawRightCorner = hasPointArray(latestStatus.ipm_right_boundary_corner) ? latestStatus.ipm_right_boundary_corner : (payload.ipmRightBoundaryCorner || []);

  drawIpmScene(
    ipmRawCanvas,
    ipmRawCtx,
    rawLeft,
    rawRight,
    rawCenter,
    rawLeftCorner,
    rawRightCorner,
    latestStatus.ipm_track_point || payload.ipm_track_point,
    latestStatus.ipm_track_valid ?? payload.ipm_track_valid
  );
}

function renderCharts(payload) {
  SharedReceiverCore.setElementVisible(curvatureCard, showCenterlineCurvatureChart);
  SharedReceiverCore.setElementVisible(leftBoundaryCurvatureCard, showBoundaryCurvatureCharts);
  SharedReceiverCore.setElementVisible(rightBoundaryCurvatureCard, showBoundaryCurvatureCharts);
  SharedReceiverCore.setElementVisible(leftBoundaryAngleCosCard, showBoundaryAngleCosCharts);
  SharedReceiverCore.setElementVisible(rightBoundaryAngleCosCard, showBoundaryAngleCosCharts);

  if (!payload) return;
  SharedReceiverCore.drawCurveChartToCanvas(curvatureCanvas, curvatureCtx, payload.centerlineCurvature || [], {
    yMin: curvatureYMin,
    yMax: curvatureYMax
  });
  SharedReceiverCore.drawCurveChartToCanvas(leftBoundaryCurvatureCanvas, leftBoundaryCurvatureCtx, payload.leftBoundaryCurvature || [], {
    yMin: curvatureYMin,
    yMax: curvatureYMax
  });
  SharedReceiverCore.drawCurveChartToCanvas(rightBoundaryCurvatureCanvas, rightBoundaryCurvatureCtx, payload.rightBoundaryCurvature || [], {
    yMin: curvatureYMin,
    yMax: curvatureYMax
  });
  SharedReceiverCore.drawCurveChartToCanvas(leftBoundaryAngleCosCanvas, leftBoundaryAngleCosCtx, payload.leftBoundaryAngleCos || [], {
    yMin: -1.0,
    yMax: 1.0
  });
  SharedReceiverCore.drawCurveChartToCanvas(rightBoundaryAngleCosCanvas, rightBoundaryAngleCosCtx, payload.rightBoundaryAngleCos || [], {
    yMin: -1.0,
    yMax: 1.0
  });
}

function renderStatusSummary(status, payload) {
  statusSummaryText.textContent = SharedReceiverCore.buildStatusSummary(status, {
    getSelectedIpmCenterline: () => (payload && Array.isArray(payload.ipmCenterline) ? payload.ipmCenterline : getSelectedBoardIpmCenterline(status)),
    getLeftBoundaryKappa: () => (payload && Array.isArray(payload.leftBoundaryCurvature) ? payload.leftBoundaryCurvature : []),
    getRightBoundaryKappa: () => (payload && Array.isArray(payload.rightBoundaryCurvature) ? payload.rightBoundaryCurvature : []),
    getLeftBoundaryAngleCos: () => (payload && Array.isArray(payload.leftBoundaryAngleCos) ? payload.leftBoundaryAngleCos : []),
    getRightBoundaryAngleCos: () => (payload && Array.isArray(payload.rightBoundaryAngleCos) ? payload.rightBoundaryAngleCos : [])
  });
}

function renderResult(payload) {
  const info = {
    summary: payload.summary || 'worker ready',
    valid: !!payload.valid,
    wasm_state: payload.wasm_state || 'unknown',
    wasm_error: payload.wasm_error || '',
    worker_threshold: payload.workerThreshold,
    sampled_rows: payload.rowSamples,
    left_boundary_points: Array.isArray(payload.leftBoundary) ? payload.leftBoundary.length : 0,
    right_boundary_points: Array.isArray(payload.rightBoundary) ? payload.rightBoundary.length : 0,
    centerline_points: Array.isArray(payload.centerline) ? payload.centerline.length : 0,
    ipm_left_boundary_points: Array.isArray(payload.ipmLeftBoundary) ? payload.ipmLeftBoundary.length : 0,
    ipm_right_boundary_points: Array.isArray(payload.ipmRightBoundary) ? payload.ipmRightBoundary.length : 0,
    left_corner_points: Array.isArray(payload.leftBoundaryCorner) ? payload.leftBoundaryCorner.length : 0,
    right_corner_points: Array.isArray(payload.rightBoundaryCorner) ? payload.rightBoundaryCorner.length : 0,
    left_aux_points: Array.isArray(payload.leftAuxiliaryLine) ? payload.leftAuxiliaryLine.length : 0,
    right_aux_points: Array.isArray(payload.rightAuxiliaryLine) ? payload.rightAuxiliaryLine.length : 0,
    line_error: payload.line_error,
    ipm_track_valid: payload.ipm_track_valid,
    ipm_track_index: payload.ipm_track_index,
    ipm_track_point: payload.ipm_track_point,
    limitations: payload.limitations || [],
    recommendation: payload.recommendation || ''
  };
  resultText.textContent = JSON.stringify(info, null, 2);
}

function bestAlignedWindow(lengthA, lengthB) {
  const best = { startA: 0, startB: 0, overlap: Math.min(lengthA, lengthB) };
  if (lengthA <= 0 || lengthB <= 0) return best;

  let bestOverlap = -1;
  let bestTrim = Number.POSITIVE_INFINITY;
  for (let startA = 0; startA < lengthA; startA += 1) {
    for (let startB = 0; startB < lengthB; startB += 1) {
      const overlap = Math.min(lengthA - startA, lengthB - startB);
      if (overlap <= 0) continue;
      const trim = startA + startB + (lengthA - startA - overlap) + (lengthB - startB - overlap);
      if (overlap > bestOverlap || (overlap === bestOverlap && trim < bestTrim)) {
        bestOverlap = overlap;
        bestTrim = trim;
        best.startA = startA;
        best.startB = startB;
        best.overlap = overlap;
      }
    }
  }
  return best;
}

function averagePointDeltaByY(a, b) {
  const aa = Array.isArray(a) ? a : [];
  const bb = Array.isArray(b) ? b : [];
  const mapA = new Map();
  const mapB = new Map();

  for (const point of aa) {
    if (!Array.isArray(point) || point.length < 2) continue;
    mapA.set(Number(point[1] || 0), Number(point[0] || 0));
  }
  for (const point of bb) {
    if (!Array.isArray(point) || point.length < 2) continue;
    mapB.set(Number(point[1] || 0), Number(point[0] || 0));
  }

  const ys = [];
  for (const y of mapA.keys()) {
    if (mapB.has(y)) ys.push(y);
  }
  ys.sort((x, y) => x - y);

  let sum = 0;
  let max = 0;
  for (const y of ys) {
    const d = Math.abs((mapA.get(y) || 0) - (mapB.get(y) || 0));
    sum += d;
    if (d > max) max = d;
  }

  return {
    count_a: aa.length,
    count_b: bb.length,
    overlap: ys.length,
    shared_y_min: ys.length > 0 ? ys[0] : null,
    shared_y_max: ys.length > 0 ? ys[ys.length - 1] : null,
    mean_delta: ys.length > 0 ? Number((sum / ys.length).toFixed(3)) : null,
    max_delta: ys.length > 0 ? Number(max.toFixed(3)) : null
  };
}

function averagePointDelta(a, b) {
  const aa = Array.isArray(a) ? a : [];
  const bb = Array.isArray(b) ? b : [];
  const alignment = bestAlignedWindow(aa.length, bb.length);
  const n = alignment.overlap;
  let sum = 0;
  let max = 0;
  for (let i = 0; i < n; i += 1) {
    const ap = aa[alignment.startA + i] || [0, 0];
    const bp = bb[alignment.startB + i] || [0, 0];
    const dx = Number(ap[0] || 0) - Number(bp[0] || 0);
    const dy = Number(ap[1] || 0) - Number(bp[1] || 0);
    const d = Math.hypot(dx, dy);
    sum += d;
    if (d > max) max = d;
  }
  return {
    count_a: aa.length,
    count_b: bb.length,
    overlap: n,
    start_a: alignment.startA,
    start_b: alignment.startB,
    mean_delta: n > 0 ? Number((sum / n).toFixed(3)) : null,
    max_delta: n > 0 ? Number(max.toFixed(3)) : null
  };
}

function averageScalarDelta(a, b) {
  const aa = Array.isArray(a) ? a : [];
  const bb = Array.isArray(b) ? b : [];
  const alignment = bestAlignedWindow(aa.length, bb.length);
  const n = alignment.overlap;
  let sum = 0;
  let max = 0;
  for (let i = 0; i < n; i += 1) {
    const d = Math.abs(Number(aa[alignment.startA + i] || 0) - Number(bb[alignment.startB + i] || 0));
    sum += d;
    if (d > max) max = d;
  }
  return {
    count_a: aa.length,
    count_b: bb.length,
    overlap: n,
    start_a: alignment.startA,
    start_b: alignment.startB,
    mean_abs_delta: n > 0 ? Number((sum / n).toFixed(6)) : null,
    max_abs_delta: n > 0 ? Number(max.toFixed(6)) : null
  };
}

function buildFullComparison(status, payload) {
  if (!SharedReceiverCore.isFullDebugProfile(status.web_data_profile)) {
    return {
      mode: 'RAW_MINIMAL',
      summary: '当前不是 FULL 模式，无法和主板全量结果逐项对比。'
    };
  }

  return {
    mode: 'FULL',
    valid: !!payload.valid,
    line_error: {
      board: Number(status.line_error) || 0,
      local: Number(payload.line_error) || 0,
      delta: (Number(payload.line_error) || 0) - (Number(status.line_error) || 0)
    },
    boundaries_src: {
      left: averagePointDeltaByY(status.left_boundary, payload.leftBoundary),
      right: averagePointDeltaByY(status.right_boundary, payload.rightBoundary)
    },
    boundaries_ipm: {
      left: averagePointDeltaByY(status.ipm_left_boundary, payload.ipmLeftBoundary),
      right: averagePointDeltaByY(status.ipm_right_boundary, payload.ipmRightBoundary)
    },
    centerline: {
      src: averagePointDeltaByY(status.src_centerline_selected_shift, payload.centerline),
      ipm: averagePointDeltaByY(status.ipm_centerline_selected_shift, payload.ipmCenterline)
    },
    corners: {
      src_left: averagePointDelta(status.left_boundary_corner, payload.leftBoundaryCorner),
      src_right: averagePointDelta(status.right_boundary_corner, payload.rightBoundaryCorner),
      ipm_left: averagePointDelta(status.ipm_left_boundary_corner, payload.ipmLeftBoundaryCorner),
      ipm_right: averagePointDelta(status.ipm_right_boundary_corner, payload.ipmRightBoundaryCorner)
    },
    auxiliary: {
      left: averagePointDeltaByY(status.left_auxiliary_line, payload.leftAuxiliaryLine),
      right: averagePointDeltaByY(status.right_auxiliary_line, payload.rightAuxiliaryLine)
    },
    curvature: {
      centerline: averageScalarDelta(status.ipm_centerline_selected_curvature, payload.centerlineCurvature),
      left_boundary: averageScalarDelta(status.ipm_left_boundary_curvature, payload.leftBoundaryCurvature),
      right_boundary: averageScalarDelta(status.ipm_right_boundary_curvature, payload.rightBoundaryCurvature)
    },
    angle_cos: {
      left_boundary: averageScalarDelta(status.ipm_left_boundary_angle_cos, payload.leftBoundaryAngleCos),
      right_boundary: averageScalarDelta(status.ipm_right_boundary_angle_cos, payload.rightBoundaryAngleCos)
    }
  };
}

function renderComparison(status, payload) {
  compareText.textContent = JSON.stringify(buildFullComparison(status, payload), null, 2);
}

function pixelTextForCanvas(activeCanvas, activeCtx, x, y) {
  const [ar, ag, ab] = SharedReceiverCore.sampleRGB(activeCtx, x, y);
  const [gx, gy] = SharedReceiverCore.mapCoord(x, y, activeCanvas.width, activeCanvas.height, grayCanvas.width, grayCanvas.height);
  const [gr, gg, gb] = SharedReceiverCore.sampleRGB(grayCtx, gx, gy);
  const gray = Math.round((gr + gg + gb) / 3);
  const threshold = Math.round(Number(latestPayload?.otsuThreshold ?? latestPayload?.workerThreshold ?? latestStatus.otsu_threshold) || 127);
  const bin = gray >= threshold ? 1 : 0;
  return `x:${x} y:${y} | Gray:${gray} Bin:${bin} RGB:(${ar},${ag},${ab})`;
}

function renderAll() {
  if (grayImg.naturalWidth > 0 && grayImg.naturalHeight > 0) {
    drawGrayFrame();
    drawLocalGrayOverlay(latestPayload);
    drawBinaryFrame(latestPayload);
  }
  renderIpmViews(latestPayload);
  renderCharts(latestPayload);
  renderStatusSummary(latestStatus, latestPayload);
  renderResult(latestPayload || {});
  renderComparison(latestStatus, latestPayload || {});
  ipmCenterlineSourceLabel.textContent = '透视中线来源：本地复算 / 同源 WASM';
}

function requestLocalRecompute() {
  if (workerBusy || frameBitmapCanvas.width <= 0 || frameBitmapCanvas.height <= 0) return;
  try {
    const imageData = frameBitmapCtx.getImageData(0, 0, frameBitmapCanvas.width, frameBitmapCanvas.height);
    workerBusy = true;
    worker.postMessage({
      imageData,
      status: latestStatus,
      mode: 'gray'
    });
  } catch (err) {
    resultText.textContent = `本地复算失败：${err}`;
  }
}

async function pullStatus() {
  try {
    latestStatus = await SharedReceiverCore.fetchJsonNoStore('/api/status');
    applyProfileUi(latestStatus);
    renderStatusSummary(latestStatus, latestPayload);
  } catch (err) {
    metaText.textContent = `status error: ${err}`;
  }
}

async function pullFrameMeta() {
  try {
    latestFrameMeta = await SharedReceiverCore.fetchJsonNoStore('/api/frame_meta');
    const graySlot = latestFrameMeta.gray || {};
    metaText.textContent = `gray frameId=${graySlot.frameId ?? -1} size=${graySlot.width || 0}x${graySlot.height || 0}`;
  } catch (err) {
    metaText.textContent = `frame meta unavailable: ${err}`;
  }
}

function pullGrayFrame() {
  grayImg.src = SharedReceiverCore.frameUrlForMode('gray');
}

grayImg.onload = () => {
  drawGrayFrame();
  renderAll();
  requestLocalRecompute();
};

grayImg.onerror = () => {
  fillBlack(grayCanvas, grayCtx);
  fillBlack(binaryCanvas, binaryCtx);
};

toggleOverlay.addEventListener('change', () => {
  overlayEnabled = !!toggleOverlay.checked;
  renderAll();
});
toggleGrayBoundaries.addEventListener('change', () => {
  showGrayBoundaries = !!toggleGrayBoundaries.checked;
  renderAll();
});
toggleMeanCenterline.addEventListener('change', () => {
  showMeanCenterline = !!toggleMeanCenterline.checked;
  renderAll();
});
toggleBoundaryPointMode.addEventListener('change', () => {
  boundaryPointMode = !!toggleBoundaryPointMode.checked;
  renderAll();
});
toggleCenterlineCurvatureChart.addEventListener('change', () => {
  showCenterlineCurvatureChart = !!toggleCenterlineCurvatureChart.checked;
  renderAll();
});
toggleBoundaryCurvatureCharts.addEventListener('change', () => {
  showBoundaryCurvatureCharts = !!toggleBoundaryCurvatureCharts.checked;
  renderAll();
});
toggleBoundaryAngleCosCharts.addEventListener('change', () => {
  showBoundaryAngleCosCharts = !!toggleBoundaryAngleCosCharts.checked;
  renderAll();
});
curvatureYMinInput.addEventListener('change', () => {
  curvatureYMin = Number(curvatureYMinInput.value);
  renderAll();
});
curvatureYMaxInput.addEventListener('change', () => {
  curvatureYMax = Number(curvatureYMaxInput.value);
  renderAll();
});
runOnceBtn.addEventListener('click', () => {
  requestLocalRecompute();
});

SharedReceiverCore.bindPixelProbe(grayCanvas, grayCtx, grayPixelInfo, {
  buildText: (x, y) => pixelTextForCanvas(grayCanvas, grayCtx, x, y)
});
SharedReceiverCore.bindPixelProbe(binaryCanvas, binaryCtx, binaryPixelInfo, {
  buildText: (x, y) => pixelTextForCanvas(binaryCanvas, binaryCtx, x, y)
});
SharedReceiverCore.bindPixelProbe(ipmCanvas, ipmCtx, ipmPixelInfo, {
  buildText: (x, y) => `x:${x} y:${y}`
});
SharedReceiverCore.bindPixelProbe(ipmRawCanvas, ipmRawCtx, ipmRawPixelInfo, {
  buildText: (x, y) => `x:${x} y:${y}`
});

setInterval(pullStatus, 120);
setInterval(pullFrameMeta, 500);
setInterval(pullGrayFrame, 80);

pullStatus();
pullFrameMeta();
pullGrayFrame();
