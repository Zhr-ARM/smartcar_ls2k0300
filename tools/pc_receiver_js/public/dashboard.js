// dashboard.js — 主控制逻辑

var currentPreset = 'drive';
var latestStatus = {};
var frameSeq = 0;
var fpsCounter = 0;
var fpsTimer = 0;
var currentFps = 0;

// Canvas contexts
var grayCanvas = document.getElementById('grayCanvas');
var gc = grayCanvas ? grayCanvas.getContext('2d') : null;
var ipmCanvas = document.getElementById('ipmCanvas');
var ic = ipmCanvas ? ipmCanvas.getContext('2d') : null;

if (typeof initOverlayContexts === 'function') initOverlayContexts(gc, ic);
if (typeof bindOverlayToggles === 'function') bindOverlayToggles();

// ===== Preset switching =====
var presetBtns = document.querySelectorAll('.preset-btn');
for (var i = 0; i < presetBtns.length; i++) {
  presetBtns[i].addEventListener('click', function() {
    for (var j = 0; j < presetBtns.length; j++) presetBtns[j].classList.remove('active');
    this.classList.add('active');
    currentPreset = this.getAttribute('data-preset') || 'drive';
    if (typeof renderCards === 'function') renderCards(latestStatus, currentPreset);
    if (typeof applyPresetPanels === 'function') applyPresetPanels(currentPreset);
  });
}

// ===== Panel toggle =====
var panelHeaders = document.querySelectorAll('.panel-header');
for (var i = 0; i < panelHeaders.length; i++) {
  panelHeaders[i].addEventListener('click', function() {
    this.parentElement.classList.toggle('open');
  });
}

// ===== FPS helpers =====
function updateFps(now) {
  fpsCounter++;
  if (!fpsTimer) fpsTimer = now;
  if (now - fpsTimer >= 1000) {
    currentFps = fpsCounter;
    fpsCounter = 0;
    fpsTimer = now;
  }
  latestStatus._fps = currentFps;

  var pill = document.getElementById('fpsPill');
  if (!pill) return;
  pill.textContent = currentFps + ' fps';
  pill.className = 'status-pill ' + (currentFps > 30 ? 'ok' : currentFps > 15 ? 'warn' : 'danger');
}

// ===== WebSocket =====
var ws = null;

function connectWs() {
  if (ws) { try { ws.close(); } catch(e) {} }
  var proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(proto + '://' + location.host + '/ws');

  ws.onmessage = function(ev) {
    // Server sends both JSON status and binary [status+image] frames
    if (typeof ev.data === 'string') {
      try {
        var msg = JSON.parse(ev.data);
        if (msg.type === 'status') {
          onStatus(msg.data || msg);
        }
      } catch(e) {}
    }
    // Binary messages (status+image) are ignored; we use HTTP polling for images
  };

  ws.onclose = function() {
    var pill = document.getElementById('connPill');
    if (pill) {
      pill.textContent = '断开';
      pill.className = 'status-pill warn';
    }
    setTimeout(connectWs, 2000);
  };

  ws.onopen = function() {
    var pill = document.getElementById('connPill');
    if (pill) {
      pill.textContent = '连接';
      pill.className = 'status-pill ok';
    }
  };
}

// ===== Status handler =====
function onStatus(raw) {
  latestStatus = raw || {};
  updateFps(Date.now());

  // Transport computed fields
  latestStatus._transport_mbps = raw._transport_mbps || '--';
  latestStatus._transport_kibs = raw._transport_kibs || '--';
  latestStatus._gray_fps = raw._gray_fps || '--';
  latestStatus._rgb_fps = raw._rgb_fps || '--';
  latestStatus._binary_fps = raw._binary_fps || '--';
  latestStatus._cpu_pct = raw._cpu_pct || '--';
  latestStatus._mem_pct = raw._mem_pct || '--';
  latestStatus._sync_note = raw._sync_note || '--';

  if (typeof renderCards === 'function') renderCards(latestStatus, currentPreset);
  if (typeof renderAllPanels === 'function') renderAllPanels(latestStatus);

  // Recording
  if (recordingActive) {
    recordingStatuses.push({ ts: Date.now(), status: JSON.parse(JSON.stringify(latestStatus)) });
  }
}

// ===== HTTP Image polling =====
function pullFrames() {
  frameSeq++;
  var ts = '&_' + frameSeq;

  // Gray image
  var grayImg = new Image();
  grayImg.onload = function() {
    if (!gc) return;
    gc.clearRect(0, 0, grayCanvas.width, grayCanvas.height);
    gc.drawImage(grayImg, 0, 0, grayCanvas.width, grayCanvas.height);
    if (typeof drawOverlays === 'function') drawOverlays(latestStatus);
  };
  grayImg.src = (typeof receiverCore !== 'undefined' ? receiverCore.frameUrlForMode('gray') : '/api/frame_gray.jpg') + ts;
}

// ===== Pixel probe =====
function bindPixelProbe(canvas, ctx, el) {
  if (!canvas || !el) return;
  canvas.addEventListener('mousemove', function(ev) {
    var rect = canvas.getBoundingClientRect();
    var scaleX = canvas.width / rect.width;
    var scaleY = canvas.height / rect.height;
    var x = Math.floor((ev.clientX - rect.left) * scaleX);
    var y = Math.floor((ev.clientY - rect.top) * scaleY);
    if (x >= 0 && x < canvas.width && y >= 0 && y < canvas.height) {
      el.textContent = 'x:' + x + ' y:' + y;
    }
  });
  canvas.addEventListener('mouseleave', function() { el.textContent = '-'; });
}
bindPixelProbe(grayCanvas, gc, document.getElementById('grayPixel'));
bindPixelProbe(ipmCanvas, ic, document.getElementById('ipmPixel'));

// ===== Recording =====
var recordingActive = false;
var recordingStatuses = [];

document.getElementById('recordBtn').addEventListener('click', function() {
  if (recordingActive) {
    recordingActive = false;
    this.textContent = '录制';
    this.classList.remove('recording');
    document.getElementById('recordStatus').textContent = '';
    if (recordingStatuses.length > 0) saveRecording();
    recordingStatuses = [];
  } else {
    recordingActive = true;
    this.textContent = '停止';
    this.classList.add('recording');
    recordingStatuses = [];
  }
});

function saveRecording() {
  var folder = 'rec_' + new Date().toISOString().replace(/[:.]/g, '-');
  var xhr = new XMLHttpRequest();
  xhr.open('POST', '/api/recordings/save', true);
  xhr.setRequestHeader('Content-Type', 'application/json');
  xhr.onload = function() {
    var el = document.getElementById('recordStatus');
    if (!el) return;
    try {
      var j = JSON.parse(xhr.responseText);
      el.textContent = j.ok ? '已保存: ' + folder : '保存失败';
      if (j.ok) setTimeout(function() { el.textContent = ''; }, 3000);
    } catch(e) { el.textContent = ''; }
  };
  xhr.send(JSON.stringify({ folder: folder, statuses: recordingStatuses }));
}

// ===== Start =====
if (typeof applyPresetPanels === 'function') applyPresetPanels(currentPreset);
connectWs();
setInterval(pullFrames, 60);
