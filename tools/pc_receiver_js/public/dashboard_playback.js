// dashboard_playback.js — 回放控制

var playbackActive = false;
var playbackData = null;
var playbackIndex = 0;
var playbackPlaying = false;
var playbackTimer = null;

function loadPlayback(folderName) {
  var xhr = new XMLHttpRequest();
  xhr.open('GET', '/api/recordings/load?folder=' + encodeURIComponent(folderName), true);
  xhr.onload = function() {
    try {
      var data = JSON.parse(xhr.responseText);
      if (!data.ok) { alert('加载失败: ' + (data.message || data.error)); return; }
      // server returns { ok, folder, status: {statuses:[...]}, meta, videos }
      playbackData = data.status || data;
      playbackIndex = 0;
      playbackActive = true;
      playbackPlaying = false;
      document.getElementById('playbackBar').classList.add('visible');
      seekPlayback(0);
    } catch (e) { alert('解析失败: ' + e.message); }
  };
  xhr.onerror = function() { alert('网络错误'); };
  xhr.send();
}

document.getElementById('pbPlay').addEventListener('click', function() {
  if (!playbackActive) return;
  if (playbackPlaying) { pausePlayback(); }
  else { playPlayback(); }
});

document.getElementById('pbBack').addEventListener('click', function() {
  seekPlaybackBy(-1.0);
});

document.getElementById('pbFwd').addEventListener('click', function() {
  seekPlaybackBy(1.0);
});

document.getElementById('pbClose').addEventListener('click', function() {
  stopPlayback();
});

document.getElementById('pbSeek').addEventListener('input', function() {
  if (!playbackData || !playbackData.statuses) return;
  var pct = Number(this.value) / 100;
  var max = playbackData.statuses.length - 1;
  playbackIndex = Math.floor(pct * max);
  if (playbackIndex < 0) playbackIndex = 0;
  if (playbackIndex > max) playbackIndex = max;
  renderPlaybackFrame();
});

function playPlayback() {
  playbackPlaying = true;
  document.getElementById('pbPlay').textContent = '⏸';
  playbackTimer = setInterval(function() {
    if (!playbackData || !playbackData.statuses || playbackIndex >= playbackData.statuses.length - 1) {
      pausePlayback();
      return;
    }
    playbackIndex++;
    renderPlaybackFrame();
    updatePlaybackUI();
  }, 1000 / 30);
}

function pausePlayback() {
  playbackPlaying = false;
  document.getElementById('pbPlay').textContent = '▶';
  if (playbackTimer) { clearInterval(playbackTimer); playbackTimer = null; }
}

function seekPlaybackBy(deltaSec) {
  if (!playbackData || !playbackData.statuses) return;
  var delta = Math.round(deltaSec * 30);
  playbackIndex += delta;
  if (playbackIndex < 0) playbackIndex = 0;
  var max = playbackData.statuses.length - 1;
  if (playbackIndex > max) playbackIndex = max;
  renderPlaybackFrame();
  updatePlaybackUI();
}

function seekPlayback(idx) {
  if (!playbackData || !playbackData.statuses) return;
  playbackIndex = idx;
  if (playbackIndex < 0) playbackIndex = 0;
  var max = playbackData.statuses.length - 1;
  if (playbackIndex > max) playbackIndex = max;
  renderPlaybackFrame();
  updatePlaybackUI();
}

function stopPlayback() {
  pausePlayback();
  playbackActive = false;
  playbackData = null;
  document.getElementById('playbackBar').classList.remove('visible');
}

function renderPlaybackFrame() {
  if (!playbackData || !playbackData.statuses || !playbackData.statuses[playbackIndex]) return;
  var frame = playbackData.statuses[playbackIndex];
  var status = frame.status || frame;
  if (typeof renderCards === 'function') renderCards(status, currentPreset || 'drive');
  if (typeof renderAllPanels === 'function') renderAllPanels(status);
}

function updatePlaybackUI() {
  if (!playbackData || !playbackData.statuses) return;
  var total = playbackData.statuses.length;
  var cur = playbackIndex;
  var pct = total > 1 ? (cur / (total - 1)) * 100 : 0;
  var seekEl = document.getElementById('pbSeek');
  var timeEl = document.getElementById('pbTime');
  if (seekEl) seekEl.value = String(Math.round(pct));
  if (timeEl) timeEl.textContent = fmtTime(cur / 30) + ' / ' + fmtTime(total / 30);
}

function fmtTime(sec) {
  var m = Math.floor(sec / 60);
  var s = Math.floor(sec % 60);
  return m + ':' + (s < 10 ? '0' : '') + s;
}
