(function () {
  const STORAGE_KEY = 'smartcar_run_control_record_enabled';

  function isTypingTarget(target) {
    if (!target) return false;
    const tag = String(target.tagName || '').toLowerCase();
    return tag === 'input' || tag === 'textarea' || tag === 'select' || target.isContentEditable;
  }

  function readRecordEnabled() {
    return localStorage.getItem(STORAGE_KEY) === '1';
  }

  function writeRecordEnabled(enabled) {
    localStorage.setItem(STORAGE_KEY, enabled ? '1' : '0');
  }

  async function fetchJson(url, options) {
    const response = await fetch(url, options);
    const text = await response.text();
    let parsed = {};
    try {
      parsed = text ? JSON.parse(text) : {};
    } catch (_) {
      parsed = { ok: false, message: text || `HTTP ${response.status}` };
    }
    if (!response.ok) {
      throw new Error(parsed.message || parsed.error || `HTTP ${response.status}`);
    }
    return parsed;
  }

  function injectStyle() {
    if (document.getElementById('smartcarRunControlStyle')) return;
    const style = document.createElement('style');
    style.id = 'smartcarRunControlStyle';
    style.textContent = `
      .run-control-float {
        position: fixed;
        right: 18px;
        bottom: 18px;
        z-index: 9999;
        width: min(320px, calc(100vw - 36px));
        padding: 12px;
        border: 1px solid rgba(255,255,255,0.14);
        border-radius: 16px;
        background: rgba(12, 19, 26, 0.92);
        color: #e7edf3;
        box-shadow: 0 18px 48px rgba(0,0,0,0.38);
        backdrop-filter: blur(12px);
        font-family: "Noto Sans SC", "Segoe UI", sans-serif;
      }
      .run-control-head {
        display: flex;
        justify-content: space-between;
        gap: 10px;
        align-items: center;
        margin-bottom: 10px;
      }
      .run-control-title { font-size: 14px; font-weight: 800; letter-spacing: .02em; }
      .run-control-state { font-size: 12px; color: #9fb0bf; }
      .run-control-row { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
      .run-control-btn {
        appearance: none;
        border: 1px solid rgba(255,255,255,0.12);
        border-radius: 12px;
        padding: 9px 12px;
        color: #e7edf3;
        background: #1b2836;
        cursor: pointer;
        font-size: 13px;
        font-weight: 700;
      }
      .run-control-btn.start {
        background: linear-gradient(180deg, #15956f, #0f7356);
        border-color: rgba(61,207,142,0.35);
      }
      .run-control-btn.stop {
        background: linear-gradient(180deg, #9c3f46, #7d3036);
        border-color: rgba(255,107,107,0.35);
      }
      .run-control-btn:disabled { opacity: .55; cursor: not-allowed; }
      .run-control-toggle {
        display: flex;
        gap: 7px;
        align-items: center;
        user-select: none;
        font-size: 13px;
        color: #d9e4ee;
      }
      .run-control-toggle input { accent-color: #3dcf8e; }
      .run-control-msg {
        margin-top: 9px;
        font-size: 12px;
        line-height: 1.35;
        color: #9fb0bf;
        white-space: pre-wrap;
        max-height: 72px;
        overflow: auto;
      }
      .run-control-msg.ok { color: #3dcf8e; }
      .run-control-msg.warn { color: #ffb347; }
      .run-control-msg.error { color: #ff8a8a; }
      @media (max-width: 720px) {
        .run-control-float { right: 10px; bottom: 10px; }
      }
    `;
    document.head.appendChild(style);
  }

  function fmtTarget(status) {
    const t = status && status.ssh_target;
    if (!t) return '';
    return `${t.user || 'root'}@${t.host || '--'}:${t.app_path || '/home/root/tst'}/project`;
  }

  function createWidget() {
    if (document.getElementById('smartcarRunControl')) return;
    injectStyle();

    const root = document.createElement('div');
    root.id = 'smartcarRunControl';
    root.className = 'run-control-float';
    root.innerHTML = `
      <div class="run-control-head">
        <div>
          <div class="run-control-title">车端运行控制</div>
          <div class="run-control-state" id="runControlState">状态读取中...</div>
        </div>
        <label class="run-control-toggle" title="打开后，开始/结束会联动网页录制。配置页没有视频画布，因此只保存开关状态。">
          <input id="runControlRecordToggle" type="checkbox" />
          录制
        </label>
      </div>
      <div class="run-control-row">
        <button class="run-control-btn start" id="runControlStartBtn" title="快捷键 S">开始 S</button>
        <button class="run-control-btn stop" id="runControlStopBtn" title="快捷键 P">结束 P</button>
      </div>
      <div class="run-control-msg" id="runControlMsg">S 开始，P 结束。结束会向车端程序发送 Ctrl+C。</div>
    `;
    document.body.appendChild(root);

    const stateEl = root.querySelector('#runControlState');
    const msgEl = root.querySelector('#runControlMsg');
    const startBtn = root.querySelector('#runControlStartBtn');
    const stopBtn = root.querySelector('#runControlStopBtn');
    const recordToggle = root.querySelector('#runControlRecordToggle');
    recordToggle.checked = readRecordEnabled();

    function setMsg(text, klass) {
      msgEl.className = `run-control-msg ${klass || ''}`.trim();
      msgEl.textContent = text;
    }

    function setBusy(busy) {
      startBtn.disabled = busy;
      stopBtn.disabled = busy;
    }

    function renderStatus(status) {
      const running = !!(status && status.running);
      stateEl.textContent = running ? '运行中' : '未运行';
      stateEl.style.color = running ? '#3dcf8e' : '#9fb0bf';
      const target = fmtTarget(status);
      if (target) {
        startBtn.title = `快捷键 S\n${target}`;
        stopBtn.title = `快捷键 P\n${target}`;
      }
    }

    async function refreshStatus() {
      try {
        const status = await fetchJson('/api/run_control/status', { cache: 'no-store' });
        renderStatus(status);
      } catch (err) {
        stateEl.textContent = '状态读取失败';
        stateEl.style.color = '#ffb347';
      }
    }

    async function maybeStartRecording() {
      if (!recordToggle.checked) return false;
      const api = window.smartcarRecordingApi;
      if (!api || typeof api.start !== 'function') {
        setMsg('录制开关已打开，但当前页面没有录制画布；仅启动车端程序。', 'warn');
        return false;
      }
      const wasActive = typeof api.isActive === 'function' ? !!api.isActive() : false;
      await api.start();
      return !wasActive;
    }

    async function maybeStopRecording() {
      if (!recordToggle.checked) return;
      const api = window.smartcarRecordingApi;
      if (!api || typeof api.stop !== 'function') return;
      await api.stop({ promptAfterStop: false });
    }

    async function startRun() {
      setBusy(true);
      let startedRecordingHere = false;
      try {
        startedRecordingHere = await maybeStartRecording();
        const status = await fetchJson('/api/run_control/start', { method: 'POST' });
        renderStatus(status);
        setMsg(`已发送开始命令。\n${fmtTarget(status)}`, 'ok');
      } catch (err) {
        if (startedRecordingHere && window.smartcarRecordingApi && typeof window.smartcarRecordingApi.stop === 'function') {
          await window.smartcarRecordingApi.stop({ promptAfterStop: false }).catch(() => {});
        }
        setMsg(`开始失败：${err.message}`, 'error');
      } finally {
        setBusy(false);
        refreshStatus();
      }
    }

    async function stopRun() {
      setBusy(true);
      try {
        const status = await fetchJson('/api/run_control/stop', { method: 'POST' });
        await maybeStopRecording();
        renderStatus(status);
        setMsg('已发送 Ctrl+C。', 'ok');
      } catch (err) {
        setMsg(`结束失败：${err.message}`, 'error');
      } finally {
        setBusy(false);
        setTimeout(refreshStatus, 500);
      }
    }

    recordToggle.addEventListener('change', () => {
      writeRecordEnabled(recordToggle.checked);
      setMsg(recordToggle.checked ? '录制联动已打开。' : '录制联动已关闭。', recordToggle.checked ? 'ok' : '');
    });
    startBtn.addEventListener('click', startRun);
    stopBtn.addEventListener('click', stopRun);
    window.addEventListener('keydown', (ev) => {
      if (ev.repeat || ev.ctrlKey || ev.altKey || ev.metaKey || isTypingTarget(ev.target)) return;
      const key = String(ev.key || '').toLowerCase();
      if (key === 's') {
        ev.preventDefault();
        startRun();
      } else if (key === 'p') {
        ev.preventDefault();
        stopRun();
      }
    });

    refreshStatus();
    setInterval(refreshStatus, 3000);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', createWidget);
  } else {
    createWidget();
  }
}());
