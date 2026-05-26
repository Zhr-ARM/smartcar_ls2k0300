(function () {
  'use strict';

  const STORAGE_KEY = 'smartcar_project_control_record';
  const replayMode = new URLSearchParams(window.location.search).has('replay');
  const state = {
    busy: false,
    running: false,
    pid: null,
    target: null,
    lastMessage: '',
    recordingActive: false
  };

  function injectStyle() {
    if (document.getElementById('projectControlStyle')) return;
    const style = document.createElement('style');
    style.id = 'projectControlStyle';
    style.textContent = `
      body { padding-bottom: max(86px, env(safe-area-inset-bottom)); }
      .project-float-bar {
        position: fixed;
        left: 50%;
        bottom: max(16px, env(safe-area-inset-bottom));
        z-index: 1200;
        transform: translateX(-50%);
        display: flex;
        align-items: center;
        gap: 10px;
        width: min(920px, calc(100vw - 24px));
        min-height: 58px;
        padding: 10px 12px;
        border: 1px solid rgba(148, 163, 184, 0.34);
        border-radius: 14px;
        background: rgba(10, 16, 24, 0.94);
        box-shadow: 0 18px 50px rgba(0, 0, 0, 0.42), inset 0 1px 0 rgba(255, 255, 255, 0.06);
        color: #e7edf3;
        backdrop-filter: blur(16px);
        font-family: "Noto Sans SC", "Microsoft YaHei", sans-serif;
      }
      .project-float-bar * { box-sizing: border-box; }
      .project-float-title {
        display: flex;
        flex-direction: column;
        gap: 2px;
        min-width: 0;
        flex: 1 1 auto;
      }
      .project-float-primary {
        display: flex;
        align-items: center;
        gap: 8px;
        min-width: 0;
        color: #f8fafc;
        font-size: 13px;
        font-weight: 700;
        line-height: 1.3;
      }
      .project-float-dot {
        width: 9px;
        height: 9px;
        flex: 0 0 auto;
        border-radius: 999px;
        background: #64748b;
        box-shadow: 0 0 0 4px rgba(100, 116, 139, 0.16);
      }
      .project-float-dot.running {
        background: #3dcf8e;
        box-shadow: 0 0 0 4px rgba(61, 207, 142, 0.18);
      }
      .project-float-dot.error {
        background: #ff6b6b;
        box-shadow: 0 0 0 4px rgba(255, 107, 107, 0.18);
      }
      .project-float-status {
        overflow: hidden;
        color: #9fb0bf;
        font-size: 12px;
        line-height: 1.35;
        text-overflow: ellipsis;
        white-space: nowrap;
      }
      .project-float-record {
        display: inline-flex;
        align-items: center;
        gap: 7px;
        flex: 0 0 auto;
        min-height: 38px;
        padding: 0 12px;
        border: 1px solid rgba(148, 163, 184, 0.24);
        border-radius: 10px;
        background: rgba(15, 23, 42, 0.76);
        color: #dbeafe;
        font-size: 13px;
        cursor: pointer;
        user-select: none;
      }
      .project-float-record input {
        width: 15px;
        height: 15px;
        accent-color: #33d6c4;
      }
      .project-float-actions {
        display: flex;
        align-items: center;
        gap: 8px;
        flex: 0 0 auto;
      }
      .project-float-btn {
        appearance: none;
        min-width: 86px;
        min-height: 38px;
        padding: 0 14px;
        border: 1px solid rgba(148, 163, 184, 0.28);
        border-radius: 10px;
        color: #f8fafc;
        font-size: 14px;
        font-weight: 700;
        cursor: pointer;
        transition: transform 0.12s ease, filter 0.12s ease, opacity 0.12s ease;
      }
      .project-float-btn:hover:not(:disabled) { transform: translateY(-1px); filter: brightness(1.08); }
      .project-float-btn:disabled { cursor: not-allowed; opacity: 0.52; }
      .project-float-start { background: linear-gradient(180deg, #188a74 0%, #106856 100%); }
      .project-float-stop { background: linear-gradient(180deg, #a33d47 0%, #792d35 100%); }
      .project-float-key {
        color: rgba(226, 232, 240, 0.72);
        font-size: 11px;
        font-weight: 600;
      }
      @media (max-width: 720px) {
        body { padding-bottom: 144px; }
        .project-float-bar {
          align-items: stretch;
          flex-wrap: wrap;
        }
        .project-float-title {
          flex-basis: 100%;
        }
        .project-float-record {
          flex: 1 1 130px;
          justify-content: center;
        }
        .project-float-actions {
          flex: 2 1 260px;
        }
        .project-float-btn {
          flex: 1 1 0;
          min-width: 0;
        }
      }
      body.project-control-config-page .project-float-bar {
        left: auto;
        right: 24px;
        transform: none;
        width: min(560px, calc(100vw - 48px));
      }
      @media (max-width: 980px) {
        body.project-control-config-page .project-float-bar {
          left: 50%;
          right: auto;
          transform: translateX(-50%);
          width: min(920px, calc(100vw - 24px));
        }
      }
    `;
    document.head.appendChild(style);
  }

  function fetchJson(url, options) {
    return fetch(url, options).then(async (response) => {
      const text = await response.text();
      let parsed = {};
      try {
        parsed = text ? JSON.parse(text) : {};
      } catch (_) {
        parsed = { ok: false, message: text || `HTTP ${response.status}` };
      }
      if (!response.ok) {
        throw new Error(parsed.error || parsed.message || `HTTP ${response.status}`);
      }
      return parsed;
    });
  }

  function postJson(url, payload) {
    return fetchJson(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload || {})
    });
  }

  function recordingBridge() {
    return window.smartcarRecordingControls || null;
  }

  async function getRecordingActive() {
    const bridge = recordingBridge();
    if (bridge && typeof bridge.isRecording === 'function') {
      return !!bridge.isRecording();
    }
    try {
      const result = await fetchJson('/api/recordings/backend/status', { cache: 'no-store' });
      return !!(result && result.active);
    } catch (_) {
      return false;
    }
  }

  async function startRecordingIfNeeded() {
    if (await getRecordingActive()) return;
    const bridge = recordingBridge();
    if (bridge && typeof bridge.start === 'function') {
      await bridge.start();
    } else {
      await postJson('/api/recordings/backend/start');
    }
  }

  async function stopRecordingIfNeeded() {
    if (!(await getRecordingActive())) return;
    const bridge = recordingBridge();
    if (bridge && typeof bridge.stop === 'function') {
      await bridge.stop();
    } else {
      await postJson('/api/recordings/backend/stop');
    }
  }

  function messageLabel(message) {
    const labels = {
      running: '运行中',
      stopped: '未运行',
      already_running: '已经在运行',
      started: '已启动',
      start_failed: '启动失败',
      app_path_not_found: '启动目录不存在',
      project_not_executable: 'project 不存在或不可执行',
      no_pid_file: '没有网页启动的 pid 记录',
      stopped: '已停止',
      not_running: '进程已不在运行',
      sigint_sent_still_running: '已发送 SIGINT，但进程仍在运行'
    };
    return labels[message] || message || '';
  }

  function targetLabel(target) {
    if (!target) return '目标: --';
    const preset = target.preset_label || target.preset_id || '当前预设';
    return `${preset} ${target.user}@${target.host}:${target.app_path}`;
  }

  function setBusy(nextBusy) {
    state.busy = !!nextBusy;
    render();
  }

  function setStatus(result, options) {
    const opts = options || {};
    state.running = !!(result && result.running);
    state.pid = result && result.pid ? result.pid : null;
    state.target = result && result.target ? result.target : state.target;
    state.lastMessage = opts.error ? String(opts.error) : messageLabel(result && result.message);
    if (opts.note) state.lastMessage = opts.note;
    render(opts.error ? 'error' : '');
  }

  function render(errorClass) {
    const bar = document.getElementById('projectFloatBar');
    if (!bar) return;
    const dot = bar.querySelector('[data-role="dot"]');
    const status = bar.querySelector('[data-role="status"]');
    const startBtn = bar.querySelector('[data-role="start"]');
    const stopBtn = bar.querySelector('[data-role="stop"]');
    const recordInput = bar.querySelector('[data-role="record"]');
    const disabled = state.busy || replayMode;
    dot.classList.toggle('running', state.running && !errorClass);
    dot.classList.toggle('error', !!errorClass);
    startBtn.disabled = disabled || state.running;
    stopBtn.disabled = disabled;
    recordInput.disabled = disabled;
    const chunks = [
      replayMode ? '回放模式禁用启停' : (state.running ? `运行中${state.pid ? ` PID ${state.pid}` : ''}` : '未运行'),
      state.lastMessage,
      targetLabel(state.target)
    ].filter(Boolean);
    status.textContent = chunks.join(' | ');
  }

  async function refreshStatus() {
    try {
      const result = await fetchJson('/api/project/status', { cache: 'no-store' });
      state.recordingActive = await getRecordingActive();
      setStatus(result, result && result.ok === false ? { error: result.message || '状态获取失败' } : null);
    } catch (err) {
      setStatus({ running: false }, { error: err.message || String(err) });
    }
  }

  async function startProjectFlow() {
    if (state.busy || replayMode) return;
    setBusy(true);
    try {
      const recordInput = document.querySelector('#projectFloatBar [data-role="record"]');
      if (recordInput && recordInput.checked) {
        await startRecordingIfNeeded();
      }
      const result = await postJson('/api/project/start');
      setStatus(result);
    } catch (err) {
      setStatus({ running: false }, { error: err.message || String(err) });
    } finally {
      setBusy(false);
      refreshStatus();
    }
  }

  async function stopProjectFlow() {
    if (state.busy || replayMode) return;
    setBusy(true);
    let stopError = '';
    try {
      const result = await postJson('/api/project/stop');
      setStatus(result);
    } catch (err) {
      stopError = err.message || String(err);
      setStatus({ running: state.running, target: state.target }, { error: stopError });
    }
    try {
      const recordInput = document.querySelector('#projectFloatBar [data-role="record"]');
      if (recordInput && (recordInput.checked || await getRecordingActive())) {
        await stopRecordingIfNeeded();
      }
    } catch (err) {
      const msg = stopError
        ? `${stopError}；录制停止失败：${err.message || err}`
        : `录制停止失败：${err.message || err}`;
      setStatus({ running: state.running, target: state.target }, { error: msg });
    } finally {
      setBusy(false);
      refreshStatus();
    }
  }

  function isEditableTarget(target) {
    if (!target) return false;
    const tag = String(target.tagName || '').toLowerCase();
    return tag === 'input' || tag === 'textarea' || tag === 'select' || target.isContentEditable;
  }

  function bindShortcuts() {
    document.addEventListener('keydown', (ev) => {
      if (ev.defaultPrevented || ev.ctrlKey || ev.metaKey || ev.altKey || isEditableTarget(ev.target)) return;
      const key = String(ev.key || '').toLowerCase();
      if (key === 'z') {
        ev.preventDefault();
        startProjectFlow();
      } else if (key === 'q') {
        ev.preventDefault();
        stopProjectFlow();
      }
    });
  }

  function mount() {
    injectStyle();
    if (window.location.pathname.endsWith('/config.html')) {
      document.body.classList.add('project-control-config-page');
    }
    if (document.getElementById('projectFloatBar')) return;
    const checked = localStorage.getItem(STORAGE_KEY) === '1';
    const bar = document.createElement('div');
    bar.className = 'project-float-bar';
    bar.id = 'projectFloatBar';
    bar.innerHTML = `
      <div class="project-float-title">
        <div class="project-float-primary">
          <span class="project-float-dot" data-role="dot"></span>
          <span>主板程序控制</span>
        </div>
        <div class="project-float-status" data-role="status">正在检测状态...</div>
      </div>
      <label class="project-float-record">
        <input type="checkbox" data-role="record" ${checked ? 'checked' : ''} />
        <span>录制</span>
      </label>
      <div class="project-float-actions">
        <button class="project-float-btn project-float-start" type="button" data-role="start">启动 <span class="project-float-key">Z</span></button>
        <button class="project-float-btn project-float-stop" type="button" data-role="stop">终止 <span class="project-float-key">Q</span></button>
      </div>
    `;
    document.body.appendChild(bar);
    bar.querySelector('[data-role="record"]').addEventListener('change', (ev) => {
      localStorage.setItem(STORAGE_KEY, ev.target.checked ? '1' : '0');
    });
    bar.querySelector('[data-role="start"]').addEventListener('click', startProjectFlow);
    bar.querySelector('[data-role="stop"]').addEventListener('click', stopProjectFlow);
    bindShortcuts();
    render();
    refreshStatus();
    setInterval(refreshStatus, 2500);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', mount);
  } else {
    mount();
  }
}());
