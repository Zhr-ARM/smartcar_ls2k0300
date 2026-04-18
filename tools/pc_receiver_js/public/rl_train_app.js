(function () {
  const taskNameInput = document.getElementById('taskNameInput');
  const trainTaskNameInput = document.getElementById('trainTaskNameInput');
  const captureDirInput = document.getElementById('captureDirInput');
  const trainCaptureDirInput = document.getElementById('trainCaptureDirInput');
  const captureStartBtn = document.getElementById('captureStartBtn');
  const captureStopBtn = document.getElementById('captureStopBtn');
  const trainStartBtn = document.getElementById('trainStartBtn');
  const trainStopBtn = document.getElementById('trainStopBtn');
  const onlineStartBtn = document.getElementById('onlineStartBtn');
  const onlineStopBtn = document.getElementById('onlineStopBtn');
  const applyBestBtn = document.getElementById('applyBestBtn');
  const refreshBtn = document.getElementById('refreshBtn');
  const loadLatestBtn = document.getElementById('loadLatestBtn');
  const captureStatus = document.getElementById('captureStatus');
  const trainStatus = document.getElementById('trainStatus');
  const onlineStatus = document.getElementById('onlineStatus');
  const resultStatus = document.getElementById('resultStatus');
  const overviewStatus = document.getElementById('overviewStatus');
  const overviewPills = document.getElementById('overviewPills');
  const trainLogBox = document.getElementById('trainLogBox');
  const onlineLogBox = document.getElementById('onlineLogBox');
  const bestParamsBox = document.getElementById('bestParamsBox');
  const leftBestParamList = document.getElementById('leftBestParamList');
  const rightBestParamList = document.getElementById('rightBestParamList');
  const bestParamNote = document.getElementById('bestParamNote');

  let refreshTimer = 0;
  let latestPayload = null;

  function setStatus(element, text, klass) {
    element.className = `status ${klass || ''}`.trim();
    element.textContent = text;
  }

  function formatJson(data) {
    return JSON.stringify(data || {}, null, 2);
  }

  function fmtValue(value) {
    const num = Number(value);
    return Number.isFinite(num) ? num.toFixed(6) : '--';
  }

  function renderParamList(container, items) {
    container.innerHTML = '';
    items.forEach((item) => {
      const row = document.createElement('div');
      row.className = 'param-item';
      const key = document.createElement('strong');
      key.textContent = item.label;
      const value = document.createElement('span');
      value.textContent = fmtValue(item.value);
      row.appendChild(key);
      row.appendChild(value);
      container.appendChild(row);
    });
  }

  function pickBestParams(payload) {
    const online = payload && payload.online_status;
    if (online && online.best_params) {
      return {
        params: online.best_params,
        source: '在线训练中的当前最优参数',
        reward: online.best_reward
      };
    }
    const live = payload && payload.train_status && payload.train_status.live_status;
    if (live && live.best_params) {
      return {
        params: live.best_params,
        source: '训练过程中的当前最优参数',
        reward: live.best_reward
      };
    }
    const result = payload && payload.latest_result;
    if (result && result.params) {
      return {
        params: result.params,
        source: '最新落盘结果 best_params.json',
        reward: result.score
      };
    }
    return null;
  }

  function renderBestParamsPanel(payload) {
    const picked = pickBestParams(payload);
    if (!picked) {
      renderParamList(leftBestParamList, [
        { label: 'Kp', value: null },
        { label: 'Ki', value: null },
        { label: 'FF Gain', value: null }
      ]);
      renderParamList(rightBestParamList, [
        { label: 'Kp', value: null },
        { label: 'Ki', value: null },
        { label: 'FF Gain', value: null }
      ]);
      bestParamNote.textContent = '等待训练中的最优参数或最新结果...';
      return;
    }

    const p = picked.params;
    renderParamList(leftBestParamList, [
      { label: 'Kp', value: p.left_kp },
      { label: 'Ki', value: p.left_ki },
      { label: 'FF Gain', value: p.left_feedforward_gain }
    ]);
    renderParamList(rightBestParamList, [
      { label: 'Kp', value: p.right_kp },
      { label: 'Ki', value: p.right_ki },
      { label: 'FF Gain', value: p.right_feedforward_gain }
    ]);
    bestParamNote.textContent = `${picked.source}\nreward: ${picked.reward != null ? picked.reward : '--'}`;
  }

  async function fetchJson(url, options) {
    const response = await fetch(url, options);
    const text = await response.text();
    let data = {};
    try {
      data = text ? JSON.parse(text) : {};
    } catch (_) {
      data = { ok: false, message: text || `HTTP ${response.status}` };
    }
    if (!response.ok) {
      throw new Error(data && data.message ? data.message : `HTTP ${response.status}`);
    }
    return data;
  }

  function renderTaskOptions(tasks) {
    const values = Array.isArray(tasks) && tasks.length ? tasks : ['task_speed_pid_default.json'];
    [taskNameInput, trainTaskNameInput].forEach((select) => {
      const current = select.value;
      select.innerHTML = '';
      values.forEach((name) => {
        const option = document.createElement('option');
        option.value = String(name).replace(/\.json$/i, '');
        option.textContent = name;
        select.appendChild(option);
      });
      if (current && values.some((name) => String(name).replace(/\.json$/i, '') === current)) {
        select.value = current;
      }
    });
  }

  function renderOverview(payload) {
    overviewPills.innerHTML = '';
    const pills = [
      `采集: ${payload.capture_status && payload.capture_status.active ? '运行中' : '空闲'}`,
      `训练: ${payload.train_status && payload.train_status.active ? '运行中' : '空闲'}`,
      `在线训练: ${payload.online_status && payload.online_status.active ? '运行中' : '空闲'}`,
      `最新采集: ${payload.latest_capture_run || '--'}`,
      `最新结果: ${payload.latest_result && payload.latest_result.best_file ? payload.latest_result.best_file : '--'}`
    ];
    pills.forEach((text) => {
      const div = document.createElement('div');
      div.className = 'pill';
      div.textContent = text;
      overviewPills.appendChild(div);
    });
  }

  function renderCaptureStatus(status) {
    const lines = [
      `active: ${!!status.active}`,
      `task_name: ${status.task_name || '--'}`,
      `capture_id: ${status.capture_id || '--'}`,
      `frame_count: ${status.frame_count || 0}`,
      `repeat: ${status.repeat_index || 0} / ${status.repeat_total || 0}`,
      `step: ${status.step_index} ${status.step_tag || ''}`,
      `latest_run_dir: ${status.latest_run_dir || '--'}`,
      `abort_reason: ${status.abort_reason || '--'}`
    ];
    const klass = status.abort_reason ? 'warn' : (status.active ? 'ok' : '');
    setStatus(captureStatus, lines.join('\n'), klass);
  }

  function renderTrainStatus(status) {
    const live = status.live_status || {};
    const lines = [
      `active: ${!!status.active}`,
      `task_name: ${status.task_name || '--'}`,
      `pid: ${status.pid || 0}`,
      `capture_run_dir: ${status.capture_run_dir || '--'}`,
      `phase: ${live.phase || '--'}`,
      `algorithm: ${live.algorithm || '--'}`,
      `progress: ${live.progress || 0} / ${live.total || 0}`,
      `best_reward: ${live.best_reward != null ? live.best_reward : '--'}`,
      `exit_code: ${status.exit_code != null ? status.exit_code : '--'}`,
      `error: ${status.error || '--'}`
    ];
    const klass = status.error ? 'error' : (status.active ? 'ok' : '');
    setStatus(trainStatus, lines.join('\n'), klass);
    trainLogBox.value = (status.logs || []).join('\n');
  }

  function renderOnlineStatus(status) {
    const lines = [
      `active: ${!!status.active}`,
      `task_name: ${status.task_name || '--'}`,
      `phase: ${status.phase || '--'}`,
      `trial: ${status.trial_index || 0} / ${status.trial_total || 0}`,
      `step_repeat: ${status.step_repeat || 0}`,
      `step_tag: ${status.step_tag || '--'}`,
      `baseline_reward: ${status.baseline_reward != null ? status.baseline_reward : '--'}`,
      `current_reward: ${status.current_reward != null ? status.current_reward : '--'}`,
      `best_reward: ${status.best_reward != null ? status.best_reward : '--'}`,
      `error: ${status.error || '--'}`
    ];
    const klass = status.error ? 'error' : (status.active ? 'ok' : '');
    setStatus(onlineStatus, lines.join('\n'), klass);
    onlineLogBox.value = (status.logs || []).join('\n');
  }

  function renderResult(result) {
    if (!result) {
      setStatus(resultStatus, '还没有训练结果。', '');
      bestParamsBox.value = '';
      return;
    }
    const lines = [
      `task_name: ${result.task_name || '--'}`,
      `algorithm: ${result.algorithm || '--'}`,
      `score: ${result.score != null ? result.score : '--'}`,
      `best_file: ${result.best_file || '--'}`
    ];
    setStatus(resultStatus, lines.join('\n'), 'ok');
    bestParamsBox.value = formatJson({
      best: result,
      evaluation: result.evaluation || null
    });
  }

  async function refreshAll() {
    const payload = await fetchJson('/api/rl/results/latest', { cache: 'no-store' });
    latestPayload = payload;
    renderTaskOptions(payload.tasks);
    renderOverview(payload);
    renderCaptureStatus(payload.capture_status || {});
    renderTrainStatus(payload.train_status || {});
    renderOnlineStatus(payload.online_status || {});
    renderResult(payload.latest_result || null);
    renderBestParamsPanel(payload);
    trainCaptureDirInput.value = trainCaptureDirInput.value || payload.latest_capture_run || '';
    captureDirInput.value = payload.capture_status && payload.capture_status.capture_id ? payload.capture_status.capture_id : captureDirInput.value;
    setStatus(overviewStatus, 'RL 控制台状态已刷新。', 'ok');
  }

  async function startCapture() {
    const payload = await fetchJson('/api/rl/capture/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json; charset=utf-8' },
      body: JSON.stringify({
        task_name: taskNameInput.value
      })
    });
    renderCaptureStatus(payload);
    setStatus(overviewStatus, '自动阶跃采集已启动。', 'ok');
  }

  async function stopCapture() {
    const payload = await fetchJson('/api/rl/capture/stop', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json; charset=utf-8' },
      body: '{}'
    });
    renderCaptureStatus(payload);
    setStatus(overviewStatus, '已请求停止采集。', 'warn');
  }

  async function startTraining() {
    const payload = await fetchJson('/api/rl/train/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json; charset=utf-8' },
      body: JSON.stringify({
        task_name: trainTaskNameInput.value,
        capture_run_dir: trainCaptureDirInput.value.trim()
      })
    });
    renderTrainStatus(payload);
    setStatus(overviewStatus, '训练任务已启动。', 'ok');
  }

  async function stopTraining() {
    const payload = await fetchJson('/api/rl/train/stop', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json; charset=utf-8' },
      body: '{}'
    });
    renderTrainStatus(payload);
    setStatus(overviewStatus, '已请求停止训练。', 'warn');
  }

  async function applyBest() {
    const payload = await fetchJson('/api/rl/apply_best', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json; charset=utf-8' },
      body: '{}'
    });
    setStatus(overviewStatus, `最佳参数已应用。\n通道: ${payload.applied && payload.applied.mode ? payload.applied.mode : '--'}`, 'ok');
    renderResult(payload.best || null);
  }

  async function startOnlineTraining() {
    const payload = await fetchJson('/api/rl/online/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json; charset=utf-8' },
      body: JSON.stringify({
        task_name: trainTaskNameInput.value
      })
    });
    renderOnlineStatus(payload);
    setStatus(overviewStatus, '在线训练已启动。已自动限制速度阶跃频率。', 'ok');
  }

  async function stopOnlineTraining() {
    const payload = await fetchJson('/api/rl/online/stop', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json; charset=utf-8' },
      body: '{}'
    });
    renderOnlineStatus(payload);
    setStatus(overviewStatus, '已请求停止在线训练。', 'warn');
  }

  function scheduleRefresh() {
    clearTimeout(refreshTimer);
    refreshTimer = window.setTimeout(async () => {
      try {
        await refreshAll();
      } catch (err) {
        setStatus(overviewStatus, `自动刷新失败:\n${err.message}`, 'error');
      }
      scheduleRefresh();
    }, 1500);
  }

  captureStartBtn.addEventListener('click', () => startCapture().then(refreshAll).catch((err) => setStatus(overviewStatus, `启动采集失败:\n${err.message}`, 'error')));
  captureStopBtn.addEventListener('click', () => stopCapture().then(refreshAll).catch((err) => setStatus(overviewStatus, `停止采集失败:\n${err.message}`, 'error')));
  trainStartBtn.addEventListener('click', () => startTraining().then(refreshAll).catch((err) => setStatus(overviewStatus, `启动训练失败:\n${err.message}`, 'error')));
  trainStopBtn.addEventListener('click', () => stopTraining().then(refreshAll).catch((err) => setStatus(overviewStatus, `停止训练失败:\n${err.message}`, 'error')));
  onlineStartBtn.addEventListener('click', () => startOnlineTraining().then(refreshAll).catch((err) => setStatus(overviewStatus, `启动在线训练失败:\n${err.message}`, 'error')));
  onlineStopBtn.addEventListener('click', () => stopOnlineTraining().then(refreshAll).catch((err) => setStatus(overviewStatus, `停止在线训练失败:\n${err.message}`, 'error')));
  applyBestBtn.addEventListener('click', () => applyBest().then(refreshAll).catch((err) => setStatus(overviewStatus, `应用最佳参数失败:\n${err.message}`, 'error')));
  refreshBtn.addEventListener('click', () => refreshAll().catch((err) => setStatus(overviewStatus, `刷新失败:\n${err.message}`, 'error')));
  loadLatestBtn.addEventListener('click', () => refreshAll().catch((err) => setStatus(overviewStatus, `读取最新结果失败:\n${err.message}`, 'error')));

  refreshAll().catch((err) => setStatus(overviewStatus, `初始化失败:\n${err.message}`, 'error'));
  scheduleRefresh();
})();
