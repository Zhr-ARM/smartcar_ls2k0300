(function () {
  const canvas = document.getElementById('speedChartCanvas');
  const ctx = canvas.getContext('2d');
  const statusBox = document.getElementById('statusBox');
  const paramStatusBox = document.getElementById('paramStatusBox');
  const paramCompareBox = document.getElementById('paramCompareBox');
  const connectionText = document.getElementById('connectionText');
  const sampleCountText = document.getElementById('sampleCountText');
  const windowText = document.getElementById('windowText');
  const yRangeText = document.getElementById('yRangeText');
  const cursorText = document.getElementById('cursorText');
  const latestText = document.getElementById('latestText');
  const followBadge = document.getElementById('followBadge');
  const pauseBadge = document.getElementById('pauseBadge');
  const leftActualToggle = document.getElementById('leftActualToggle');
  const rightActualToggle = document.getElementById('rightActualToggle');
  const leftTargetToggle = document.getElementById('leftTargetToggle');
  const rightTargetToggle = document.getElementById('rightTargetToggle');
  const followLatestToggle = document.getElementById('followLatestToggle');
  const pauseToggle = document.getElementById('pauseToggle');
  const resetViewBtn = document.getElementById('resetViewBtn');
  const clearDataBtn = document.getElementById('clearDataBtn');
  const applyParamsBtn = document.getElementById('applyParamsBtn');
  const cancelParamsBtn = document.getElementById('cancelParamsBtn');
  const leftFields = document.getElementById('leftFields');
  const commonFields = document.getElementById('commonFields');
  const rightFields = document.getElementById('rightFields');
  const fixedTargetFields = document.getElementById('fixedTargetFields');

  const MAX_POINTS = 24000;
  const DEFAULT_X_SPAN_MS = 12000;
  const DEFAULT_Y_MIN = -100;
  const DEFAULT_Y_MAX = 800;
  const YOOM_MIN_SPAN = 20;
  const XOOM_MIN_SPAN_MS = 1500;
  const COLORS = {
    leftActual: '#39d98a',
    rightActual: '#4cc2ff',
    leftTarget: '#ffd166',
    rightTarget: '#ff7aa2',
    grid: 'rgba(255,255,255,0.08)',
    axis: 'rgba(148,163,184,0.75)',
    text: '#d9e2ec',
    subText: '#8ea2b4'
  };

  const PARAM_GROUPS = [
    {
      container: leftFields,
      items: [
        { id: 'left_kp', section: 'pid.motor_speed', key: 'left_kp', type: 'number', label: '左轮 Kp', description: '作用：直接决定左轮速度误差的修正力度。', suggestion: '建议：想让左轮跟目标更快时优先加它；如果开始抖或冲过头，就往回收。' },
        { id: 'left_ki', section: 'pid.motor_speed', key: 'left_ki', type: 'number', label: '左轮 Ki', description: '作用：消除左轮稳态误差，让最终速度更贴目标。', suggestion: '建议：只在 Kp 基本稳定后再加；太大会拖尾、慢振。' },
        { id: 'left_kd', section: 'pid.motor_speed', key: 'left_kd', type: 'number', label: '左轮 Kd', description: '作用：抑制左轮误差变化过快带来的过冲。', suggestion: '建议：通常小量微调；如果当前 0 也能稳，先不要急着加。' },
        { id: 'left_feedforward_gain', section: 'pid.motor_speed', key: 'left_feedforward_gain', type: 'number', label: '左轮前馈增益', description: '作用：按目标速度直接给左轮基础输出，是“两步到位”的主力之一。', suggestion: '建议：想让第一拍就更接近目标，先调它；过大时会天然偏快。' },
        { id: 'left_feedforward_bias', section: 'pid.motor_speed', key: 'left_feedforward_bias', type: 'number', label: '左轮静摩擦补偿', description: '作用：补左轮低速起转死区和静摩擦。', suggestion: '建议：小目标速度起不来、低速发肉时增大；太大时低速会突然窜。' }
      ]
    },
    {
      container: commonFields,
      items: [
        { id: 'integral_limit', section: 'pid.motor_speed', key: 'integral_limit', type: 'number', label: '积分限幅', description: '作用：限制积分项累计大小，避免积分饱和。', suggestion: '建议：过冲后拉不回来时减小；长期差一点时可适当放大。' },
        { id: 'max_output_step', section: 'pid.motor_speed', key: 'max_output_step', type: 'number', label: '单周期最大输出步长', description: '作用：限制 PID 修正项每一拍能变化多快。', suggestion: '建议：你要追求两步到位，这个不能太小；但太大容易跳变和抖。' },
        { id: 'correction_limit', section: 'pid.motor_speed', key: 'correction_limit', type: 'number', label: 'PID 修正项总限幅', description: '作用：限制速度环修正量绝对上限。', suggestion: '建议：如果 Kp 已经不小但就是补不上去，检查这里是不是卡住了。' },
        { id: 'feedforward_bias_threshold', section: 'pid.motor_speed', key: 'feedforward_bias_threshold', type: 'number', label: '静摩擦补偿阈值', description: '作用：只有目标速度超过这个阈值时，才加静摩擦补偿。', suggestion: '建议：低速段过冲明显时适当调大；起转太晚时调小。' },
        { id: 'decel_error_threshold', section: 'pid.motor_speed', key: 'decel_error_threshold', type: 'number', label: '减速辅助触发阈值', description: '作用：目标明显低于当前速度时，何时开始额外帮你“刹车”。', suggestion: '建议：想让降速更快进入辅助制动，就调小。' },
        { id: 'decel_duty_gain', section: 'pid.motor_speed', key: 'decel_duty_gain', type: 'number', label: '减速辅助增益', description: '作用：减速误差越大时，额外减速输出增长得有多快。', suggestion: '建议：降速收不住就加；太大时会刹得太猛。' },
        { id: 'decel_duty_limit', section: 'pid.motor_speed', key: 'decel_duty_limit', type: 'number', label: '减速辅助限幅', description: '作用：限制减速辅助的最大输出。', suggestion: '建议：只有确认减速辅助有效但强度不够时再加。' },
        { id: 'feedback_average_window', section: 'pid.motor_speed', key: 'feedback_average_window', type: 'number', label: '反馈平均窗口', description: '作用：对编码器反馈做滑动平均，平滑噪声。', suggestion: '建议：窗口大更稳但更慢；你追求两步到位时不要设太大。' },
        { id: 'feedback_low_pass_alpha', section: 'pid.motor_speed', key: 'feedback_low_pass_alpha', type: 'number', label: '反馈低通 alpha', description: '作用：控制反馈滤波的跟随速度。', suggestion: '建议：越接近 1 越灵敏，越小越平滑；过重滤波会直接拖慢两步响应。' }
      ]
    },
    {
      container: rightFields,
      items: [
        { id: 'right_kp', section: 'pid.motor_speed', key: 'right_kp', type: 'number', label: '右轮 Kp', description: '作用：直接决定右轮速度误差的修正力度。', suggestion: '建议：想让右轮跟目标更快时优先加它；如果开始抖或冲过头，就往回收。' },
        { id: 'right_ki', section: 'pid.motor_speed', key: 'right_ki', type: 'number', label: '右轮 Ki', description: '作用：消除右轮稳态误差，让最终速度更贴目标。', suggestion: '建议：只在 Kp 基本稳定后再加；太大会拖尾、慢振。' },
        { id: 'right_kd', section: 'pid.motor_speed', key: 'right_kd', type: 'number', label: '右轮 Kd', description: '作用：抑制右轮误差变化过快带来的过冲。', suggestion: '建议：通常小量微调；如果当前 0 也能稳，先不要急着加。' },
        { id: 'right_feedforward_gain', section: 'pid.motor_speed', key: 'right_feedforward_gain', type: 'number', label: '右轮前馈增益', description: '作用：按目标速度直接给右轮基础输出，是“两步到位”的主力之一。', suggestion: '建议：想让第一拍就更接近目标，先调它；过大时会天然偏快。' },
        { id: 'right_feedforward_bias', section: 'pid.motor_speed', key: 'right_feedforward_bias', type: 'number', label: '右轮静摩擦补偿', description: '作用：补右轮低速起转死区和静摩擦。', suggestion: '建议：小目标速度起不来、低速发肉时增大；太大时低速会突然窜。' }
      ]
    }
  ];

  const FIXED_TARGET_ITEMS = [
    { id: 'fixed_target_count_override_enabled', section: 'pid.line_follow', key: 'fixed_target_count_override_enabled', type: 'checkbox', label: '启用固定时速覆盖', description: '作用：打开后，左右轮目标速度不再由视觉误差合成，而是直接使用下面两个固定目标。', suggestion: '建议：做速度环训练时打开；正常巡线时关闭。' },
    { id: 'fixed_left_target_count', section: 'pid.line_follow', key: 'fixed_left_target_count', type: 'number', label: '固定左轮目标', description: '作用：训练模式下左轮使用的固定目标速度。', suggestion: '建议：先让左右目标相等，再逐步拉高测试响应。' },
    { id: 'fixed_right_target_count', section: 'pid.line_follow', key: 'fixed_right_target_count', type: 'number', label: '固定右轮目标', description: '作用：训练模式下右轮使用的固定目标速度。', suggestion: '建议：若左右轮机械差异明显，可单独做微调。' }
  ];

  const FIELD_DEFS = [...PARAM_GROUPS.flatMap((group) => group.items), ...FIXED_TARGET_ITEMS];
  const FIELD_MAP = new Map();
  const points = [];
  let ws = null;
  let reconnectTimer = 0;
  let lastReceivedAt = 0;
  let latestTimestamp = 0;
  let firstArrivalAt = 0;
  let boardTimeOffsetMs = null;
  let boardTimeLastMs = null;
  let xCenterMs = 0;
  let xSpanMs = DEFAULT_X_SPAN_MS;
  let yMin = DEFAULT_Y_MIN;
  let yMax = DEFAULT_Y_MAX;
  let dragging = false;
  let lastDragX = 0;
  let lastDragY = 0;
  let hoveredData = null;
  let lastLoadedTomlText = '';
  let lastConnectionStatus = null;

  function setStatus(text, klass) {
    statusBox.className = `status-line ${klass || ''}`.trim();
    statusBox.textContent = text;
  }

  function setParamStatus(text, klass) {
    paramStatusBox.className = `status-line ${klass || ''}`.trim();
    paramStatusBox.textContent = text;
  }

  function setParamCompare(text, klass) {
    paramCompareBox.className = `status-line ${klass || ''}`.trim();
    paramCompareBox.textContent = text;
  }

  function formatValue(value) {
    const num = Number(value);
    return Number.isFinite(num) ? num.toFixed(1) : '--';
  }

  function formatClock(tsMs) {
    if (!Number.isFinite(tsMs)) return '--';
    const d = new Date(tsMs);
    const pad = (n, w = 2) => String(n).padStart(w, '0');
    return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`;
  }

  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  function normalizeTomlForComparison(text) {
    return String(text || '')
      .replace(/\r\n/g, '\n')
      .replace(/\r/g, '\n')
      .replace(/[ \t]+$/gm, '')
      .replace(/\n+$/g, '');
  }

  function summarizeDiff(boardText, editorText) {
    if (boardText === editorText) {
      return { same: true, text: '比较结果一致。' };
    }
    const normalizedBoardText = normalizeTomlForComparison(boardText);
    const normalizedEditorText = normalizeTomlForComparison(editorText);
    if (normalizedBoardText === normalizedEditorText) {
      return { same: true, text: '比较结果一致，仅存在换行或末尾空白差异。' };
    }
    const boardLines = normalizedBoardText.split('\n');
    const editorLines = normalizedEditorText.split('\n');
    const maxLines = Math.max(boardLines.length, editorLines.length);
    let firstDiffLine = -1;
    const samples = [];
    let diffCount = 0;
    for (let i = 0; i < maxLines; i += 1) {
      const boardLine = boardLines[i] ?? '';
      const editorLine = editorLines[i] ?? '';
      if (boardLine !== editorLine) {
        diffCount += 1;
        if (firstDiffLine < 0) firstDiffLine = i + 1;
        if (samples.length < 4) {
          samples.push(`L${i + 1}\n板端: ${boardLine || '(空)'}\n提交: ${editorLine || '(空)'}`);
        }
      }
    }
    return {
      same: false,
      text: ['比较结果不一致。', `首个差异行: L${firstDiffLine}`, `差异总行数: ${diffCount}`, '', ...samples].join('\n')
    };
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
        throw new Error(parsed && parsed.message ? parsed.message : `HTTP ${response.status}`);
      }
      return parsed;
    });
  }

  function createField(container, field) {
    const card = document.createElement('div');
    card.className = 'field-card';

    if (field.type === 'checkbox') {
      const label = document.createElement('label');
      label.className = 'field-inline';
      const input = document.createElement('input');
      input.type = 'checkbox';
      input.id = field.id;
      label.appendChild(input);
      const text = document.createElement('span');
      text.textContent = field.label;
      label.appendChild(text);
      card.appendChild(label);
      FIELD_MAP.set(field.id, { field, input });
    } else {
      const label = document.createElement('label');
      label.htmlFor = field.id;
      label.textContent = field.label;
      const input = document.createElement('input');
      input.id = field.id;
      input.type = field.type;
      input.step = field.step || 'any';
      card.appendChild(label);
      card.appendChild(input);
      FIELD_MAP.set(field.id, { field, input });
    }

    const note = document.createElement('div');
    note.className = 'field-note';
    note.textContent = `${field.description}\n${field.suggestion}`;
    card.appendChild(note);
    container.appendChild(card);
  }

  function renderParamFields() {
    PARAM_GROUPS.forEach((group) => group.items.forEach((field) => createField(group.container, field)));
    FIXED_TARGET_ITEMS.forEach((field) => createField(fixedTargetFields, field));
  }

  function stripTomlComment(raw) {
    let inQuotes = false;
    let escaped = false;
    let out = '';
    for (let i = 0; i < raw.length; i += 1) {
      const ch = raw[i];
      if (escaped) {
        out += ch;
        escaped = false;
        continue;
      }
      if (ch === '\\') {
        out += ch;
        escaped = true;
        continue;
      }
      if (ch === '"') {
        inQuotes = !inQuotes;
        out += ch;
        continue;
      }
      if (!inQuotes && ch === '#') {
        break;
      }
      out += ch;
    }
    return out.trim();
  }

  function getTomlValue(text, sectionName, key) {
    const lines = String(text || '').split('\n');
    let currentSection = '';
    for (const line of lines) {
      const trimmed = line.trim();
      if (trimmed.startsWith('[') && trimmed.endsWith(']')) {
        currentSection = trimmed.slice(1, -1).trim();
        continue;
      }
      if (currentSection !== sectionName) {
        continue;
      }
      const eqIndex = line.indexOf('=');
      if (eqIndex < 0) {
        continue;
      }
      const rawKey = line.slice(0, eqIndex).trim();
      if (rawKey !== key) {
        continue;
      }
      return stripTomlComment(line.slice(eqIndex + 1));
    }
    return null;
  }

  function setTomlValue(text, sectionName, key, rawValue) {
    const lines = String(text || '').split('\n');
    let currentSection = '';
    let replaced = false;
    for (let i = 0; i < lines.length; i += 1) {
      const trimmed = lines[i].trim();
      if (trimmed.startsWith('[') && trimmed.endsWith(']')) {
        currentSection = trimmed.slice(1, -1).trim();
        continue;
      }
      if (currentSection !== sectionName) {
        continue;
      }
      const eqIndex = lines[i].indexOf('=');
      if (eqIndex < 0) {
        continue;
      }
      const rawKey = lines[i].slice(0, eqIndex).trim();
      if (rawKey !== key) {
        continue;
      }
      lines[i] = `${key} = ${rawValue}`;
      replaced = true;
      break;
    }
    if (!replaced) {
      throw new Error(`TOML 中未找到 ${sectionName}.${key}`);
    }
    return lines.join('\n');
  }

  function parseFieldValue(field, rawValue) {
    if (field.type === 'checkbox') {
      return String(rawValue).trim() === 'true';
    }
    return rawValue;
  }

  function serializeFieldValue(field, inputValue) {
    if (field.type === 'checkbox') {
      return inputValue ? 'true' : 'false';
    }
    const text = String(inputValue).trim();
    if (!text) {
      throw new Error(`${field.label} 不能为空`);
    }
    const numberValue = Number(text);
    if (!Number.isFinite(numberValue)) {
      throw new Error(`${field.label} 不是有效数字`);
    }
    if (field.key === 'feedback_average_window') {
      return String(Math.round(numberValue));
    }
    return String(numberValue);
  }

  function loadFieldsFromToml(tomlText) {
    FIELD_DEFS.forEach((field) => {
      const mapping = FIELD_MAP.get(field.id);
      if (!mapping) return;
      const rawValue = getTomlValue(tomlText, field.section, field.key);
      if (rawValue === null) {
        return;
      }
      if (field.type === 'checkbox') {
        mapping.input.checked = parseFieldValue(field, rawValue);
      } else {
        mapping.input.value = parseFieldValue(field, rawValue);
      }
    });
  }

  function buildTomlWithCurrentInputs(baseTomlText) {
    let nextToml = String(baseTomlText || '');
    FIELD_DEFS.forEach((field) => {
      const mapping = FIELD_MAP.get(field.id);
      const inputValue = field.type === 'checkbox' ? mapping.input.checked : mapping.input.value;
      nextToml = setTomlValue(nextToml, field.section, field.key, serializeFieldValue(field, inputValue));
    });
    return nextToml;
  }

  function fetchRuntimeConfigText() {
    return fetchJson('/api/config/current', { cache: 'no-store' });
  }

  function fetchBoardConfigTextViaSsh() {
    return fetchJson('/api/config/ssh_pull', { cache: 'no-store' });
  }

  function refreshConnectionStatus() {
    return fetchJson('/api/config/status', { cache: 'no-store' }).then((result) => {
      lastConnectionStatus = result;
      return result;
    });
  }

  function chooseConfigReadMode(status) {
    if (status && status.runtime_http && status.runtime_http.online) {
      return 'runtime';
    }
    if (status && status.ssh_transport && status.ssh_transport.online) {
      return 'ssh';
    }
    throw new Error('当前在线 HTTP 和离线 SSH 都不可用，无法读取板端参数');
  }

  function chooseConfigWriteMode(status) {
    if (status && status.runtime_http && status.runtime_http.online) {
      return 'runtime';
    }
    if (status && status.ssh_transport && status.ssh_transport.online) {
      return 'ssh';
    }
    throw new Error('当前在线 HTTP 和离线 SSH 都不可用，无法下发参数');
  }

  function describeConnectionMode(mode) {
    return mode === 'runtime' ? '在线运行态应用' : '离线 SSH 写入';
  }

  function fillParamsFromBoard() {
    return refreshConnectionStatus().then((status) => {
      const mode = chooseConfigReadMode(status);
      setParamStatus(`正在通过${describeConnectionMode(mode)}读取板端参数...`, 'warn');
      return (mode === 'runtime' ? fetchRuntimeConfigText() : fetchBoardConfigTextViaSsh()).then((result) => {
        const tomlText = result.toml_text || '';
        if (!tomlText.trim()) {
          throw new Error('读取到的 TOML 为空');
        }
        lastLoadedTomlText = tomlText;
        loadFieldsFromToml(tomlText);
        setParamStatus(`已从板端读取参数并回填输入框。\n通道: ${describeConnectionMode(mode)}`, 'ok');
        setParamCompare('取消更改后，这里会显示主板和本地文件的比对结果。', '');
      });
    }).catch((err) => {
      setParamStatus(`读取板端参数失败:\n${err.message}`, 'error');
      throw err;
    });
  }

  function applyParams() {
    return refreshConnectionStatus().then((status) => {
      const mode = chooseConfigWriteMode(status);
      const sourceToml = lastLoadedTomlText || '';
      if (!sourceToml.trim()) {
        throw new Error('当前还没有板端 TOML 基线，请先点击取消更改或等待自动读取成功');
      }
      const nextToml = buildTomlWithCurrentInputs(sourceToml);
      setParamStatus(`正在通过${describeConnectionMode(mode)}应用速度环参数...`, 'warn');
      if (mode === 'runtime') {
        return fetchJson('/api/config/apply', {
          method: 'POST',
          headers: { 'Content-Type': 'text/plain; charset=utf-8' },
          body: nextToml
        }).then((result) => fetchRuntimeConfigText().then((runtimeResult) => {
          const runtimeSummary = summarizeDiff(runtimeResult.toml_text || '', nextToml);
          const localSummary = summarizeDiff(result.local_config_text || '', nextToml);
          lastLoadedTomlText = runtimeResult.toml_text || nextToml;
          loadFieldsFromToml(lastLoadedTomlText);
          setParamStatus(
            (runtimeSummary.same && localSummary.same)
              ? `应用成功。\n通道: ${describeConnectionMode(mode)}\n主板运行态与电脑本地文件都已校验一致。`
              : `应用已返回成功，但主板运行态或电脑本地文件校验存在差异。`,
            (runtimeSummary.same && localSummary.same)
              ? (result.restart_required ? 'warn' : 'ok')
              : 'warn'
          );
          setParamCompare(['[主板运行态]', runtimeSummary.text, '', '[电脑本地文件]', localSummary.text].join('\n'),
            (runtimeSummary.same && localSummary.same) ? 'ok' : 'warn');
        }));
      }

      return fetchJson('/api/config/ssh_push', {
        method: 'POST',
        headers: { 'Content-Type': 'text/plain; charset=utf-8' },
        body: nextToml
      }).then((result) => fetchBoardConfigTextViaSsh().then((sshResult) => {
        const boardSummary = summarizeDiff(sshResult.toml_text || '', nextToml);
        const localSummary = summarizeDiff(result.local_config_text || '', nextToml);
        lastLoadedTomlText = sshResult.toml_text || nextToml;
        loadFieldsFromToml(lastLoadedTomlText);
        setParamStatus(
          (boardSummary.same && localSummary.same)
            ? `离线写入成功。\n通道: ${describeConnectionMode(mode)}\n主板文件与电脑本地文件都已校验一致。`
            : '离线写入已完成，但主板文件或电脑本地文件校验存在差异。',
          (boardSummary.same && localSummary.same) ? 'ok' : 'warn'
        );
        setParamCompare(['[主板文件]', boardSummary.text, '', '[电脑本地文件]', localSummary.text].join('\n'),
          (boardSummary.same && localSummary.same) ? 'ok' : 'warn');
      }));
    }).catch((err) => {
      setParamStatus(`应用参数失败:\n${err.message}`, 'error');
      throw err;
    });
  }

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.round(rect.width * ratio);
    canvas.height = Math.round(rect.height * ratio);
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    draw();
  }

  function visibleSeries() {
    return [
      { key: 'leftActual', label: '左轮实际', color: COLORS.leftActual, visible: leftActualToggle.checked },
      { key: 'rightActual', label: '右轮实际', color: COLORS.rightActual, visible: rightActualToggle.checked },
      { key: 'leftTarget', label: '左轮目标', color: COLORS.leftTarget, visible: leftTargetToggle.checked },
      { key: 'rightTarget', label: '右轮目标', color: COLORS.rightTarget, visible: rightTargetToggle.checked }
    ];
  }

  function resetView() {
    xSpanMs = DEFAULT_X_SPAN_MS;
    yMin = DEFAULT_Y_MIN;
    yMax = DEFAULT_Y_MAX;
    if (latestTimestamp > 0) {
      xCenterMs = followLatestToggle.checked ? latestTimestamp : latestTimestamp - xSpanMs * 0.2;
    } else {
      xCenterMs = 0;
    }
    draw();
  }

  function clearData() {
    points.length = 0;
    latestTimestamp = 0;
    boardTimeOffsetMs = null;
    boardTimeLastMs = null;
    firstArrivalAt = 0;
    hoveredData = null;
    resetView();
    setStatus('历史曲线已清空，等待新的状态流样本。', 'warn');
  }

  function normalizeTimestamp(status) {
    const arrivalMs = Date.now();
    const boardTsMs = Number(status && status.ts_ms);
    if (!Number.isFinite(boardTsMs)) {
      return arrivalMs;
    }
    if (boardTimeOffsetMs === null || boardTimeLastMs === null || boardTsMs < boardTimeLastMs - 500) {
      boardTimeOffsetMs = arrivalMs - boardTsMs;
    }
    boardTimeLastMs = boardTsMs;
    return boardTsMs + boardTimeOffsetMs;
  }

  function pushPointFromStatus(status) {
    const timestampMs = normalizeTimestamp(status);
    if (!firstArrivalAt) {
      firstArrivalAt = timestampMs;
    }
    latestTimestamp = timestampMs;
    const point = {
      t: timestampMs,
      leftActual: Number(status && status.left_filtered_count),
      rightActual: Number(status && status.right_filtered_count),
      leftTarget: Number(status && status.left_target_count),
      rightTarget: Number(status && status.right_target_count)
    };
    points.push(point);
    if (points.length > MAX_POINTS) {
      points.splice(0, points.length - MAX_POINTS);
    }
    if (followLatestToggle.checked) {
      xCenterMs = latestTimestamp;
    } else if (!Number.isFinite(xCenterMs) || xCenterMs === 0) {
      xCenterMs = latestTimestamp;
    }
  }

  function connectWs() {
    if (ws) {
      ws.close();
      ws = null;
    }
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    ws = new WebSocket(`${protocol}//${location.host}/`);
    ws.addEventListener('open', () => {
      connectionText.textContent = 'WebSocket 已连接';
      setStatus('已连接实时状态流，正在接收左右轮目标/实际速度。', 'ok');
      clearTimeout(reconnectTimer);
    });
    ws.addEventListener('message', (event) => {
      let payload = null;
      try {
        payload = JSON.parse(event.data);
      } catch (_) {
        return;
      }
      if (!payload || payload.type !== 'status' || !payload.data) {
        return;
      }
      lastReceivedAt = Date.now();
      if (pauseToggle.checked) {
        draw();
        return;
      }
      pushPointFromStatus(payload.data);
      draw();
    });
    ws.addEventListener('close', () => {
      connectionText.textContent = 'WebSocket 已断开';
      setStatus('实时状态流连接已断开，1 秒后自动重连。', 'warn');
      reconnectTimer = window.setTimeout(connectWs, 1000);
    });
    ws.addEventListener('error', () => {
      connectionText.textContent = 'WebSocket 连接异常';
      setStatus('实时状态流连接异常，准备自动重连。', 'error');
    });
  }

  function computeBounds() {
    const width = canvas.getBoundingClientRect().width || 1;
    const height = canvas.getBoundingClientRect().height || 1;
    const padding = { left: 76, right: 20, top: 20, bottom: 42 };
    return {
      width,
      height,
      padding,
      plotLeft: padding.left,
      plotTop: padding.top,
      plotWidth: Math.max(80, width - padding.left - padding.right),
      plotHeight: Math.max(80, height - padding.top - padding.bottom)
    };
  }

  function xToCanvas(t, bounds) {
    const xMin = xCenterMs - xSpanMs / 2;
    return bounds.plotLeft + ((t - xMin) / xSpanMs) * bounds.plotWidth;
  }

  function yToCanvas(v, bounds) {
    return bounds.plotTop + ((yMax - v) / (yMax - yMin)) * bounds.plotHeight;
  }

  function canvasToTime(x, bounds) {
    const ratio = (x - bounds.plotLeft) / bounds.plotWidth;
    return (xCenterMs - xSpanMs / 2) + ratio * xSpanMs;
  }

  function canvasToValue(y, bounds) {
    const ratio = (y - bounds.plotTop) / bounds.plotHeight;
    return yMax - ratio * (yMax - yMin);
  }

  function drawGrid(bounds) {
    ctx.save();
    ctx.strokeStyle = COLORS.grid;
    ctx.lineWidth = 1;
    ctx.fillStyle = COLORS.subText;
    ctx.font = '12px "JetBrains Mono", monospace';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';

    const yTicks = 6;
    for (let i = 0; i <= yTicks; i += 1) {
      const value = yMin + ((yMax - yMin) * i) / yTicks;
      const y = yToCanvas(value, bounds);
      ctx.beginPath();
      ctx.moveTo(bounds.plotLeft, y);
      ctx.lineTo(bounds.plotLeft + bounds.plotWidth, y);
      ctx.stroke();
      ctx.fillText(formatValue(value), bounds.plotLeft - 10, y);
    }

    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const xTicks = 8;
    const xMin = xCenterMs - xSpanMs / 2;
    for (let i = 0; i <= xTicks; i += 1) {
      const t = xMin + (xSpanMs * i) / xTicks;
      const x = xToCanvas(t, bounds);
      ctx.beginPath();
      ctx.moveTo(x, bounds.plotTop);
      ctx.lineTo(x, bounds.plotTop + bounds.plotHeight);
      ctx.stroke();
      ctx.fillText(formatClock(t), x, bounds.plotTop + bounds.plotHeight + 8);
    }

    ctx.strokeStyle = COLORS.axis;
    ctx.beginPath();
    ctx.moveTo(bounds.plotLeft, bounds.plotTop);
    ctx.lineTo(bounds.plotLeft, bounds.plotTop + bounds.plotHeight);
    ctx.lineTo(bounds.plotLeft + bounds.plotWidth, bounds.plotTop + bounds.plotHeight);
    ctx.stroke();
    ctx.restore();
  }

  function drawSeries(bounds) {
    const xMin = xCenterMs - xSpanMs / 2;
    const xMax = xCenterMs + xSpanMs / 2;
    const seriesList = visibleSeries().filter((item) => item.visible);
    for (const series of seriesList) {
      ctx.save();
      ctx.strokeStyle = series.color;
      ctx.lineWidth = series.key.includes('Target') ? 1.6 : 2.1;
      if (series.key.includes('Target')) {
        ctx.setLineDash([8, 5]);
      }
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < points.length; i += 1) {
        const point = points[i];
        if (point.t < xMin || point.t > xMax) {
          continue;
        }
        const value = point[series.key];
        if (!Number.isFinite(value)) {
          started = false;
          continue;
        }
        const x = xToCanvas(point.t, bounds);
        const y = yToCanvas(value, bounds);
        if (!started) {
          ctx.moveTo(x, y);
          started = true;
        } else {
          ctx.lineTo(x, y);
        }
      }
      ctx.stroke();
      ctx.restore();
    }
  }

  function findNearestPoint(timeMs) {
    if (!points.length) return null;
    let best = null;
    let bestDistance = Infinity;
    for (let i = 0; i < points.length; i += 1) {
      const distance = Math.abs(points[i].t - timeMs);
      if (distance < bestDistance) {
        bestDistance = distance;
        best = points[i];
      }
    }
    return best;
  }

  function drawHover(bounds) {
    if (!hoveredData) return;
    const x = xToCanvas(hoveredData.t, bounds);
    ctx.save();
    ctx.strokeStyle = 'rgba(255,255,255,0.35)';
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(x, bounds.plotTop);
    ctx.lineTo(x, bounds.plotTop + bounds.plotHeight);
    ctx.stroke();
    ctx.restore();
  }

  function updateMeta() {
    const xMin = xCenterMs - xSpanMs / 2;
    const xMax = xCenterMs + xSpanMs / 2;
    sampleCountText.textContent = `${points.length}`;
    windowText.textContent = `${(xSpanMs / 1000).toFixed(2)} s  |  ${formatClock(xMin)} ~ ${formatClock(xMax)}`;
    yRangeText.textContent = `${formatValue(yMin)} ~ ${formatValue(yMax)}`;
    if (hoveredData) {
      cursorText.textContent =
        `${formatClock(hoveredData.t)} | 左实 ${formatValue(hoveredData.leftActual)} | 右实 ${formatValue(hoveredData.rightActual)} | 左目 ${formatValue(hoveredData.leftTarget)} | 右目 ${formatValue(hoveredData.rightTarget)}`;
    } else {
      cursorText.textContent = '--';
    }
    const latest = points[points.length - 1];
    latestText.textContent = latest
      ? `左实 ${formatValue(latest.leftActual)} / 右实 ${formatValue(latest.rightActual)} / 左目 ${formatValue(latest.leftTarget)} / 右目 ${formatValue(latest.rightTarget)}`
      : '--';
    followBadge.textContent = `跟随最新: ${followLatestToggle.checked ? '开' : '关'}`;
    pauseBadge.textContent = `采样: ${pauseToggle.checked ? '暂停' : '实时'}`;
    connectionText.textContent =
      lastReceivedAt > 0 && (Date.now() - lastReceivedAt) < 2000
        ? 'WebSocket 已连接'
        : connectionText.textContent;
  }

  function drawEmpty(bounds) {
    ctx.save();
    ctx.fillStyle = COLORS.subText;
    ctx.font = '16px "Noto Sans SC", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('等待左右轮速度数据...', bounds.plotLeft + bounds.plotWidth / 2, bounds.plotTop + bounds.plotHeight / 2);
    ctx.restore();
  }

  function draw() {
    const bounds = computeBounds();
    ctx.clearRect(0, 0, bounds.width, bounds.height);
    drawGrid(bounds);
    if (!points.length) {
      drawEmpty(bounds);
    } else {
      drawSeries(bounds);
      drawHover(bounds);
    }
    updateMeta();
  }

  function zoomX(scale) {
    xSpanMs = clamp(xSpanMs * scale, XOOM_MIN_SPAN_MS, 5 * 60 * 1000);
    draw();
  }

  function zoomY(scale, anchorValue) {
    const currentSpan = yMax - yMin;
    const newSpan = clamp(currentSpan * scale, YOOM_MIN_SPAN, 10000);
    const anchor = Number.isFinite(anchorValue) ? anchorValue : (yMin + yMax) * 0.5;
    const ratio = (anchor - yMin) / currentSpan;
    yMin = anchor - newSpan * ratio;
    yMax = yMin + newSpan;
    draw();
  }

  canvas.addEventListener('wheel', (event) => {
    event.preventDefault();
    const bounds = computeBounds();
    if (event.shiftKey) {
      zoomX(event.deltaY > 0 ? 1.1 : 0.9);
      return;
    }
    const anchorValue = canvasToValue(event.offsetY, bounds);
    zoomY(event.deltaY > 0 ? 1.1 : 0.9, anchorValue);
  }, { passive: false });

  canvas.addEventListener('mousedown', (event) => {
    dragging = true;
    lastDragX = event.clientX;
    lastDragY = event.clientY;
    followLatestToggle.checked = false;
    canvas.classList.add('dragging');
    draw();
  });

  window.addEventListener('mouseup', () => {
    dragging = false;
    canvas.classList.remove('dragging');
  });

  window.addEventListener('mousemove', (event) => {
    const rect = canvas.getBoundingClientRect();
    if (event.clientX >= rect.left && event.clientX <= rect.right &&
        event.clientY >= rect.top && event.clientY <= rect.bottom) {
      const bounds = computeBounds();
      hoveredData = findNearestPoint(canvasToTime(event.clientX - rect.left, bounds));
    } else {
      hoveredData = null;
    }

    if (dragging) {
      const bounds = computeBounds();
      const dx = event.clientX - lastDragX;
      const dy = event.clientY - lastDragY;
      xCenterMs -= (dx / bounds.plotWidth) * xSpanMs;
      const ySpan = yMax - yMin;
      const dyValue = (dy / bounds.plotHeight) * ySpan;
      yMin += dyValue;
      yMax += dyValue;
      lastDragX = event.clientX;
      lastDragY = event.clientY;
    }
    draw();
  });

  canvas.addEventListener('mouseleave', () => {
    if (!dragging) {
      hoveredData = null;
      draw();
    }
  });

  followLatestToggle.addEventListener('change', () => {
    if (followLatestToggle.checked && latestTimestamp > 0) {
      xCenterMs = latestTimestamp;
    }
    draw();
  });

  pauseToggle.addEventListener('change', () => {
    setStatus(pauseToggle.checked ? '已暂停接收新点，当前图表保持静止。' : '已恢复实时接收。', pauseToggle.checked ? 'warn' : 'ok');
    draw();
  });

  [leftActualToggle, rightActualToggle, leftTargetToggle, rightTargetToggle].forEach((el) => {
    el.addEventListener('change', draw);
  });

  resetViewBtn.addEventListener('click', () => {
    followLatestToggle.checked = true;
    resetView();
    setStatus('视图已重置，重新跟随最新数据。', 'ok');
  });

  clearDataBtn.addEventListener('click', clearData);
  applyParamsBtn.addEventListener('click', () => { applyParams().catch(() => {}); });
  cancelParamsBtn.addEventListener('click', () => { fillParamsFromBoard().catch(() => {}); });

  window.addEventListener('resize', resizeCanvas);

  renderParamFields();
  resizeCanvas();
  resetView();
  connectWs();
  fillParamsFromBoard().catch(() => {});
})();
