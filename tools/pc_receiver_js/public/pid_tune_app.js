(function () {
  const receiverCore = window.SharedReceiverCore;

  const routeStateSelect = document.getElementById('routeStateSelect');
  const btnLoad = document.getElementById('btnLoad');
  const btnApply = document.getElementById('btnApply');
  const btnResetCharts = document.getElementById('btnResetCharts');
  const chipConn = document.getElementById('chipConn');
  const logBox = document.getElementById('logBox');

  const vPosErr = document.getElementById('vPosErr');
  const vYawErr = document.getElementById('vYawErr');
  const vYawRef = document.getElementById('vYawRef');
  const vYawMeas = document.getElementById('vYawMeas');

  const posMeta = document.getElementById('posMeta');
  const yawMeta = document.getElementById('yawMeta');

  const posErrChart = document.getElementById('posErrChart');
  const yawErrChart = document.getElementById('yawErrChart');
  const posErrCtx = posErrChart.getContext('2d');
  const yawErrCtx = yawErrChart.getContext('2d');

  const ROUTE_FIELD_KEYS = [
    'position_dynamic_kp_base',
    'position_ki',
    'position_kd',
    'position_max_integral',
    'position_max_output',
    'yaw_rate_kp',
    'yaw_rate_ki',
    'yaw_rate_kd',
    'yaw_rate_max_integral',
    'yaw_rate_max_output',
    'yaw_rate_ref_limit_dps'
  ];

  const MOTOR_FIELD_IDS = {
    left_kp: 'left_kp',
    right_kp: 'right_kp',
    left_ki: 'left_ki',
    right_ki: 'right_ki',
    left_kd: 'left_kd',
    right_kd: 'right_kd',
    integral_limit: 'motor_integral_limit',
    max_output_step: 'motor_max_output_step',
    correction_limit: 'motor_correction_limit',
    duty_limit: 'motor_duty_limit'
  };

  const routeInputs = {};
  for (const key of ROUTE_FIELD_KEYS) {
    routeInputs[key] = document.getElementById(key);
  }
  const motorInputs = {};
  for (const key of Object.keys(MOTOR_FIELD_IDS)) {
    motorInputs[key] = document.getElementById(MOTOR_FIELD_IDS[key]);
  }

  const POS_BUF = [];
  const YAW_BUF = [];
  const MAX_POINTS = 320;

  let latestTomlText = '';
  let lastStatusAtMs = 0;

  function setLog(text, level) {
    logBox.className = `log${level ? ` ${level}` : ''}`;
    logBox.textContent = text;
  }

  function setConnChip(text, level) {
    chipConn.className = `chip${level ? ` ${level}` : ''}`;
    chipConn.textContent = text;
  }

  function toTextValue(value) {
    if (value === undefined || value === null) return '';
    return String(value).trim();
  }

  function formatValue(value, digits) {
    const n = Number(value);
    if (!Number.isFinite(n)) return '--';
    return n.toFixed(digits);
  }

  function limitBuffer(buf, value) {
    if (!Number.isFinite(value)) return;
    buf.push(value);
    if (buf.length > MAX_POINTS) {
      buf.splice(0, buf.length - MAX_POINTS);
    }
  }

  function calcYRange(values, fallbackAbs) {
    if (!Array.isArray(values) || values.length < 1) {
      return { min: -fallbackAbs, max: fallbackAbs };
    }
    let maxAbs = 0;
    for (const v of values) {
      const n = Math.abs(Number(v) || 0);
      if (n > maxAbs) maxAbs = n;
    }
    maxAbs = Math.max(1e-3, maxAbs * 1.2, fallbackAbs * 0.25);
    return { min: -maxAbs, max: maxAbs };
  }

  function drawCharts() {
    const posRange = calcYRange(POS_BUF, 50);
    const yawRange = calcYRange(YAW_BUF, 180);
    receiverCore.drawCurveChartToCanvas(posErrChart, posErrCtx, POS_BUF, {
      yMin: posRange.min,
      yMax: posRange.max,
      lineColor: '#38bdf8',
      backgroundColor: '#020914'
    });
    receiverCore.drawCurveChartToCanvas(yawErrChart, yawErrCtx, YAW_BUF, {
      yMin: yawRange.min,
      yMax: yawRange.max,
      lineColor: '#f97316',
      backgroundColor: '#020914'
    });

    const posNow = POS_BUF.length > 0 ? POS_BUF[POS_BUF.length - 1] : null;
    const yawNow = YAW_BUF.length > 0 ? YAW_BUF[YAW_BUF.length - 1] : null;
    const posPeak = POS_BUF.length > 0 ? Math.max(...POS_BUF.map((v) => Math.abs(v))) : null;
    const yawPeak = YAW_BUF.length > 0 ? Math.max(...YAW_BUF.map((v) => Math.abs(v))) : null;
    posMeta.textContent = `当前: ${formatValue(posNow, 3)} | 峰值(abs): ${formatValue(posPeak, 3)} | 点数: ${POS_BUF.length}`;
    yawMeta.textContent = `当前: ${formatValue(yawNow, 3)} | 峰值(abs): ${formatValue(yawPeak, 3)} | 点数: ${YAW_BUF.length}`;
  }

  function sectionNameForRouteState() {
    return `pid.route_line_follow.${routeStateSelect.value}`;
  }

  function parseSectionValue(lines, sectionName, key) {
    let inSection = false;
    const sectionRegex = /^\s*\[([^\]]+)\]\s*$/;
    const keyRegex = new RegExp(`^\\s*${key.replace(/[.*+?^${}()|[\\]\\]/g, '\\$&')}\\s*=\\s*(.+)$`);
    for (const line of lines) {
      const sm = line.match(sectionRegex);
      if (sm) {
        inSection = sm[1].trim() === sectionName;
        continue;
      }
      if (!inSection) continue;
      const km = line.match(keyRegex);
      if (!km) continue;
      const raw = String(km[1]).split('#')[0].trim();
      return raw;
    }
    return null;
  }

  function findSectionRange(lines, sectionName) {
    const sectionRegex = /^\s*\[([^\]]+)\]\s*$/;
    let start = -1;
    let end = lines.length;
    for (let i = 0; i < lines.length; i += 1) {
      const m = lines[i].match(sectionRegex);
      if (!m) continue;
      const sec = m[1].trim();
      if (start >= 0) {
        end = i;
        break;
      }
      if (sec === sectionName) {
        start = i + 1;
      }
    }
    return { start, end };
  }

  function setSectionKey(lines, sectionName, key, rawValue) {
    const keyRegex = new RegExp(`^\\s*${key.replace(/[.*+?^${}()|[\\]\\]/g, '\\$&')}\\s*=`);
    let range = findSectionRange(lines, sectionName);
    if (range.start < 0) {
      if (lines.length > 0 && lines[lines.length - 1].trim() !== '') {
        lines.push('');
      }
      lines.push(`[${sectionName}]`);
      lines.push(`${key} = ${rawValue}`);
      return;
    }

    for (let i = range.start; i < range.end; i += 1) {
      if (keyRegex.test(lines[i])) {
        lines[i] = `${key} = ${rawValue}`;
        return;
      }
    }

    let insertAt = range.end;
    while (insertAt > range.start && lines[insertAt - 1].trim() === '') {
      insertAt -= 1;
    }
    lines.splice(insertAt, 0, `${key} = ${rawValue}`);
  }

  function loadInputsFromToml(tomlText) {
    const lines = String(tomlText || '').split('\n');
    const routeSection = sectionNameForRouteState();

    for (const key of ROUTE_FIELD_KEYS) {
      const value = parseSectionValue(lines, routeSection, key);
      routeInputs[key].value = toTextValue(value);
    }

    for (const [key, input] of Object.entries(motorInputs)) {
      const value = parseSectionValue(lines, 'pid.motor_speed', key);
      input.value = toTextValue(value);
    }
  }

  function collectInputsForApply() {
    const routeData = {};
    for (const key of ROUTE_FIELD_KEYS) {
      const raw = String(routeInputs[key].value || '').trim();
      if (!raw) {
        throw new Error(`缺少参数: ${key}`);
      }
      routeData[key] = raw;
    }

    const motorData = {};
    for (const [key, input] of Object.entries(motorInputs)) {
      const raw = String(input.value || '').trim();
      if (!raw) {
        throw new Error(`缺少参数: pid.motor_speed.${key}`);
      }
      motorData[key] = raw;
    }

    return { routeData, motorData };
  }

  function buildUpdatedTomlText(baseTomlText, routeSection, routeData, motorData) {
    const lines = String(baseTomlText || '').split('\n');
    for (const [key, raw] of Object.entries(routeData)) {
      setSectionKey(lines, routeSection, key, raw);
    }
    for (const [key, raw] of Object.entries(motorData)) {
      setSectionKey(lines, 'pid.motor_speed', key, raw);
    }
    return lines.join('\n');
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
    if (!response.ok || data.ok === false) {
      throw new Error(data.message || `HTTP ${response.status}`);
    }
    return data;
  }

  async function loadConfigToForm() {
    const result = await fetchJson('/api/config/current', { cache: 'no-store' });
    latestTomlText = String(result.toml_text || '');
    loadInputsFromToml(latestTomlText);
    setLog(
      [
        '已读取主板当前配置。',
        `loaded_path: ${result.loaded_path || '--'}`,
        `route_state: ${routeStateSelect.value}`,
        `time: ${new Date().toLocaleTimeString()}`
      ].join('\n'),
      'ok'
    );
  }

  async function applyFormToBoard() {
    if (!latestTomlText) {
      await loadConfigToForm();
    }

    const routeSection = sectionNameForRouteState();
    const { routeData, motorData } = collectInputsForApply();
    const updatedToml = buildUpdatedTomlText(latestTomlText, routeSection, routeData, motorData);

    const result = await fetchJson('/api/config/apply', {
      method: 'POST',
      headers: { 'Content-Type': 'text/plain; charset=utf-8' },
      body: updatedToml
    });

    latestTomlText = updatedToml;
    const restartKeys = Array.isArray(result.restart_required_keys) ? result.restart_required_keys : [];
    setLog(
      [
        '应用成功。',
        `route_state: ${routeStateSelect.value}`,
        `restart_required: ${result.restart_required ? 'yes' : 'no'}`,
        `restart_keys: ${restartKeys.length > 0 ? restartKeys.join(', ') : 'none'}`,
        `time: ${new Date().toLocaleTimeString()}`
      ].join('\n'),
      restartKeys.length > 0 ? 'warn' : 'ok'
    );
  }

  function onStatus(status) {
    const posErr = Number(status && status.pid_common_position_pid_error);
    const yawErr = Number(status && status.pid_common_yaw_rate_error_dps);
    const yawRef = Number(status && status.pid_common_yaw_rate_ref_dps);
    const yawMeas = Number(status && status.pid_common_measured_yaw_rate_dps);

    vPosErr.textContent = formatValue(posErr, 3);
    vYawErr.textContent = formatValue(yawErr, 3);
    vYawRef.textContent = formatValue(yawRef, 2);
    vYawMeas.textContent = formatValue(yawMeas, 2);

    limitBuffer(POS_BUF, posErr);
    limitBuffer(YAW_BUF, yawErr);
    drawCharts();

    lastStatusAtMs = Date.now();
    setConnChip(`连接状态：在线 @ ${new Date(lastStatusAtMs).toLocaleTimeString()}`, 'ok');
  }

  async function pullStatus() {
    try {
      const status = await receiverCore.fetchJsonNoStore('/api/status');
      onStatus(status);
    } catch (err) {
      const staleMs = Date.now() - lastStatusAtMs;
      const staleText = Number.isFinite(staleMs) ? `${Math.round(staleMs)}ms` : '--';
      setConnChip(`连接状态：离线 (${staleText})`, 'error');
    }
  }

  btnLoad.addEventListener('click', () => {
    loadConfigToForm().catch((err) => {
      setLog(`读取失败: ${err.message}`, 'error');
    });
  });

  btnApply.addEventListener('click', () => {
    applyFormToBoard().catch((err) => {
      setLog(`应用失败: ${err.message}`, 'error');
    });
  });

  btnResetCharts.addEventListener('click', () => {
    POS_BUF.length = 0;
    YAW_BUF.length = 0;
    drawCharts();
    setLog('已清空误差曲线缓存。', 'warn');
  });

  routeStateSelect.addEventListener('change', () => {
    if (latestTomlText) {
      loadInputsFromToml(latestTomlText);
      setLog(`已切换状态并加载本地缓存参数: ${routeStateSelect.value}`, 'warn');
    }
  });

  drawCharts();
  setInterval(pullStatus, 120);
  pullStatus();
  loadConfigToForm().catch((err) => {
    setLog(`初始化读取失败: ${err.message}`, 'error');
  });
})();
