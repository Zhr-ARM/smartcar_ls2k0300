(function () {
  const configEditor = document.getElementById('configEditor');
  const applyStatusBox = document.getElementById('applyStatusBox');
  const compareStatusBox = document.getElementById('compareStatusBox');
  const presetSelect = document.getElementById('presetSelect');
  const presetLabelInput = document.getElementById('presetLabelInput');
  const presetIdInput = document.getElementById('presetIdInput');
  const boardConfigBaseUrlInput = document.getElementById('boardConfigBaseUrlInput');
  const boardSshHostInput = document.getElementById('boardSshHostInput');
  const boardSshPortInput = document.getElementById('boardSshPortInput');
  const boardSshUserInput = document.getElementById('boardSshUserInput');
  const boardSshTargetPathInput = document.getElementById('boardSshTargetPathInput');
  const pcReceiverIpInput = document.getElementById('pcReceiverIpInput');
  const assistantReceiverIpInput = document.getElementById('assistantReceiverIpInput');
  const currentBaseUrlText = document.getElementById('currentBaseUrlText');
  const upstreamBaseUrlText = document.getElementById('upstreamBaseUrlText');
  const sshTargetText = document.getElementById('sshTargetText');
  const loadedPathText = document.getElementById('loadedPathText');
  const lastActionText = document.getElementById('lastActionText');
  const restartRequiredList = document.getElementById('restartRequiredList');
  const runtimeHttpBadge = document.getElementById('runtimeHttpBadge');
  const sshBadge = document.getElementById('sshBadge');
  const runtimeOverviewText = document.getElementById('runtimeOverviewText');
  const sshOverviewText = document.getElementById('sshOverviewText');
  const runtimeOverviewCard = document.getElementById('runtimeOverviewCard');
  const sshOverviewCard = document.getElementById('sshOverviewCard');

  let lastConnectionStatus = null;
  let connectionStore = { active_preset: '', presets: [] };
  let lastBoardFileTomlText = '';

  function setStatus(text, klass) {
    applyStatusBox.className = `status-box ${klass || ''}`.trim();
    applyStatusBox.textContent = text;
  }

  function setCompareStatus(text, klass) {
    compareStatusBox.className = `status-box ${klass || ''}`.trim();
    compareStatusBox.textContent = text;
  }

  function setBadge(el, text, klass) {
    el.className = `status-badge ${klass || ''}`.trim();
    el.textContent = text;
  }

  function setRestartKeys(keys) {
    restartRequiredList.innerHTML = '';
    if (!Array.isArray(keys) || keys.length === 0) {
      const li = document.createElement('li');
      li.textContent = '当前无';
      restartRequiredList.appendChild(li);
      return;
    }
    for (const key of keys) {
      const li = document.createElement('li');
      li.textContent = key;
      restartRequiredList.appendChild(li);
    }
  }

  function currentConnectionSettingsFromInputs() {
    return {
      id: (presetIdInput.value || '').trim(),
      label: (presetLabelInput.value || '').trim(),
      board_config_base_url: (boardConfigBaseUrlInput.value || '').trim(),
      board_ssh_host: (boardSshHostInput.value || '').trim(),
      board_ssh_port: Number((boardSshPortInput.value || '').trim() || '22'),
      board_ssh_user: (boardSshUserInput.value || '').trim(),
      board_ssh_target_path: (boardSshTargetPathInput.value || '').trim(),
      pc_receiver_ip: (pcReceiverIpInput.value || '').trim(),
      assistant_receiver_ip: (assistantReceiverIpInput.value || '').trim()
    };
  }

  function fillConnectionSettingsInputs(settings) {
    const s = settings || {};
    presetIdInput.value = s.id || '';
    presetLabelInput.value = s.label || '';
    boardConfigBaseUrlInput.value = s.board_config_base_url || '';
    boardSshHostInput.value = s.board_ssh_host || '';
    boardSshPortInput.value = String(s.board_ssh_port || 22);
    boardSshUserInput.value = s.board_ssh_user || '';
    boardSshTargetPathInput.value = s.board_ssh_target_path || '';
    pcReceiverIpInput.value = s.pc_receiver_ip || '';
    assistantReceiverIpInput.value = s.assistant_receiver_ip || '';
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
      const message = parsed && parsed.message ? parsed.message : `HTTP ${response.status}`;
      throw new Error(message);
    }
    return parsed;
  }

  function formatTimestampForFilename() {
    const d = new Date();
    const pad = (n) => String(n).padStart(2, '0');
    return `${d.getFullYear()}${pad(d.getMonth() + 1)}${pad(d.getDate())}_${pad(d.getHours())}${pad(d.getMinutes())}${pad(d.getSeconds())}`;
  }

  function downloadTextFile(filename, text) {
    const blob = new Blob([text || ''], { type: 'text/plain;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    a.click();
    URL.revokeObjectURL(url);
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
      return { same: true, text: '主板文件与当前编辑框完全一致。' };
    }
    const normalizedBoardText = normalizeTomlForComparison(boardText);
    const normalizedEditorText = normalizeTomlForComparison(editorText);
    if (normalizedBoardText === normalizedEditorText) {
      return {
        same: true,
        text: '主板文件与当前编辑框内容一致。原始文本仅存在换行、行尾空格或末尾空行差异。'
      };
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
        if (samples.length < 6) {
          samples.push(`L${i + 1}\n主板: ${boardLine || '(空)'}\n当前: ${editorLine || '(空)'}`);
        }
      }
    }
    const summary = [
      '主板文件与当前编辑框存在差异。',
      `首个差异行: L${firstDiffLine}`,
      `差异总行数: ${diffCount}`,
      '',
      ...samples
    ].join('\n');
    return { same: false, text: summary };
  }

  function syncPresetIpsIntoTomlText(text) {
    const current = currentConnectionSettingsFromInputs();
    const webIp = current.pc_receiver_ip || '';
    const assistantIp = current.assistant_receiver_ip || current.pc_receiver_ip || '';
    if (!webIp || !assistantIp) {
      throw new Error('当前预设缺少电脑接收端 IP，请先补全并保存连接设置');
    }
    const lines = String(text || '').split('\n');
    let section = '';
    let replacedWeb = false;
    let replacedAssistant = false;
    for (let i = 0; i < lines.length; i += 1) {
      const line = lines[i];
      const trimmed = line.trim();
      if (trimmed.startsWith('[') && trimmed.endsWith(']')) {
        section = trimmed.slice(1, -1).trim();
        continue;
      }
      if (!trimmed.startsWith('server_ip')) continue;
      if (section === 'vision.runtime.web') {
        lines[i] = line.replace(/server_ip\s*=\s*"[^"]*"/, `server_ip = "${webIp}"`);
        replacedWeb = true;
        continue;
      }
      if (section === 'vision.runtime.assistant') {
        lines[i] = line.replace(/server_ip\s*=\s*"[^"]*"/, `server_ip = "${assistantIp}"`);
        replacedAssistant = true;
      }
    }
    if (!replacedWeb) throw new Error('未找到 [vision.runtime.web] 下的 server_ip');
    if (!replacedAssistant) throw new Error('未找到 [vision.runtime.assistant] 下的 server_ip');
    return lines.join('\n');
  }

  function syncPresetIpsIntoEditor() {
    const synced = syncPresetIpsIntoTomlText(configEditor.value || '');
    configEditor.value = synced;
    return synced;
  }

  function activePreset() {
    if (!connectionStore || !Array.isArray(connectionStore.presets)) return null;
    return connectionStore.presets.find((preset) => preset.id === connectionStore.active_preset) || connectionStore.presets[0] || null;
  }

  function renderPresetSelect() {
    presetSelect.innerHTML = '';
    const presets = Array.isArray(connectionStore.presets) ? connectionStore.presets : [];
    for (const preset of presets) {
      const option = document.createElement('option');
      option.value = preset.id;
      option.textContent = `${preset.label || preset.id} (${preset.id})`;
      if (preset.id === connectionStore.active_preset) option.selected = true;
      presetSelect.appendChild(option);
    }
    const current = activePreset();
    if (current) fillConnectionSettingsInputs(current);
  }

  function setConnectionStore(store, preserveInputs = false) {
    connectionStore = store && Array.isArray(store.presets)
      ? { active_preset: store.active_preset || '', presets: store.presets.slice() }
      : { active_preset: '', presets: [] };
    renderPresetSelect();
    if (preserveInputs) {
      fillConnectionSettingsInputs(currentConnectionSettingsFromInputs());
    }
  }

  function renderConnectionStatus(result) {
    lastConnectionStatus = result || null;
    if (result && result.connection_store) {
      setConnectionStore(result.connection_store);
    } else if (result && result.connection_settings) {
      fillConnectionSettingsInputs(result.connection_settings);
    }
    currentBaseUrlText.textContent = `网页代理: ${window.location.origin}`;
    upstreamBaseUrlText.textContent = `主板配置服务: ${result && result.board_config_base_url ? result.board_config_base_url : '--'}`;
    if (result && result.ssh_transport) {
      const ssh = result.ssh_transport;
      sshTargetText.textContent = `SSH 目标: ${ssh.user}@${ssh.host}:${ssh.target_path} (port ${ssh.port})`;
    } else {
      sshTargetText.textContent = 'SSH 目标: --';
    }

    const runtimeOnline = !!(result && result.runtime_http && result.runtime_http.online);
    const runtimeError = result && result.runtime_http ? (result.runtime_http.error || '') : '';
    setBadge(runtimeHttpBadge, runtimeOnline ? '主板程序在线' : '主板程序离线', runtimeOnline ? 'ok' : 'error');
    runtimeOverviewCard.classList.toggle('offline', !runtimeOnline);
    runtimeOverviewText.textContent = runtimeOnline
      ? '主板配置 HTTP 服务已在线。\n现在可以“读取当前配置”和“应用到主板”，并立即热更新可热更参数。'
      : `主板配置 HTTP 服务未连通。\n常见原因：主板程序未启动、程序异常退出、主板 IP 错误。\n${runtimeError ? `错误: ${runtimeError}` : ''}`.trim();

    const sshOnline = !!(result && result.ssh_transport && result.ssh_transport.online);
    const sshError = result && result.ssh_transport ? (result.ssh_transport.error || '') : '';
    setBadge(sshBadge, sshOnline ? 'SSH 可直连' : 'SSH 不可用', sshOnline ? 'ok' : 'error');
    sshOverviewCard.classList.toggle('offline', !sshOnline);
    sshOverviewText.textContent = sshOnline
      ? '主板 SSH 端口可连通。\n即使主板程序没跑，也可以把当前 TOML 直接写到主板文件系统。'
      : `主板 SSH 端口未连通。\n如果你要用离线落盘，需要先确保主板 SSH 服务可访问。\n${sshError ? `错误: ${sshError}` : ''}`.trim();
  }

  async function refreshConnectionStatus() {
    try {
      const result = await fetchJson('/api/config/status', { cache: 'no-store' });
      renderConnectionStatus(result);
      return result;
    } catch (err) {
      lastConnectionStatus = null;
      currentBaseUrlText.textContent = `网页代理: ${window.location.origin}`;
      upstreamBaseUrlText.textContent = '主板配置服务: --';
      sshTargetText.textContent = 'SSH 目标: --';
      setBadge(runtimeHttpBadge, '状态未知', 'warn');
      setBadge(sshBadge, '状态未知', 'warn');
      runtimeOverviewCard.classList.add('offline');
      sshOverviewCard.classList.add('offline');
      runtimeOverviewText.textContent = `无法从电脑侧代理获取连接状态。\n${err.message}`;
      sshOverviewText.textContent = `无法从电脑侧代理获取连接状态。\n${err.message}`;
      throw err;
    }
  }

  async function loadConnectionSettings() {
    const result = await fetchJson('/api/config/connection_settings', { cache: 'no-store' });
    if (result.connection_store) {
      setConnectionStore(result.connection_store);
    } else {
      fillConnectionSettingsInputs(result.connection_settings || {});
    }
    return result.connection_store || null;
  }

  async function saveConnectionSettings() {
    const currentActiveId = connectionStore.active_preset;
    const activePresetData = (connectionStore.presets || []).find((preset) => preset.id === currentActiveId) || {};
    const editedPreset = Object.assign({}, activePresetData, currentConnectionSettingsFromInputs());
    const presets = (connectionStore && Array.isArray(connectionStore.presets) ? connectionStore.presets.slice() : []);
    const activeIndex = presets.findIndex((preset) => preset.id === currentActiveId);
    if (activeIndex >= 0) {
      presets.splice(activeIndex, 1);
    }
    const index = presets.findIndex((preset) => preset.id === editedPreset.id);
    if (index >= 0) {
      presets[index] = editedPreset;
    } else {
      presets.push(editedPreset);
    }
    const payload = {
      active_preset: editedPreset.id,
      presets
    };
    const result = await fetchJson('/api/config/connection_settings', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json; charset=utf-8' },
      body: JSON.stringify(payload)
    });
    renderConnectionStatus(result);
    return result;
  }

  function createPresetIdFromLabel(label) {
    const base = (label || 'preset')
      .trim()
      .toLowerCase()
      .replace(/[^a-z0-9._-]+/g, '_')
      .replace(/^_+|_+$/g, '') || 'preset';
    let candidate = base;
    let suffix = 2;
    while ((connectionStore.presets || []).some((preset) => preset.id === candidate)) {
      candidate = `${base}_${suffix++}`;
    }
    return candidate;
  }

  function createNewPresetFromCurrent() {
    const source = Object.assign({}, activePreset() || {}, currentConnectionSettingsFromInputs());
    const newLabel = `${source.label || '新预设'} 副本`;
    const newPreset = Object.assign({}, source, {
      id: createPresetIdFromLabel(newLabel),
      label: newLabel
    });
    const presets = (connectionStore.presets || []).slice();
    presets.push(newPreset);
    setConnectionStore({ active_preset: newPreset.id, presets });
  }

  function deleteCurrentPreset() {
    const presets = (connectionStore.presets || []).slice();
    if (presets.length <= 1) {
      throw new Error('至少需要保留一个连接预设');
    }
    const currentId = (presetIdInput.value || '').trim() || connectionStore.active_preset;
    const filtered = presets.filter((preset) => preset.id !== currentId);
    if (filtered.length === presets.length) {
      throw new Error('当前预设不存在，无法删除');
    }
    setConnectionStore({
      active_preset: filtered[0].id,
      presets: filtered
    });
  }

  async function loadCurrentConfig() {
    await refreshConnectionStatus().catch(() => {});
    lastActionText.textContent = '最近操作: 正在读取当前配置...';
    setStatus('正在通过电脑侧网页代理读取主板当前配置...', 'warn');
    try {
      const result = await fetchJson('/api/config/current', { cache: 'no-store' });
      configEditor.value = result.toml_text || '';
      loadedPathText.textContent = `配置路径: ${result.loaded_path || '--'}`;
      upstreamBaseUrlText.textContent = `主板配置服务: ${result.board_config_base_url || '--'}`;
      lastActionText.textContent = `最近操作: 读取成功 @ ${new Date().toLocaleTimeString()}`;
      setStatus('已从主板读取当前配置。', 'ok');
      setRestartKeys([]);
      await refreshConnectionStatus().catch(() => {});
    } catch (err) {
      loadedPathText.textContent = '配置路径: --';
      lastActionText.textContent = `最近操作: 读取失败 @ ${new Date().toLocaleTimeString()}`;
      setStatus(`读取失败:\n${err.message}`, 'error');
    }
  }

  async function fetchCurrentConfigFromRuntime() {
    const result = await fetchJson('/api/config/current', { cache: 'no-store' });
    loadedPathText.textContent = `配置路径: ${result.loaded_path || '--'}`;
    upstreamBaseUrlText.textContent = `主板配置服务: ${result.board_config_base_url || '--'}`;
    return result;
  }

  function buildApplyStatusText(applyResult, verifySummary) {
    const lines = [];
    const restartKeys = Array.isArray(applyResult && applyResult.restart_required_keys)
      ? applyResult.restart_required_keys
      : [];
    if (verifySummary.same) {
      lines.push('应用请求已成功发送到主板，且回读校验一致。');
    } else {
      lines.push('主板返回应用成功，但回读校验发现“当前加载配置”与本次提交内容不一致。');
    }
    lines.push(`主板配置服务: ${(applyResult && applyResult.board_config_base_url) || '--'}`);
    lines.push(`当前加载路径: ${(applyResult && applyResult.loaded_path) || '--'}`);
    lines.push(`需要重启的参数数: ${restartKeys.length}`);
    if (restartKeys.length > 0) {
      lines.push('说明: 这些参数已经写入文件，但运行中的主板进程需要重启后才会完全生效。');
    } else {
      lines.push('说明: 这次提交未命中“需重启参数”，可热更新部分应已立即生效。');
    }
    if (!verifySummary.same) {
      lines.push('说明: 下方“比较结果”展示的是主板当前加载配置与本次提交内容的差异。');
    }
    const boardVerify = applyResult && applyResult.board_verify;
    if (boardVerify) {
      lines.push(
        boardVerify.same
          ? '板端一致性校验: 通过'
          : `板端一致性校验: 未通过${boardVerify.message ? ` (${boardVerify.message})` : ''}`
      );
    }
    const localSync = applyResult && applyResult.local_sync;
    if (localSync && localSync.ok) {
      lines.push(`电脑本地参数文件: 已更新 (${localSync.path || '--'})`);
    } else if (localSync && !localSync.ok) {
      lines.push(`电脑本地参数文件: 更新失败 (${localSync.path || '--'})`);
      lines.push(`失败原因: ${localSync.message || '未知错误'}`);
    } else if (applyResult && applyResult.local_config_path) {
      lines.push(`电脑本地参数文件: ${applyResult.local_config_path} (未返回同步结果)`);
    }
    const localVerify = applyResult && applyResult.local_verify;
    if (localVerify) {
      lines.push(
        localVerify.same
          ? '本地文件一致性校验: 通过'
          : `本地文件一致性校验: 未通过${localVerify.message ? ` (${localVerify.message})` : ''}`
      );
    }
    return lines.join('\n');
  }

  async function applyCurrentConfig() {
    let tomlText = configEditor.value;
    if (!tomlText.trim()) {
      setStatus('TOML 文本为空，不能应用。', 'error');
      return;
    }
    tomlText = syncPresetIpsIntoTomlText(tomlText);
    configEditor.value = tomlText;
    await refreshConnectionStatus().catch(() => {});
    lastActionText.textContent = '最近操作: 正在应用配置...';
    setStatus('正在通过电脑侧网页代理发送配置到主板并校验...', 'warn');
    try {
      const result = await fetchJson('/api/config/apply', {
        method: 'POST',
        headers: { 'Content-Type': 'text/plain; charset=utf-8' },
        body: tomlText
      });
      const currentResult = await fetchCurrentConfigFromRuntime();
      const verifySummary = summarizeDiff(currentResult.toml_text || '', tomlText);
      lastActionText.textContent = `最近操作: 应用成功 @ ${new Date().toLocaleTimeString()}`;
      setRestartKeys(result.restart_required_keys || []);
      setCompareStatus(verifySummary.text, verifySummary.same ? 'ok' : 'warn');
      setStatus(
        buildApplyStatusText(result, verifySummary),
        (verifySummary.same &&
         (!result.board_verify || result.board_verify.same) &&
         (!result.local_verify || result.local_verify.same) &&
         (!result.local_sync || result.local_sync.ok))
          ? (result.restart_required ? 'warn' : 'ok')
          : 'warn'
      );
      await refreshConnectionStatus().catch(() => {});
    } catch (err) {
      lastActionText.textContent = `最近操作: 应用失败 @ ${new Date().toLocaleTimeString()}`;
      setStatus(`应用失败:\n${err.message}`, 'error');
    }
  }

  async function sshPushCurrentConfig() {
    let tomlText = configEditor.value;
    if (!tomlText.trim()) {
      setStatus('TOML 文本为空，不能通过 SSH 写入。', 'error');
      return;
    }
    tomlText = syncPresetIpsIntoTomlText(tomlText);
    configEditor.value = tomlText;
    await refreshConnectionStatus().catch(() => {});
    lastActionText.textContent = '最近操作: 正在通过 SSH 写入主板...';
    setStatus('正在通过电脑侧代理使用 SSH 把 TOML 写入主板...', 'warn');
    try {
      const result = await fetchJson('/api/config/ssh_push', {
        method: 'POST',
        headers: { 'Content-Type': 'text/plain; charset=utf-8' },
        body: tomlText
      });
      const boardText = await fetchBoardFileViaSsh();
      const summary = summarizeDiff(boardText, tomlText);
      if (result.ssh_target) {
        sshTargetText.textContent =
          `SSH 目标: ${result.ssh_target.user}@${result.ssh_target.host}:${result.ssh_target.target_path} (port ${result.ssh_target.port})`;
      }
      lastActionText.textContent = `最近操作: SSH 写入成功 @ ${new Date().toLocaleTimeString()}`;
      setStatus(
        [
          summary.same
            ? 'SSH 写入成功，并已回读校验一致。\n配置文件已经落盘到主板，但不会自动热更新当前运行进程。'
            : 'SSH 写入已完成，但回读校验发现主板文件与当前写入内容仍有差异，请看下方比较结果。',
          result.local_sync
            ? (result.local_sync.ok
              ? `电脑本地参数文件已同步更新: ${result.local_sync.path || '--'}`
              : `电脑本地参数文件同步失败: ${result.local_sync.path || '--'}\n失败原因: ${result.local_sync.message || '未知错误'}`)
            : (result.local_config_path
              ? `电脑本地参数文件路径: ${result.local_config_path} (未返回同步结果)`
              : '电脑本地参数文件: 未返回同步结果')
        ].join('\n'),
        (summary.same && (!result.local_sync || result.local_sync.ok)) ? 'ok' : 'warn'
      );
      setCompareStatus(summary.text, summary.same ? 'ok' : 'warn');
      await refreshConnectionStatus().catch(() => {});
    } catch (err) {
      lastActionText.textContent = `最近操作: SSH 写入失败 @ ${new Date().toLocaleTimeString()}`;
      setStatus(`SSH 写入失败:\n${err.message}`, 'error');
    }
  }

  async function sshPullCurrentConfig() {
    await refreshConnectionStatus().catch(() => {});
    lastActionText.textContent = '最近操作: 正在通过 SSH 读取主板配置...';
    setStatus('正在通过电脑侧代理使用 SSH 从主板读取 TOML...', 'warn');
    try {
      const result = await fetchJson('/api/config/ssh_pull', { cache: 'no-store' });
      lastBoardFileTomlText = result.toml_text || '';
      configEditor.value = result.toml_text || '';
      if (result.ssh_target) {
        sshTargetText.textContent =
          `SSH 目标: ${result.ssh_target.user}@${result.ssh_target.host}:${result.ssh_target.target_path} (port ${result.ssh_target.port})`;
      }
      lastActionText.textContent = `最近操作: SSH 读取成功 @ ${new Date().toLocaleTimeString()}`;
      setStatus('SSH 读取成功。\n已把主板上的 TOML 文件内容载入当前编辑框。', 'ok');
      await refreshConnectionStatus().catch(() => {});
    } catch (err) {
      lastActionText.textContent = `最近操作: SSH 读取失败 @ ${new Date().toLocaleTimeString()}`;
      setStatus(`SSH 读取失败:\n${err.message}`, 'error');
    }
  }

  async function fetchBoardFileViaSsh() {
    const result = await fetchJson('/api/config/ssh_pull', { cache: 'no-store' });
    lastBoardFileTomlText = result.toml_text || '';
    if (result.ssh_target) {
      sshTargetText.textContent =
        `SSH 目标: ${result.ssh_target.user}@${result.ssh_target.host}:${result.ssh_target.target_path} (port ${result.ssh_target.port})`;
    }
    return result.toml_text || '';
  }

  async function compareBoardFileAgainstEditor() {
    await refreshConnectionStatus().catch(() => {});
    lastActionText.textContent = '最近操作: 正在比较主板文件与当前编辑框...';
    setCompareStatus('正在通过 SSH 拉取主板文件并比较...', 'warn');
    try {
      const boardText = await fetchBoardFileViaSsh();
      const summary = summarizeDiff(boardText, configEditor.value || '');
      lastActionText.textContent = `最近操作: 比较完成 @ ${new Date().toLocaleTimeString()}`;
      setCompareStatus(summary.text, summary.same ? 'ok' : 'warn');
      await refreshConnectionStatus().catch(() => {});
    } catch (err) {
      lastActionText.textContent = `最近操作: 比较失败 @ ${new Date().toLocaleTimeString()}`;
      setCompareStatus(`比较失败:\n${err.message}`, 'error');
    }
  }

  async function backupBoardFileToLocal() {
    await refreshConnectionStatus().catch(() => {});
    lastActionText.textContent = '最近操作: 正在备份主板文件到本地...';
    setCompareStatus('正在通过 SSH 拉取主板文件并生成本地备份...', 'warn');
    try {
      const boardText = await fetchBoardFileViaSsh();
      const presetId = (connectionStore.active_preset || 'board').replace(/[^a-zA-Z0-9._-]+/g, '_');
      const filename = `smartcar_config_${presetId}_${formatTimestampForFilename()}.toml`;
      downloadTextFile(filename, boardText);
      lastActionText.textContent = `最近操作: 备份成功 @ ${new Date().toLocaleTimeString()}`;
      setCompareStatus(`主板文件已备份到本地。\n文件名: ${filename}`, 'ok');
      await refreshConnectionStatus().catch(() => {});
    } catch (err) {
      lastActionText.textContent = `最近操作: 备份失败 @ ${new Date().toLocaleTimeString()}`;
      setCompareStatus(`备份失败:\n${err.message}`, 'error');
    }
  }

  function downloadCurrentText() {
    downloadTextFile('smartcar_config.toml', configEditor.value || '');
  }

  document.getElementById('loadConfigBtn').addEventListener('click', loadCurrentConfig);
  document.getElementById('applyConfigBtn').addEventListener('click', applyCurrentConfig);
  presetSelect.addEventListener('change', () => {
    const selectedId = presetSelect.value;
    connectionStore.active_preset = selectedId;
    const current = activePreset();
    if (current) {
      fillConnectionSettingsInputs(current);
      setStatus('已切换到本地预设，修改后点“保存连接设置”才会真正写入电脑本地并刷新连接目标。', 'warn');
    }
  });
  document.getElementById('newPresetBtn').addEventListener('click', () => {
    createNewPresetFromCurrent();
    setStatus('已基于当前配置新增一个连接预设，修改后点“保存连接设置”即可持久化。', 'ok');
  });
  document.getElementById('deletePresetBtn').addEventListener('click', () => {
    try {
      deleteCurrentPreset();
      setStatus('当前预设已从本地列表移除，点“保存连接设置”后才会真正写入电脑本地。', 'warn');
    } catch (err) {
      setStatus(`删除预设失败:\n${err.message}`, 'error');
    }
  });
  document.getElementById('saveConnectionSettingsBtn').addEventListener('click', () => {
    saveConnectionSettings().then((result) => {
      const runtimeOnline = !!(result && result.runtime_http && result.runtime_http.online);
      const sshOnline = !!(result && result.ssh_transport && result.ssh_transport.online);
      setStatus(
        `连接设置已保存到电脑本地。\n运行态 HTTP: ${runtimeOnline ? '在线' : '离线'}\n离线 SSH: ${sshOnline ? '在线' : '离线'}`,
        (runtimeOnline || sshOnline) ? 'ok' : 'warn'
      );
    }).catch((err) => {
      setStatus(`保存连接设置失败:\n${err.message}`, 'error');
    });
  });
  document.getElementById('reloadConnectionSettingsBtn').addEventListener('click', () => {
    loadConnectionSettings().then(() => {
      return refreshConnectionStatus();
    }).then(() => {
      setStatus('已从电脑本地重新读取连接设置并刷新状态。', 'ok');
    }).catch((err) => {
      setStatus(`重新读取连接设置失败:\n${err.message}`, 'error');
    });
  });
  document.getElementById('syncPresetToEditorBtn').addEventListener('click', () => {
    try {
      syncPresetIpsIntoEditor();
      setStatus('已把当前预设里的电脑接收端 IP 同步到 TOML 编辑框。', 'ok');
    } catch (err) {
      setStatus(`同步预设 IP 到 TOML 失败:\n${err.message}`, 'error');
    }
  });
  document.getElementById('refreshStatusBtn').addEventListener('click', () => {
    refreshConnectionStatus().then((result) => {
      const runtimeOnline = !!(result && result.runtime_http && result.runtime_http.online);
      const sshOnline = !!(result && result.ssh_transport && result.ssh_transport.online);
      setStatus(
        `连接状态已刷新。\n运行态 HTTP: ${runtimeOnline ? '在线' : '离线'}\n离线 SSH: ${sshOnline ? '在线' : '离线'}`,
        (runtimeOnline || sshOnline) ? 'ok' : 'warn'
      );
    }).catch((err) => {
      setStatus(`刷新连接状态失败:\n${err.message}`, 'error');
    });
  });
  document.getElementById('sshPullBtn').addEventListener('click', sshPullCurrentConfig);
  document.getElementById('sshPushBtn').addEventListener('click', sshPushCurrentConfig);
  document.getElementById('compareBoardBtn').addEventListener('click', compareBoardFileAgainstEditor);
  document.getElementById('backupBoardBtn').addEventListener('click', backupBoardFileToLocal);
  document.getElementById('downloadConfigBtn').addEventListener('click', downloadCurrentText);
  currentBaseUrlText.textContent = `网页代理: ${window.location.origin}`;
  upstreamBaseUrlText.textContent = '主板配置服务: --';
  sshTargetText.textContent = 'SSH 目标: --';
  setBadge(runtimeHttpBadge, '检测中', 'warn');
  setBadge(sshBadge, '检测中', 'warn');
  setRestartKeys([]);
  setCompareStatus('等待比较...', '');
  loadConnectionSettings()
    .then(() => refreshConnectionStatus())
    .then((result) => {
      if (result && result.runtime_http && result.runtime_http.online) {
        return loadCurrentConfig();
      }
      setStatus('主板运行态接口离线。你仍可编辑 TOML，并使用“离线 SSH 写入主板”只覆盖配置文件。', 'warn');
      return null;
    })
    .catch((err) => {
      setStatus(`初始化连接状态失败:\n${err.message}`, 'error');
    });
}());
