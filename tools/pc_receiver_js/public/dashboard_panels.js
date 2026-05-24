// dashboard_panels.js — L3 折叠面板渲染

function buildTable(rows) {
  var html = '<table class="panel-table">';
  for (var i = 0; i < rows.length; i++) {
    var key = rows[i][0];
    var val = rows[i][1];
    var display = (val === null || val === undefined || (typeof val === 'number' && isNaN(val))) ? '--' : String(val);
    html += '<tr><td class="key">' + key + '</td><td class="val">' + display + '</td></tr>';
  }
  html += '</table>';
  return html;
}

// ===== 元素状态机详情 =====
function renderRoutePanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';
  var d = status.detour || {};
  var cl = status.centerline || {};
  var rows = [];

  rows.push(['主状态', receiverCore.formatRouteMainState(d.route_main)]);
  rows.push(['子状态', receiverCore.formatRouteSubState(d.route_sub)]);
  rows.push(['首选来源', receiverCore.formatRoutePreferredSource(d.preferred_source)]);
  rows.push(['斑马线计数', d.zebra]);

  rows.push(['中线来源', cl.source]);
  rows.push(['中线点数', cl.selected_count]);
  rows.push(['跟踪有效', yn(cl.track_valid)]);
  rows.push(['跟踪点', Array.isArray(cl.track_point) ? cl.track_point.join(', ') : '--']);
  rows.push(['跟踪索引', cl.track_index]);

  rows.push(['line_error', status.line_error]);

  return buildTable(rows);
}

// ===== 绕行状态机详情 =====
function renderDetourPanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';
  var el = status.elements || {};
  var rows = [];

  rows.push(['红框检测', yn(el.red_found)]);
  if (Array.isArray(el.red)) {
    rows.push(['红框坐标', 'x=' + el.red[0] + ' y=' + el.red[1] + ' w=' + el.red[2] + ' h=' + el.red[3]]);
  }
  rows.push(['ncnn标签', el.ncnn_label || '--']);
  rows.push(['ncnn置信度', receiverCore.formatValue(el.ncnn_score)]);
  rows.push(['ncnn类别ID', el.ncnn_class_id]);

  return buildTable(rows);
}

// ===== PID 参数详情 =====
function renderPidPanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';

  var p = status.pid || {};
  var pos = p.pos || {};
  var yaw = p.yaw || {};
  var whl = status.wheels || {};
  var wl = whl.left || {};
  var wr = whl.right || {};
  var spd = status.speed || {};
  var imu = status.imu || {};

  var html = '<div class="pid-grid">';

  html += '<div><div class="pid-col-title">位置环 PID</div>';
  html += buildTable([
    ['Kp(dyn)', receiverCore.formatValue(pos.kp_dynamic)],
    ['error', receiverCore.formatValue(pos.error)],
    ['output', receiverCore.formatValue(pos.output)],
    ['integral', receiverCore.formatValue(pos.integral)]
  ]);
  html += '</div>';

  html += '<div><div class="pid-col-title">角速度环 PID</div>';
  html += buildTable([
    ['yaw ref', receiverCore.formatValue(yaw.ref)],
    ['yaw error', receiverCore.formatValue(yaw.error)],
    ['yaw output', receiverCore.formatValue(yaw.output)],
    ['yaw integral', receiverCore.formatValue(yaw.integral)]
  ]);
  html += '</div>';

  html += '<div><div class="pid-col-title">转向输出</div>';
  html += buildTable([
    ['steering', receiverCore.formatValue(p.steering)],
    ['gyro_z', receiverCore.formatValue(imu.gyro_z)],
    ['base speed', receiverCore.formatValue(spd.base)],
    ['adj speed', receiverCore.formatValue(spd.adjusted)]
  ]);
  html += '</div>';

  html += '</div>';

  html += '<div style="margin-top:8px;"><div class="pid-col-title">车轮详情</div>';
  html += buildTable([
    ['左目标', receiverCore.formatValue(wl.target)],
    ['左当前', receiverCore.formatValue(wl.current)],
    ['左误差', receiverCore.formatValue(wl.error)],
    ['左占空比', receiverCore.formatValue(wl.duty)],
    ['右目标', receiverCore.formatValue(wr.target)],
    ['右当前', receiverCore.formatValue(wr.current)],
    ['右误差', receiverCore.formatValue(wr.error)],
    ['右占空比', receiverCore.formatValue(wr.duty)]
  ]);
  html += '</div>';

  return html;
}

// ===== 陀螺仪数据 =====
function renderGyroPanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';
  var imu = status.imu || {};
  return buildTable([
    ['角速度 Z(dps)', receiverCore.formatValue(imu.gyro_z)],
    ['yaw ref', receiverCore.formatValue(imu.yaw_ref)],
    ['yaw error', receiverCore.formatValue(imu.yaw_error)],
    ['line_error', status.line_error]
  ]);
}

// ===== dir 数组 =====
function renderDirPanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';
  var html = '';
  if (Array.isArray(status.left_trace_dir)) {
    html += '<div style="font-size:12px;color:var(--text-secondary);margin-bottom:4px;">左 dir: ' + receiverCore.formatArrayInline(status.left_trace_dir) + '</div>';
  }
  if (Array.isArray(status.right_trace_dir)) {
    html += '<div style="font-size:12px;color:var(--text-secondary);">右 dir: ' + receiverCore.formatArrayInline(status.right_trace_dir) + '</div>';
  }
  if (!html) html = '<div class="pid-empty">等待 dir 数据...</div>';
  return html;
}

// ===== 传输状态详情 =====
function renderTransportPanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';
  var fps = status.fps || {};
  return buildTable([
    ['UDP速率(Mbps)', status._transport_mbps],
    ['UDP速率(KiB/s)', status._transport_kibs],
    ['传输帧率', status._fps],
    ['采集FPS', fps.capture],
    ['视觉FPS', fps.vision],
    ['发送FPS', fps.tx],
    ['车端时间(s)', status.ts ? (status.ts / 1000).toFixed(1) : '--']
  ]);
}

// ===== 面板注册表 =====
var PANEL_REGISTRY = {
  route:    { fn: renderRoutePanel,     bodyId: 'panelRouteBody' },
  detour:   { fn: renderDetourPanel,    bodyId: 'panelDetourBody' },
  pid:      { fn: renderPidPanel,       bodyId: 'panelPidBody' },
  gyro:     { fn: renderGyroPanel,      bodyId: 'panelGyroBody' },
  dir:      { fn: renderDirPanel,       bodyId: 'panelDirBody' },
  transport:{ fn: renderTransportPanel, bodyId: 'panelTransportBody' }
};

var PRESET_OPEN = {
  drive:    ['transport'],
  vision:   ['route', 'detour', 'transport'],
  control:  ['pid', 'gyro', 'transport']
};

function renderAllPanels(status) {
  var keys = Object.keys(PANEL_REGISTRY);
  for (var i = 0; i < keys.length; i++) {
    var cfg = PANEL_REGISTRY[keys[i]];
    var body = document.getElementById(cfg.bodyId);
    if (body) body.innerHTML = cfg.fn(status);
  }
}

function applyPresetPanels(preset) {
  var openSet = PRESET_OPEN[preset] || [];
  var keys = Object.keys(PANEL_REGISTRY);
  for (var i = 0; i < keys.length; i++) {
    var panelId = keys[i];
    var domId = 'panel' + panelId.charAt(0).toUpperCase() + panelId.slice(1);
    var el = document.getElementById(domId);
    if (!el) continue;
    if (openSet.indexOf(panelId) >= 0) {
      el.classList.add('open');
    } else {
      el.classList.remove('open');
    }
  }
}

function yn(v) { return v ? '✓' : '✗'; }
