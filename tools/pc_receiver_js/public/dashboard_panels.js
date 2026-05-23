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
  var rows = [];

  rows.push(['主状态', receiverCore.formatRouteMainState(status.route_main_state)]);
  rows.push(['子状态', receiverCore.formatRouteSubState(status.route_sub_state)]);
  rows.push(['首选来源', receiverCore.formatRoutePreferredSource(status.route_preferred_source)]);
  rows.push(['编码器计数', status.route_encoder_since_enter]);

  rows.push(['直道中线数', status.straight_selected_centerline_count]);
  rows.push(['直道 lastIndex', status.straight_required_last_index]);
  rows.push(['直道误差和', receiverCore.formatValue(status.straight_abs_error_sum)]);
  rows.push(['直道误差最大', receiverCore.formatValue(status.straight_abs_error_sum_max)]);
  rows.push(['直道 ready', yn(status.straight_state_ready_now)]);

  rows.push(['十字左角行数', status.cross_left_corner_post_frame_wall_rows]);
  rows.push(['十字右角行数', status.cross_right_corner_post_frame_wall_rows]);
  rows.push(['十字 gap_x', status.cross_start_boundary_gap_x]);
  rows.push(['十字 entry', yn(status.cross_state_entry_ready_now)]);
  rows.push(['十字 stage2', yn(status.cross_state_stage2_ready_now)]);
  rows.push(['十字 stage3', yn(status.cross_state_stage3_ready_now)]);
  rows.push(['十字 exit', yn(status.cross_state_exit_ready_now)]);

  rows.push(['下一状态', status.route_next_state_label || '--']);

  rows.push(['直道计数', status.route_cross_loss_count]);
  rows.push(['左丢线', status.route_left_loss_count]);
  rows.push(['左得线', status.route_left_gain_count]);
  rows.push(['右丢线', status.route_right_loss_count]);
  rows.push(['右得线', status.route_right_gain_count]);
  rows.push(['斑马线', status.zebra_cross_count]);

  return buildTable(rows);
}

// ===== 绕行状态机详情 =====
function renderDetourPanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';
  var rows = [];

  rows.push(['绕行主状态', status.detour_main_state || '--']);
  rows.push(['绕行子状态', status.detour_sub_state || '--']);

  if (status.infer_probs && Array.isArray(status.infer_probs)) {
    for (var i = 0; i < status.infer_probs.length; i++) {
      rows.push(['分类' + i, (status.infer_probs[i] * 100).toFixed(1) + '%']);
    }
  }

  rows.push(['左帧墙行数', status.left_start_frame_wall_rows]);
  rows.push(['右帧墙行数', status.right_start_frame_wall_rows]);
  rows.push(['左帧墙有', yn(status.src_left_trace_has_frame_wall)]);
  rows.push(['右帧墙有', yn(status.src_right_trace_has_frame_wall)]);

  return buildTable(rows);
}

// ===== PID 参数详情 =====
function renderPidPanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';

  var html = '<div class="pid-grid">';

  html += '<div><div class="pid-col-title">公共 PID</div>';
  html += buildTable([
    ['Kp', status.pid_common_kp], ['Ki', status.pid_common_ki], ['Kd', status.pid_common_kd],
    ['目标角速度(dps)', receiverCore.formatValue(status.pid_common_target_yaw_rate_abs_filtered_dps)],
    ['yaw rate ref', receiverCore.formatValue(status.pid_common_yaw_rate_ref_dps)]
  ]);
  html += '</div>';

  html += '<div><div class="pid-col-title">左轮 PID</div>';
  html += buildTable([
    ['Kp', status.pid_left_kp], ['Ki', status.pid_left_ki], ['Kd', status.pid_left_kd],
    ['转速(rpm)', status.pid_left_motor_speed_rpm]
  ]);
  html += '</div>';

  html += '<div><div class="pid-col-title">右轮 PID</div>';
  html += buildTable([
    ['Kp', status.pid_right_kp], ['Ki', status.pid_right_ki], ['Kd', status.pid_right_kd],
    ['转速(rpm)', status.pid_right_motor_speed_rpm]
  ]);
  html += '</div>';

  html += '</div>';

  html += '<div style="margin-top:8px;"><div class="pid-col-title">减速方案</div>';
  html += buildTable([
    ['exp_lambda', status.pid_common_speed_scheme_rear_exp_lambda],
    ['split_ratio', status.pid_common_speed_scheme_split_ratio],
    ['error_scale_raw', status.pid_common_speed_scheme_error_scale_raw],
    ['realtime_speed', status.pid_common_speed_scheme_realtime_speed],
    ['winner_branch', status.pid_common_speed_scheme_winner_branch],
    ['final_scale', status.pid_common_speed_scheme_final_speed_scale]
  ]);
  html += '</div>';

  return html;
}

// ===== 陀螺仪数据 =====
function renderGyroPanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';
  return buildTable([
    ['角速度 Z(dps)', status.gyro_z_dps],
    ['角速度 X', status.gyro_x_dps],
    ['角速度 Y', status.gyro_y_dps],
    ['加速度 X', status.accel_x],
    ['加速度 Y', status.accel_y],
    ['加速度 Z', status.accel_z]
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
  return buildTable([
    ['速率(Mbps)', status._transport_mbps],
    ['速率(KiB/s)', status._transport_kibs],
    ['gray fps', status._gray_fps],
    ['rgb fps', status._rgb_fps],
    ['binary fps', status._binary_fps],
    ['传输模式', status.udp_web_mode],
    ['Max FPS', status.udp_web_max_fps],
    ['CPU %', status._cpu_pct],
    ['MEM %', status._mem_pct],
    ['同步状态', status._sync_note]
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
  drive:    [],
  vision:   ['route', 'detour', 'dir'],
  control:  ['pid', 'gyro']
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
