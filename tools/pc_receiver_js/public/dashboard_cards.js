// dashboard_cards.js — L1 核心卡片渲染

var CARD_DEFS = {
  drive: [
    { label: '主状态',      fmt: 'label' },
    { label: '子状态',      fmt: 'label' },
    { label: '左轮转速',    fmt: 'int' },
    { label: '右轮转速',    fmt: 'int' },
    { label: '基础速度',    fmt: 'float1' },
    { label: '帧率',        fmt: 'int' }
  ],
  vision: [
    { label: '主状态',      fmt: 'label' },
    { label: '子状态',      fmt: 'label' },
    { label: '绕行状态',    fmt: 'label' },
    { label: '推理置信度',  fmt: 'pct' },
    { label: '中线数量',    fmt: 'int' },
    { label: '帧率',        fmt: 'int' }
  ],
  control: [
    { label: '主状态',      fmt: 'label' },
    { label: '子状态',      fmt: 'label' },
    { label: '基础速度',    fmt: 'float1' },
    { label: '差速',        fmt: 'float1' },
    { label: '目标角速度',  fmt: 'float1' },
    { label: '陀螺仪',      fmt: 'float1' }
  ]
};

function cardValueForStatus(status, preset, index) {
  var defs = CARD_DEFS[preset] || CARD_DEFS['drive'];
  var def = defs[index];
  if (!def) return '--';

  switch (preset) {
    case 'drive':
      switch (index) {
        case 0: return receiverCore.formatRouteMainState(status.route_main_state);
        case 1: return receiverCore.formatRouteSubState(status.route_sub_state);
        case 2: return fmtNum(status.pid_left_motor_speed_rpm);
        case 3: return fmtNum(status.pid_right_motor_speed_rpm);
        case 4: return fmtFloat1(status.pid_common_applied_base_speed);
        case 5: return fmtNum(status._fps);
      }
      break;
    case 'vision':
      switch (index) {
        case 0: return receiverCore.formatRouteMainState(status.route_main_state);
        case 1: return receiverCore.formatRouteSubState(status.route_sub_state);
        case 2: return status.detour_main_state || '--';
        case 3: return fmtPct(status.infer_max_prob);
        case 4: return fmtNum(status.straight_selected_centerline_count);
        case 5: return fmtNum(status._fps);
      }
      break;
    case 'control':
      switch (index) {
        case 0: return receiverCore.formatRouteMainState(status.route_main_state);
        case 1: return receiverCore.formatRouteSubState(status.route_sub_state);
        case 2: return fmtFloat1(status.pid_common_applied_base_speed);
        case 3: return fmtFloat1(diffSpeed(status));
        case 4: return fmtFloat1(status.pid_common_target_yaw_rate_abs_filtered_dps);
        case 5: return fmtFloat1(status.gyro_z_dps);
      }
      break;
  }
  return '--';
}

function diffSpeed(s) {
  var l = Number(s && s.pid_left_motor_speed_rpm);
  var r = Number(s && s.pid_right_motor_speed_rpm);
  if (isNaN(l) || isNaN(r)) return null;
  return l - r;
}

function fmtNum(v) {
  if (v === null || v === undefined) return '--';
  var n = Number(v);
  return isNaN(n) ? '--' : String(Math.round(n));
}

function fmtFloat1(v) {
  if (v === null || v === undefined) return '--';
  var n = Number(v);
  return isNaN(n) ? '--' : n.toFixed(1);
}

function fmtPct(v) {
  if (v === null || v === undefined) return '--';
  var n = Number(v);
  return isNaN(n) ? '--' : (n * 100).toFixed(1) + '%';
}

function renderCards(status, preset) {
  var defs = CARD_DEFS[preset] || CARD_DEFS['drive'];
  for (var i = 0; i < 6; i++) {
    var card = document.getElementById('card' + i);
    if (!card) continue;
    var label = card.querySelector('.l1-label');
    var value = card.querySelector('.l1-value');
    if (label) label.textContent = defs[i] ? defs[i].label : '--';
    if (value) {
      var val = status ? cardValueForStatus(status, preset, i) : '--';
      value.textContent = val;
    }
  }
}
