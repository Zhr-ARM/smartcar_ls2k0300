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

function detour(s) { return (s && s.detour) || {}; }
function wheels(s) { return (s && s.wheels) || {}; }
function wheel(s, side) { return (wheels(s)[side]) || {}; }
function spd(s) { return (s && s.speed) || {}; }
function imu(s) { return (s && s.imu) || {}; }
function ctlPid(s) { return (s && s.pid) || {}; }
function posPid(s) { return ctlPid(s).pos || {}; }
function yawPid(s) { return ctlPid(s).yaw || {}; }
function elems(s) { return (s && s.elements) || {}; }
function cl(s) { return (s && s.centerline) || {}; }

function cardValueForStatus(status, preset, index) {
  var defs = CARD_DEFS[preset] || CARD_DEFS['drive'];
  var def = defs[index];
  if (!def) return '--';

  switch (preset) {
    case 'drive':
      switch (index) {
        case 0: return receiverCore.formatRouteMainState(detour(status).route_main);
        case 1: return receiverCore.formatRouteSubState(detour(status).route_sub);
        case 2: return fmtNum(wheel(status, 'left').current);
        case 3: return fmtNum(wheel(status, 'right').current);
        case 4: return fmtFloat1(spd(status).adjusted);
        case 5: return fmtNum(status._fps);
      }
      break;
    case 'vision':
      switch (index) {
        case 0: return receiverCore.formatRouteMainState(detour(status).route_main);
        case 1: return receiverCore.formatRouteSubState(detour(status).route_sub);
        case 2: return receiverCore.formatRouteMainState(detour(status).route_main);
        case 3: return fmtPct(elems(status).ncnn_score);
        case 4: return fmtNum(cl(status).selected_count);
        case 5: return fmtNum(status._fps);
      }
      break;
    case 'control':
      switch (index) {
        case 0: return receiverCore.formatRouteMainState(detour(status).route_main);
        case 1: return receiverCore.formatRouteSubState(detour(status).route_sub);
        case 2: return fmtFloat1(spd(status).adjusted);
        case 3: return fmtFloat1(diffSpeed(status));
        case 4: return fmtFloat1(Math.abs(yawPid(status).ref));
        case 5: return fmtFloat1(imu(status).gyro_z);
      }
      break;
  }
  return '--';
}

function diffSpeed(s) {
  var l = Number(wheel(s, 'left').target);
  var r = Number(wheel(s, 'right').target);
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
