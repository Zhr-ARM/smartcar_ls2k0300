// dashboard_params.js — 快捷调参侧边栏

var PARAM_DEFS = [
  { key: 'pid_common_kp',               label: '公共 Kp',        min: 0, max: 10, step: 0.01 },
  { key: 'pid_common_ki',               label: '公共 Ki',        min: 0, max: 5,  step: 0.01 },
  { key: 'pid_common_kd',               label: '公共 Kd',        min: 0, max: 5,  step: 0.01 },
  { key: 'pid_left_kp',                 label: '左轮 Kp',        min: 0, max: 10, step: 0.01 },
  { key: 'pid_left_ki',                 label: '左轮 Ki',        min: 0, max: 5,  step: 0.01 },
  { key: 'pid_left_kd',                 label: '左轮 Kd',        min: 0, max: 5,  step: 0.01 },
  { key: 'pid_right_kp',                label: '右轮 Kp',        min: 0, max: 10, step: 0.01 },
  { key: 'pid_right_ki',                label: '右轮 Ki',        min: 0, max: 5,  step: 0.01 },
  { key: 'pid_right_kd',                label: '右轮 Kd',        min: 0, max: 5,  step: 0.01 },
  { key: 'pid_common_desired_base_speed',label: '目标基础速度',   min: 0, max: 500, step: 1 }
];

var currentParamValues = {};

function buildParamsPanel() {
  var container = document.getElementById('paramsContent');
  if (!container) return;
  var html = '';
  for (var i = 0; i < PARAM_DEFS.length; i++) {
    var def = PARAM_DEFS[i];
    html += '<label>' + def.label + '</label>';
    html += '<div class="param-row">';
    html += '<input type="range" id="param_' + def.key + '" min="' + def.min + '" max="' + def.max + '" step="' + def.step + '" value="0">';
    html += '<span class="param-val" id="param_' + def.key + '_val">--</span>';
    html += '</div>';
  }
  container.innerHTML = html;

  for (var i = 0; i < PARAM_DEFS.length; i++) {
    var def = PARAM_DEFS[i];
    var slider = document.getElementById('param_' + def.key);
    var display = document.getElementById('param_' + def.key + '_val');
    if (!slider || !display) continue;
    slider.addEventListener('input', function(d, s, disp) {
      return function() {
        disp.textContent = this.value;
        currentParamValues[d.key] = parseFloat(this.value);
      };
    }(def, slider, display));
  }
}

function syncParamsFromStatus(status) {
  if (!status) return;
  for (var i = 0; i < PARAM_DEFS.length; i++) {
    var def = PARAM_DEFS[i];
    var val = status[def.key];
    if (val !== null && val !== undefined) {
      var slider = document.getElementById('param_' + def.key);
      var display = document.getElementById('param_' + def.key + '_val');
      if (slider && display) {
        slider.value = String(val);
        display.textContent = String(val);
        currentParamValues[def.key] = parseFloat(val);
      }
    }
  }
}

document.getElementById('paramsBtn').addEventListener('click', function() {
  var panel = document.getElementById('paramsPanel');
  if (!panel) return;
  panel.classList.toggle('visible');
  if (panel.classList.contains('visible') && typeof syncParamsFromStatus === 'function') {
    syncParamsFromStatus(latestStatus);
  }
});

document.getElementById('paramsCloseBtn').addEventListener('click', function() {
  document.getElementById('paramsPanel').classList.remove('visible');
});

document.getElementById('paramsApplyBtn').addEventListener('click', function() {
  var resultEl = document.getElementById('paramsResult');
  if (!resultEl) return;
  resultEl.textContent = '正在应用...';
  resultEl.style.color = '';

  var xhr = new XMLHttpRequest();
  xhr.open('POST', '/api/config/apply', true);
  xhr.setRequestHeader('Content-Type', 'application/json');
  xhr.onload = function() {
    try {
      var j = JSON.parse(xhr.responseText);
      if (j.ok) {
        resultEl.textContent = '已热更新成功';
        resultEl.style.color = 'var(--ok)';
      } else if (j.offline_ok) {
        resultEl.textContent = '热更新不可用，已通过 SSH 离线写入';
        resultEl.style.color = 'var(--warn)';
      } else {
        resultEl.textContent = '应用失败: ' + (j.message || '未知错误');
        resultEl.style.color = 'var(--danger)';
      }
    } catch (e) {
      resultEl.textContent = '响应解析失败: ' + e.message;
      resultEl.style.color = 'var(--danger)';
    }
  };
  xhr.onerror = function() {
    resultEl.textContent = '请求失败';
    resultEl.style.color = 'var(--danger)';
  };
  xhr.send(JSON.stringify({ params: currentParamValues }));
});

buildParamsPanel();
