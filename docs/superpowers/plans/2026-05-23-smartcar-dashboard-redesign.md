# Smart Car Dashboard 网页重设计实施计划

> **For agentic workers:** 此计划需按任务顺序执行。每步使用 checkbox (`- [ ]`) 跟踪进度。

**目标:** 将智能小车仪表盘从当前 474 行精简版重建为功能完整、信息层级清晰的单页仪表盘。

**架构:** 单 HTML 页面 + 5 个 JS 模块（卡片渲染、折叠面板、图像叠加、回放控制、快捷调参），复用 v1 的 `shared_receiver_core.js` 数据处理层和 `server.js` 后端数据管道。回放复用同一仪表盘 UI，仅切换数据源。

**技术栈:** 纯 HTML/CSS/JS，Canvas 2D，WebSocket，无框架无构建工具。

---

## 文件结构

| 操作 | 文件 | 职责 |
|------|------|------|
| 重写 | `tools/pc_receiver_js/public/index.html` | HTML 骨架 + 内联 CSS（~300行） |
| 新建 | `tools/pc_receiver_js/public/dashboard.js` | 主逻辑：WebSocket、预设切换、数据协调、录制状态机（~400行） |
| 新建 | `tools/pc_receiver_js/public/dashboard_cards.js` | L1 大字卡片渲染，5-6 个卡片，三预设不同内容（~150行） |
| 新建 | `tools/pc_receiver_js/public/dashboard_panels.js` | L3 6 个折叠面板渲染（~350行） |
| 新建 | `tools/pc_receiver_js/public/dashboard_overlay.js` | Canvas 叠加层：边界/中线/锚点/dir图（~250行） |
| 新建 | `tools/pc_receiver_js/public/dashboard_playback.js` | 回放数据加载、播放控制、时间轴（~200行） |
| 新建 | `tools/pc_receiver_js/public/dashboard_params.js` | 快捷调参侧边栏：PID 滑块 + 智能应用按钮（~120行） |
| 修改 | `tools/pc_receiver_js/public/config.html` | 合并热更新+离线按钮为一个智能按钮 |
| 修改 | `tools/pc_receiver_js/public/config_app.js` | 适配智能按钮逻辑 |
| 保留 | `tools/pc_receiver_js/public/shared_receiver_core.js` | 不改动 |
| 保留 | `tools/pc_receiver_js/server.js` | 不改动 |

---

### Task 1: HTML + CSS 骨架

**文件:**
- 重写: `tools/pc_receiver_js/public/index.html`

- [ ] **Step 1: 写入 HTML 结构和内联 CSS**

完整的 index.html，包含：
- 顶部 hero bar：三预设 Tab（驾驶/视觉/车控）+ 帧率pill + 连接状态pill + 参数配置按钮 + 录制按钮
- 左侧 30%：灰度图 canvas + IPM 透视图 canvas + 叠加开关 checkbox 行
- 右侧 70%：L1 卡片行（5 个卡片 slot）+ 折叠面板容器（6 个 panel）
- 回放控制条（默认隐藏）
- 快捷调参侧边栏（默认隐藏）
- 底部加载所有 JS 文件（`shared_receiver_core.js` 先加载）

```html
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>Smart Car Dashboard</title>
<style>
/* ===== 全局 ===== */
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  font-family: "Noto Sans SC", "Microsoft YaHei", sans-serif;
  background: linear-gradient(180deg, #0f172a 0%, #0b1220 100%);
  color: #e2e8f0;
  height: 100vh;
  overflow: hidden;
}
.dashboard {
  height: 100vh;
  display: flex;
  flex-direction: column;
}

/* ===== Hero Bar ===== */
.hero {
  display: flex; align-items: center; justify-content: space-between;
  padding: 6px 14px; flex-shrink: 0;
  background: rgba(17,24,39,0.96);
  border-bottom: 1px solid rgba(71,85,105,0.4);
}
.hero-left { display: flex; align-items: center; gap: 4px; }
.preset-btn {
  padding: 5px 16px; border-radius: 6px; font-size: 13px;
  border: 1px solid rgba(71,85,105,0.6);
  background: rgba(30,41,59,0.8); color: #94a3b8; cursor: pointer;
}
.preset-btn.active {
  background: rgba(59,130,246,0.2); color: #60a5fa;
  border-color: rgba(59,130,246,0.5);
}
.hero-right { display: flex; align-items: center; gap: 10px; }
.status-pill {
  padding: 3px 10px; border-radius: 10px; font-size: 12px; font-weight: 600;
  border: 1px solid;
}
.status-pill.ok { background: rgba(16,185,129,0.15); color: #6ee7b7; border-color: rgba(16,185,129,0.3); }
.status-pill.warn { background: rgba(251,191,36,0.15); color: #fcd34d; border-color: rgba(251,191,36,0.3); }
.btn-hero {
  padding: 4px 12px; border-radius: 5px; font-size: 12px;
  border: 1px solid rgba(71,85,105,0.6);
  background: rgba(30,41,59,0.8); color: #cbd5e1; cursor: pointer;
}
.btn-hero.recording {
  background: rgba(220,38,38,0.2); color: #fca5a5;
  border-color: rgba(220,38,38,0.5);
}

/* ===== Main Layout ===== */
.main {
  display: flex; flex: 1; min-height: 0; gap: 8px; padding: 8px;
}
.left-col { width: 30%; display: flex; flex-direction: column; gap: 8px; min-width: 0; }
.right-col { flex: 1; display: flex; flex-direction: column; gap: 6px; min-width: 0; }

/* ===== 图像面板 ===== */
.img-card {
  background: rgba(17,24,39,0.96); border: 1px solid rgba(71,85,105,0.5);
  border-radius: 8px; padding: 6px; display: flex; flex-direction: column;
}
.img-card .img-title {
  font-size: 11px; color: #94a3b8; margin-bottom: 4px;
  display: flex; justify-content: space-between;
}
.img-card canvas { width: 100%; border-radius: 4px; image-rendering: pixelated; background: #000; }

/* ===== 叠加开关 ===== */
.overlay-toggles {
  display: flex; flex-wrap: wrap; gap: 6px;
  padding: 6px 8px;
  background: rgba(17,24,39,0.96); border: 1px solid rgba(71,85,105,0.5);
  border-radius: 8px;
}
.overlay-toggles label {
  font-size: 11px; color: #94a3b8; display: flex; align-items: center; gap: 3px; cursor: pointer;
}

/* ===== L1 卡片行 ===== */
.cards-row {
  display: grid; grid-template-columns: repeat(6, 1fr); gap: 6px;
  flex-shrink: 0;
}
.l1-card {
  background: rgba(17,24,39,0.96); border: 1px solid rgba(71,85,105,0.5);
  border-radius: 8px; padding: 10px 12px; text-align: center;
}
.l1-card .l1-label { font-size: 10px; color: #64748b; text-transform: uppercase; letter-spacing: 0.05em; }
.l1-card .l1-value { font-size: 28px; font-weight: 700; color: #f1f5f9; margin-top: 2px; }
.l1-card .l1-sub { font-size: 11px; color: #94a3b8; margin-top: 1px; }

/* ===== 折叠面板 ===== */
.panels-area { flex: 1; min-height: 0; overflow-y: auto; display: flex; flex-direction: column; gap: 4px; }
.panel {
  background: rgba(17,24,39,0.96); border: 1px solid rgba(71,85,105,0.4);
  border-radius: 8px; flex-shrink: 0;
}
.panel-header {
  display: flex; align-items: center; justify-content: space-between;
  padding: 8px 12px; cursor: pointer; user-select: none;
}
.panel-header:hover { background: rgba(59,130,246,0.08); }
.panel-header .panel-title { font-size: 13px; font-weight: 600; color: #cbd5e1; }
.panel-header .panel-arrow { font-size: 11px; color: #64748b; transition: transform 0.2s; }
.panel.open .panel-arrow { transform: rotate(90deg); }
.panel-body { display: none; padding: 0 12px 10px; max-height: 350px; overflow-y: auto; }
.panel.open .panel-body { display: block; }

/* 面板内数据样式 */
.panel-table { width: 100%; border-collapse: collapse; font-size: 12px; }
.panel-table td { padding: 3px 6px; border-bottom: 1px solid rgba(71,85,105,0.2); }
.panel-table .key { color: #94a3b8; white-space: nowrap; }
.panel-table .val { color: #e2e8f0; font-family: "JetBrains Mono", monospace; }

/* ===== 回放控制条 ===== */
.playback-bar {
  display: none; align-items: center; gap: 8px;
  padding: 6px 12px; flex-shrink: 0;
  background: rgba(30,41,59,0.96); border-top: 1px solid rgba(251,191,36,0.3);
}
.playback-bar.visible { display: flex; }
.playback-bar button {
  padding: 4px 10px; border-radius: 4px; font-size: 12px;
  border: 1px solid rgba(71,85,105,0.6);
  background: rgba(30,41,59,0.8); color: #cbd5e1; cursor: pointer;
}

/* ===== 调参侧边栏 ===== */
.params-panel {
  display: none; flex-direction: column; gap: 10px;
  position: fixed; right: 0; top: 0; bottom: 0; width: 320px; z-index: 100;
  background: rgba(15,23,42,0.98); border-left: 1px solid rgba(59,130,246,0.3);
  padding: 16px; overflow-y: auto;
}
.params-panel.visible { display: flex; }
.params-panel label { font-size: 12px; color: #94a3b8; }
.params-panel input { width: 100%; margin-bottom: 8px; }
.params-panel input[type="range"] { accent-color: #3b82f6; }
.params-panel .param-val { font-size: 11px; color: #60a5fa; float: right; }

/* ===== 滚动条 ===== */
.panels-area::-webkit-scrollbar, .panel-body::-webkit-scrollbar { width: 4px; }
.panels-area::-webkit-scrollbar-thumb, .panel-body::-webkit-scrollbar-thumb {
  background: rgba(71,85,105,0.5); border-radius: 2px;
}
</style>
</head>
<body>
<div class="dashboard">
  <!-- Hero Bar -->
  <div class="hero">
    <div class="hero-left">
      <button class="preset-btn active" data-preset="drive">驾驶</button>
      <button class="preset-btn" data-preset="vision">视觉调试</button>
      <button class="preset-btn" data-preset="control">车控调试</button>
    </div>
    <div class="hero-right">
      <span class="status-pill ok" id="connPill">连接</span>
      <span class="status-pill" id="fpsPill">-- fps</span>
      <button class="btn-hero" id="paramsBtn">参数</button>
      <button class="btn-hero" id="recordBtn">录制</button>
      <span style="font-size:11px;color:#64748b;" id="recordStatus"></span>
    </div>
  </div>

  <!-- Main -->
  <div class="main">
    <!-- Left Col -->
    <div class="left-col">
      <div class="img-card" style="flex:1;">
        <div class="img-title"><span>灰度图</span><span id="grayPixel">-</span></div>
        <canvas id="grayCanvas" width="320" height="240"></canvas>
      </div>
      <div class="img-card">
        <div class="img-title"><span>IPM 透视图</span><span id="ipmPixel">-</span></div>
        <canvas id="ipmCanvas" width="400" height="120"></canvas>
      </div>
      <div class="overlay-toggles" id="overlayToggles">
        <label><input type="checkbox" id="togBoundary" checked>边界</label>
        <label><input type="checkbox" id="togCenterline">中线</label>
        <label><input type="checkbox" id="togAnchor" checked>锚点</label>
        <label><input type="checkbox" id="togDir">dir图</label>
        <label><input type="checkbox" id="togSamplePoints">采样点</label>
      </div>
    </div>

    <!-- Right Col -->
    <div class="right-col">
      <!-- L1 Cards -->
      <div class="cards-row" id="cardsRow">
        <div class="l1-card" id="card0"><div class="l1-label">--</div><div class="l1-value">--</div></div>
        <div class="l1-card" id="card1"><div class="l1-label">--</div><div class="l1-value">--</div></div>
        <div class="l1-card" id="card2"><div class="l1-label">--</div><div class="l1-value">--</div></div>
        <div class="l1-card" id="card3"><div class="l1-label">--</div><div class="l1-value">--</div></div>
        <div class="l1-card" id="card4"><div class="l1-label">--</div><div class="l1-value">--</div></div>
        <div class="l1-card" id="card5"><div class="l1-label">--</div><div class="l1-value">--</div></div>
      </div>

      <!-- Panels Area -->
      <div class="panels-area" id="panelsArea">
        <!-- 6 collapsible panels, initially all collapsed -->
        <div class="panel" id="panelRoute">
          <div class="panel-header" data-panel="route"><span class="panel-title">元素状态机详情</span><span class="panel-arrow">▶</span></div>
          <div class="panel-body" id="panelRouteBody"></div>
        </div>
        <div class="panel" id="panelDetour">
          <div class="panel-header" data-panel="detour"><span class="panel-title">绕行状态机详情</span><span class="panel-arrow">▶</span></div>
          <div class="panel-body" id="panelDetourBody"></div>
        </div>
        <div class="panel" id="panelPid">
          <div class="panel-header" data-panel="pid"><span class="panel-title">PID 参数详情</span><span class="panel-arrow">▶</span></div>
          <div class="panel-body" id="panelPidBody"></div>
        </div>
        <div class="panel" id="panelGyro">
          <div class="panel-header" data-panel="gyro"><span class="panel-title">陀螺仪数据</span><span class="panel-arrow">▶</span></div>
          <div class="panel-body" id="panelGyroBody"></div>
        </div>
        <div class="panel" id="panelDir">
          <div class="panel-header" data-panel="dir"><span class="panel-title">dir 数组</span><span class="panel-arrow">▶</span></div>
          <div class="panel-body" id="panelDirBody"></div>
        </div>
        <div class="panel" id="panelTransport">
          <div class="panel-header" data-panel="transport"><span class="panel-title">传输状态详情</span><span class="panel-arrow">▶</span></div>
          <div class="panel-body" id="panelTransportBody"></div>
        </div>
      </div>
    </div>
  </div>

  <!-- Playback Bar -->
  <div class="playback-bar" id="playbackBar">
    <button id="pbPlay">▶/⏸</button>
    <button id="pbBack">⏪ -1s</button>
    <button id="pbFwd">⏩ +1s</button>
    <span style="font-size:12px;color:#94a3b8;" id="pbTime">00:00 / 00:00</span>
    <input type="range" id="pbSeek" style="flex:1;" min="0" max="100" value="0">
    <button id="pbClose">关闭回放</button>
  </div>

  <!-- Quick Params Sidebar -->
  <div class="params-panel" id="paramsPanel">
    <div style="display:flex;justify-content:space-between;align-items:center;">
      <h3 style="font-size:14px;">快捷调参</h3>
      <button id="paramsCloseBtn" style="background:none;border:none;color:#94a3b8;cursor:pointer;font-size:16px;">✕</button>
    </div>
    <div id="paramsContent"></div>
    <button id="paramsApplyBtn" class="btn-hero" style="width:100%;margin-top:8px;padding:8px;">应用 (优先热更新)</button>
    <div id="paramsResult" style="font-size:11px;color:#94a3b8;margin-top:4px;"></div>
  </div>
</div>

<script src="shared_receiver_core.js"></script>
<script src="dashboard_cards.js"></script>
<script src="dashboard_panels.js"></script>
<script src="dashboard_overlay.js"></script>
<script src="dashboard_playback.js"></script>
<script src="dashboard_params.js"></script>
<script src="dashboard.js"></script>
</body>
</html>
```

- [ ] **Step 2: 验证骨架**
  用浏览器打开 `tools/pc_receiver_js/public/index.html`，确认：
  - 布局比例为左 30% 右 70%
  - 无水平或垂直滚动条（在 1820x1200 窗口下）
  - 三预设按钮可点击切换 active 状态
  - 6 个折叠面板点击可展开/收起
  - 回放控制条和调参侧边栏默认隐藏

- [ ] **Step 3: Commit**

```bash
git add tools/pc_receiver_js/public/index.html
git commit -m "feat: add dashboard HTML skeleton with layout and collapsible panels"
```

---

### Task 2: L1 大字卡片渲染

**文件:**
- 新建: `tools/pc_receiver_js/public/dashboard_cards.js`

- [ ] **Step 1: 写入 dashboard_cards.js**

卡片定义 —— 三预设下各自展示 6 个卡片：

```javascript
// dashboard_cards.js — L1 核心卡片渲染

const CARD_DEFS = {
  drive: [
    { id: 'mainState', label: '主状态', key: 'route_main_state', fmt: 'stateName' },
    { id: 'subState', label: '子状态', key: 'route_sub_state', fmt: 'stateName' },
    { id: 'speedL', label: '左轮转速', key: 'pid_left_motor_speed_rpm', fmt: 'int' },
    { id: 'speedR', label: '右轮转速', key: 'pid_right_motor_speed_rpm', fmt: 'int' },
    { id: 'baseSpeed', label: '基础速度', key: 'pid_common_applied_base_speed', fmt: 'float1' },
    { id: 'fps', label: '帧率', key: '_fps', fmt: 'int' },
  ],
  vision: [
    { id: 'mainState', label: '主状态', key: 'route_main_state', fmt: 'stateName' },
    { id: 'subState', label: '子状态', key: 'route_sub_state', fmt: 'stateName' },
    { id: 'detourState', label: '绕行状态', key: 'detour_main_state', fmt: 'stateName' },
    { id: 'inferConf', label: '推理置信度', key: 'infer_max_prob', fmt: 'pct' },
    { id: 'clCount', label: '中线数量', key: 'straight_selected_centerline_count', fmt: 'int' },
    { id: 'fps', label: '帧率', key: '_fps', fmt: 'int' },
  ],
  control: [
    { id: 'mainState', label: '主状态', key: 'route_main_state', fmt: 'stateName' },
    { id: 'subState', label: '子状态', key: 'route_sub_state', fmt: 'stateName' },
    { id: 'baseSpeed', label: '基础速度', key: 'pid_common_applied_base_speed', fmt: 'float1' },
    { id: 'diffSpeed', label: '差速', key: '_diff_speed', fmt: 'float1' },
    { id: 'yawRate', label: '目标角速度', key: 'pid_common_target_yaw_rate_abs_filtered_dps', fmt: 'float1' },
    { id: 'gyro', label: '陀螺仪', key: '_gyro_z', fmt: 'float1' },
  ],
};

const STATE_LABELS = {}; // populated by shared_receiver_core.js functions

function formatCardValue(val, fmt) {
  if (val === null || val === undefined || (typeof val === 'number' && isNaN(val))) return '--';
  switch (fmt) {
    case 'int': return String(Math.round(val));
    case 'float1': return Number(val).toFixed(1);
    case 'pct': return Number(val).toFixed(1) + '%';
    case 'stateName': return String(val); // shared_receiver_core provides label functions
    default: return String(val);
  }
}

function renderCards(status, preset) {
  const defs = CARD_DEFS[preset] || CARD_DEFS['drive'];
  defs.forEach((def, i) => {
    const card = document.getElementById('card' + i);
    if (!card) return;
    let val = status[def.key];
    // computed keys (prefixed with _)
    if (def.key === '_fps') {
      val = (typeof status._fps === 'number') ? status._fps : '--';
    }
    if (def.key === '_diff_speed') {
      const l = Number(status.pid_left_motor_speed_rpm);
      const r = Number(status.pid_right_motor_speed_rpm);
      val = (Number.isFinite(l) && Number.isFinite(r)) ? (l - r) : null;
    }
    if (def.key === '_gyro_z') {
      val = status.gyro_z_dps;
    }
    const label = card.querySelector('.l1-label');
    const value = card.querySelector('.l1-value');
    if (label) label.textContent = def.label;
    if (value) value.textContent = formatCardValue(val, def.fmt);
  });
}
```

- [ ] **Step 2: 验证**
  在 `dashboard.js` 中模拟 status 对象调用 `renderCards(status, 'drive')`，确认 6 个卡片正确渲染。

- [ ] **Step 3: Commit**

```bash
git add tools/pc_receiver_js/public/dashboard_cards.js
git commit -m "feat: add L1 card renderer with three preset definitions"
```

---

### Task 3: L3 折叠面板渲染

**文件:**
- 新建: `tools/pc_receiver_js/public/dashboard_panels.js`

- [ ] **Step 1: 写入 dashboard_panels.js**

6 个面板的渲染函数，每个函数接收 status 对象，返回 HTML 字符串：

```javascript
// dashboard_panels.js — L3 折叠面板渲染

// ===== 元素状态机详情 =====
function renderRoutePanel(status) {
  const rows = [];
  if (!status) return '<div class="pid-empty">等待数据...</div>';

  const mainState = receiverCore.formatRouteMainState(status.route_main_state);
  const subState = receiverCore.formatRouteSubState(status.route_sub_state);
  const prefSrc = receiverCore.formatRoutePreferredSource(status.route_preferred_source);

  rows.push(['主状态', mainState], ['子状态', subState], ['首选来源', prefSrc]);
  rows.push(['编码器计数', status.route_encoder_since_enter]);

  // 直道焦点
  rows.push(['直道中线数', status.straight_selected_centerline_count]);
  rows.push(['直道 lastIndex', status.straight_required_last_index]);
  rows.push(['直道误差和', receiverCore.formatValue(status.straight_abs_error_sum)]);
  rows.push(['直道误差最大', receiverCore.formatValue(status.straight_abs_error_sum_max)]);
  rows.push(['直道 ready', status.straight_state_ready_now ? '✓' : '✗']);

  // 十字焦点
  rows.push(['十字左角行数', status.cross_left_corner_post_frame_wall_rows]);
  rows.push(['十字右角行数', status.cross_right_corner_post_frame_wall_rows]);
  rows.push(['十字 gap_x', status.cross_start_boundary_gap_x]);
  rows.push(['十字 entry ready', status.cross_state_entry_ready_now ? '✓' : '✗']);
  rows.push(['十字 stage2 ready', status.cross_state_stage2_ready_now ? '✓' : '✗']);
  rows.push(['十字 stage3 ready', status.cross_state_stage3_ready_now ? '✓' : '✗']);
  rows.push(['十字 exit ready', status.cross_state_exit_ready_now ? '✓' : '✗']);

  // 下一状态
  rows.push(['下一状态', status.route_next_state_label || '--']);

  // 状态计数器
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
  const rows = [];
  rows.push(['绕行主状态', status.detour_main_state || '--']);
  rows.push(['绕行子状态', status.detour_sub_state || '--']);

  // 推理概率
  if (status.infer_probs && Array.isArray(status.infer_probs)) {
    status.infer_probs.forEach((p, i) => {
      rows.push([`分类${i}`, (p * 100).toFixed(1) + '%']);
    });
  }

  // 帧墙数据
  rows.push(['左帧墙行数', status.left_start_frame_wall_rows]);
  rows.push(['右帧墙行数', status.right_start_frame_wall_rows]);
  rows.push(['左帧墙有', status.src_left_trace_has_frame_wall ? '✓' : '✗']);
  rows.push(['右帧墙有', status.src_right_trace_has_frame_wall ? '✓' : '✗']);

  return buildTable(rows);
}

// ===== PID 参数详情 =====
function renderPidPanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';

  let html = '<div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;">';

  // 公共 PID
  html += '<div><div style="font-weight:700;font-size:12px;color:#f8fafc;margin-bottom:6px;">公共 PID</div>';
  html += buildTable([
    ['Kp', status.pid_common_kp], ['Ki', status.pid_common_ki], ['Kd', status.pid_common_kd],
    ['目标角速度(dps)', receiverCore.formatValue(status.pid_common_target_yaw_rate_abs_filtered_dps)],
    ['yaw rate ref', receiverCore.formatValue(status.pid_common_yaw_rate_ref_dps)],
  ]);
  html += '</div>';

  // 左轮 PID
  html += '<div><div style="font-weight:700;font-size:12px;color:#f8fafc;margin-bottom:6px;">左轮 PID</div>';
  html += buildTable([
    ['Kp', status.pid_left_kp], ['Ki', status.pid_left_ki], ['Kd', status.pid_left_kd],
    ['转速(rpm)', status.pid_left_motor_speed_rpm],
  ]);
  html += '</div>';

  // 右轮 PID
  html += '<div><div style="font-weight:700;font-size:12px;color:#f8fafc;margin-bottom:6px;">右轮 PID</div>';
  html += buildTable([
    ['Kp', status.pid_right_kp], ['Ki', status.pid_right_ki], ['Kd', status.pid_right_kd],
    ['转速(rpm)', status.pid_right_motor_speed_rpm],
  ]);
  html += '</div>';

  html += '</div>';

  // 减速方案
  html += '<div style="margin-top:8px;font-weight:700;font-size:12px;color:#f8fafc;">减速方案</div>';
  html += buildTable([
    ['exp_lambda', status.pid_common_speed_scheme_rear_exp_lambda],
    ['split_ratio', status.pid_common_speed_scheme_split_ratio],
    ['error_scale_raw', status.pid_common_speed_scheme_error_scale_raw],
    ['realtime_speed', status.pid_common_speed_scheme_realtime_speed],
    ['winner_branch', status.pid_common_speed_scheme_winner_branch],
    ['final_scale', status.pid_common_speed_scheme_final_speed_scale],
  ]);

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
    ['加速度 Z', status.accel_z],
  ]);
}

// ===== dir 数组 =====
function renderDirPanel(status) {
  if (!status) return '<div class="pid-empty">等待数据...</div>';
  let html = '';
  if (Array.isArray(status.left_trace_dir)) {
    html += '<div style="font-size:12px;color:#94a3b8;margin-bottom:4px;">左 dir: ' + receiverCore.formatArrayInline(status.left_trace_dir) + '</div>';
  }
  if (Array.isArray(status.right_trace_dir)) {
    html += '<div style="font-size:12px;color:#94a3b8;">右 dir: ' + receiverCore.formatArrayInline(status.right_trace_dir) + '</div>';
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
    ['模式', status.udp_web_mode],
    ['Max FPS', status.udp_web_max_fps],
    ['CPU %', status._cpu_pct],
    ['MEM %', status._mem_pct],
    ['同步状态', status._sync_note],
  ]);
}

// ===== 工具函数 =====
function buildTable(rows) {
  let html = '<table class="panel-table">';
  for (const [key, val] of rows) {
    const display = (val === null || val === undefined || (typeof val === 'number' && isNaN(val))) ? '--' : String(val);
    html += `<tr><td class="key">${key}</td><td class="val">${display}</td></tr>`;
  }
  html += '</table>';
  return html;
}

// ===== 面板渲染总入口 =====
const PANEL_RENDERERS = {
  route:   { fn: renderRoutePanel,     bodyId: 'panelRouteBody' },
  detour:  { fn: renderDetourPanel,    bodyId: 'panelDetourBody' },
  pid:     { fn: renderPidPanel,       bodyId: 'panelPidBody' },
  gyro:    { fn: renderGyroPanel,      bodyId: 'panelGyroBody' },
  dir:     { fn: renderDirPanel,       bodyId: 'panelDirBody' },
  transport:{ fn: renderTransportPanel,bodyId: 'panelTransportBody' },
};

const PRESET_OPEN_PANELS = {
  drive:    [],
  vision:   ['route', 'detour', 'dir'],
  control:  ['pid', 'gyro'],
};

function renderAllPanels(status) {
  for (const panelId of Object.keys(PANEL_RENDERERS)) {
    const cfg = PANEL_RENDERERS[panelId];
    const body = document.getElementById(cfg.bodyId);
    if (body) body.innerHTML = cfg.fn(status);
  }
}

function applyPresetPanels(preset) {
  const openSet = new Set(PRESET_OPEN_PANELS[preset] || []);
  for (const panelId of Object.keys(PANEL_RENDERERS)) {
    const panel = document.getElementById('panel' + panelId.charAt(0).toUpperCase() + panelId.slice(1));
    // Map panelId to DOM id: route -> panelRoute, detour -> panelDetour, etc.
    const domId = 'panel' + panelId.charAt(0).toUpperCase() + panelId.slice(1);
    const el = document.getElementById(domId);
    if (!el) continue;
    if (openSet.has(panelId)) {
      el.classList.add('open');
    } else {
      el.classList.remove('open');
    }
  }
}
```

- [ ] **Step 2: 验证**
  用模拟 status 数据调用 `renderAllPanels()`，确认 6 个面板都能正确生成 HTML 内容，表格行对齐。

- [ ] **Step 3: Commit**

```bash
git add tools/pc_receiver_js/public/dashboard_panels.js
git commit -m "feat: add L3 collapsible panel renderers for all 6 data categories"
```

---

### Task 4: Canvas 图像叠加层

**文件:**
- 新建: `tools/pc_receiver_js/public/dashboard_overlay.js`

- [ ] **Step 1: 写入 dashboard_overlay.js**

```javascript
// dashboard_overlay.js — Canvas 图像叠加层

let showBoundary = true;
let showCenterline = false;
let showAnchor = true;
let showDir = false;
let showSamplePoints = true;

// Canvas contexts (set by dashboard.js)
let grayCtx, ipmCtx;

function initOverlayContexts(gCtx, iCtx) {
  grayCtx = gCtx;
  ipmCtx = iCtx;
}

function drawOverlays(status) {
  if (!status) return;
  drawGrayOverlays(status);
  drawIpmOverlays(status);
}

function drawGrayOverlays(status) {
  if (!grayCtx) return;
  const canvas = grayCtx.canvas;
  // Overlay is drawn on top of the existing gray image.
  // The gray image is rendered first by dashboard.js, then this is called.

  if (showBoundary) {
    // Left boundary in red
    if (Array.isArray(status.left_boundary)) {
      receiverCore.drawPolyline(grayCtx, status.left_boundary, '#ef4444', 1);
    }
    // Right boundary in blue
    if (Array.isArray(status.right_boundary)) {
      receiverCore.drawPolyline(grayCtx, status.right_boundary, '#3b82f6', 1);
    }
  }

  if (showCenterline) {
    const cl = getSelectedCenterline(status);
    if (cl) {
      receiverCore.drawPolyline(grayCtx, cl, '#22c55e', 1);
    }
  }
}

function drawIpmOverlays(status) {
  if (!ipmCtx) return;
  const canvas = ipmCtx.canvas;

  if (showBoundary) {
    if (Array.isArray(status.left_boundary)) {
      receiverCore.drawPolyline(ipmCtx, status.left_boundary, '#ef4444', 1);
    }
    if (Array.isArray(status.right_boundary)) {
      receiverCore.drawPolyline(ipmCtx, status.right_boundary, '#3b82f6', 1);
    }
  }

  if (showCenterline) {
    const cl = getSelectedCenterline(status);
    if (cl) receiverCore.drawPolyline(ipmCtx, cl, '#22c55e', 1);
  }

  if (showAnchor) {
    const leftCorner = Array.isArray(status.cross_lower_left_corner_point) ? status.cross_lower_left_corner_point : null;
    const rightCorner = Array.isArray(status.cross_lower_right_corner_point) ? status.cross_lower_right_corner_point : null;
    if (leftCorner && leftCorner.length === 2) {
      receiverCore.drawPointSet(ipmCtx, [leftCorner], '#fbbf24', 4);
    }
    if (rightCorner && rightCorner.length === 2) {
      receiverCore.drawPointSet(ipmCtx, [rightCorner], '#fbbf24', 4);
    }
  }
}

function getSelectedCenterline(status) {
  if (Array.isArray(status.ipm_centerline_selected_shift)) return status.ipm_centerline_selected_shift;
  if (status.ipm_centerline_source === 1) return status.ipm_centerline_from_right_shift;
  return status.ipm_centerline_from_left_shift;
}

// Bind checkbox toggles
function bindOverlayToggles() {
  document.getElementById('togBoundary').addEventListener('change', function() {
    showBoundary = this.checked;
    // re-render overlays on next frame
  });
  document.getElementById('togCenterline').addEventListener('change', function() {
    showCenterline = this.checked;
  });
  document.getElementById('togAnchor').addEventListener('change', function() {
    showAnchor = this.checked;
  });
  document.getElementById('togDir').addEventListener('change', function() {
    showDir = this.checked;
  });
  document.getElementById('togSamplePoints').addEventListener('change', function() {
    showSamplePoints = this.checked;
  });
}
```

- [ ] **Step 2: 验证**
  启动 `server.js`，打开仪表盘，切换各 checkbox 确认叠加层正确显示/隐藏。

- [ ] **Step 3: Commit**

```bash
git add tools/pc_receiver_js/public/dashboard_overlay.js
git commit -m "feat: add canvas overlay layer for boundaries, centerlines, and anchors"
```

---

### Task 5: 主逻辑 + 预设切换

**文件:**
- 新建: `tools/pc_receiver_js/public/dashboard.js`

- [ ] **Step 1: 写入 dashboard.js**

```javascript
// dashboard.js — 主控制逻辑

let currentPreset = 'drive';
let latestStatus = {};
let frameSeq = 0;
let fpsCounter = 0;
let fpsTimer = Date.now();
let currentFps = 0;

// Canvas contexts
const grayCanvas = document.getElementById('grayCanvas');
const grayCtx = grayCanvas.getContext('2d');
const ipmCanvas = document.getElementById('ipmCanvas');
const ipmCtx = ipmCanvas.getContext('2d');

// Init
initOverlayContexts(grayCtx, ipmCtx);
bindOverlayToggles();

// ===== Preset switching =====
document.querySelectorAll('.preset-btn').forEach(btn => {
  btn.addEventListener('click', function() {
    document.querySelectorAll('.preset-btn').forEach(b => b.classList.remove('active'));
    this.classList.add('active');
    currentPreset = this.dataset.preset;
    renderCards(latestStatus, currentPreset);
    applyPresetPanels(currentPreset);
  });
});

// ===== Panel collapsible toggle =====
document.querySelectorAll('.panel-header').forEach(header => {
  header.addEventListener('click', function() {
    this.parentElement.classList.toggle('open');
  });
});

// ===== WebSocket =====
function connectWs() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/ws`);

  ws.onmessage = function(ev) {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'status') {
      onStatus(msg.data || msg);
    } else if (msg.type === 'frame') {
      onFrame(msg);
    }
  };

  ws.onclose = function() {
    document.getElementById('connPill').className = 'status-pill warn';
    document.getElementById('connPill').textContent = '断开';
    setTimeout(connectWs, 2000);
  };

  ws.onopen = function() {
    document.getElementById('connPill').className = 'status-pill ok';
    document.getElementById('connPill').textContent = '连接';
  };
}

function onStatus(status) {
  latestStatus = status || {};
  // FPS counting
  fpsCounter++;
  const now = Date.now();
  if (now - fpsTimer >= 1000) {
    currentFps = fpsCounter;
    fpsCounter = 0;
    fpsTimer = now;
  }
  latestStatus._fps = currentFps;

  // Render
  renderCards(latestStatus, currentPreset);
  renderAllPanels(latestStatus);
}

function onFrame(msg) {
  frameSeq++;
  // Render gray image from URL
  const grayImg = new Image();
  grayImg.onload = function() {
    grayCtx.clearRect(0, 0, grayCanvas.width, grayCanvas.height);
    grayCtx.drawImage(grayImg, 0, 0, grayCanvas.width, grayCanvas.height);
    drawOverlays(latestStatus);
  };
  grayImg.src = receiverCore.frameUrlForMode('gray') + '&_' + frameSeq;

  // IPM image
  const ipmImg = new Image();
  ipmImg.onload = function() {
    ipmCtx.clearRect(0, 0, ipmCanvas.width, ipmCanvas.height);
    ipmCtx.drawImage(ipmImg, 0, 0, ipmCanvas.width, ipmCanvas.height);
    drawOverlays(latestStatus);
  };
  ipmImg.src = receiverCore.frameUrlForMode('ipm') + '&_' + frameSeq;

  // FPS pill
  document.getElementById('fpsPill').textContent = currentFps + ' fps';
}

// ===== Pixel probe on canvases =====
function bindPixelProbe(canvas, ctx, displayEl) {
  canvas.addEventListener('mousemove', function(ev) {
    const pt = receiverCore.getCanvasPoint(ev, canvas);
    if (!pt) return;
    const x = Math.floor(pt.x);
    const y = Math.floor(pt.y);
    displayEl.textContent = `x:${x} y:${y}`;
  });
  canvas.addEventListener('mouseleave', function() {
    displayEl.textContent = '-';
  });
}

bindPixelProbe(grayCanvas, grayCtx, document.getElementById('grayPixel'));
bindPixelProbe(ipmCanvas, ipmCtx, document.getElementById('ipmPixel'));

// ===== Recording state =====
let recordingActive = false;
let recordingFrames = [];
let recordingStatuses = [];

document.getElementById('recordBtn').addEventListener('click', async function() {
  if (recordingActive) {
    // Stop recording
    recordingActive = false;
    this.textContent = '录制';
    this.classList.remove('recording');
    document.getElementById('recordStatus').textContent = '';
    // Save recording
    if (recordingFrames.length > 0 || recordingStatuses.length > 0) {
      await saveRecordingFramesToServer(recordingFrames, recordingStatuses);
    }
    recordingFrames = [];
    recordingStatuses = [];
  } else {
    // Start recording
    recordingActive = true;
    this.textContent = '停止';
    this.classList.add('recording');
    recordingFrames = [];
    recordingStatuses = [];
  }
});

async function saveRecordingFramesToServer(frames, statuses) {
  const folderName = 'rec_' + new Date().toISOString().replace(/[:.]/g, '-');
  await fetch('/api/recording/save', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ folder: folderName, frames: frames, statuses: statuses }),
  });
  document.getElementById('recordStatus').textContent = '已保存';
  setTimeout(() => { document.getElementById('recordStatus').textContent = ''; }, 3000);
}

// Hook recording into onFrame and onStatus
const origOnFrame = onFrame;
onFrame = function(msg) {
  origOnFrame(msg);
  if (recordingActive) {
    recordingFrames.push({ ts: Date.now(), msg: msg });
  }
};

const origOnStatus = onStatus;
onStatus = function(status) {
  origOnStatus(status);
  if (recordingActive) {
    recordingStatuses.push({ ts: Date.now(), status: JSON.parse(JSON.stringify(status)) });
  }
};

// ===== Start =====
applyPresetPanels(currentPreset);
connectWs();
```

- [ ] **Step 2: 验证**
  - 启动 server.js，打开仪表盘
  - 确认 WebSocket 连接成功（连接 pill 变绿）
  - 确认图像实时刷新，L1 卡片更新
  - 切换三预设，确认卡片内容和展开面板变化
  - 点击录制按钮，等待几秒后点击停止，确认录制保存

- [ ] **Step 3: Commit**

```bash
git add tools/pc_receiver_js/public/dashboard.js
git commit -m "feat: add dashboard main controller with websocket, preset switching, and recording"
```

---

### Task 6: 回放功能

**文件:**
- 新建: `tools/pc_receiver_js/public/dashboard_playback.js`

- [ ] **Step 1: 写入 dashboard_playback.js**

```javascript
// dashboard_playback.js — 回放控制

let playbackActive = false;
let playbackData = null;       // { statuses: [...], frames: [...] }
let playbackIndex = 0;
let playbackTimer = null;
let playbackPlaying = false;

// ===== Load recording =====
async function loadPlayback(folderName) {
  const resp = await fetch('/api/recording/load/' + encodeURIComponent(folderName));
  const data = await resp.json();
  if (!data.ok) { alert('加载失败: ' + data.message); return; }
  playbackData = data;
  playbackIndex = 0;
  playbackActive = true;
  playbackPlaying = false;
  document.getElementById('playbackBar').classList.add('visible');
  seekPlayback(0);
}

// ===== Playback controls =====
document.getElementById('pbPlay').addEventListener('click', function() {
  if (!playbackActive) return;
  if (playbackPlaying) {
    pausePlayback();
  } else {
    playPlayback();
  }
});

document.getElementById('pbBack').addEventListener('click', function() {
  seekPlaybackBy(-1.0);
});

document.getElementById('pbFwd').addEventListener('click', function() {
  seekPlaybackBy(1.0);
});

document.getElementById('pbClose').addEventListener('click', function() {
  stopPlayback();
});

document.getElementById('pbSeek').addEventListener('input', function() {
  if (!playbackData) return;
  const pct = Number(this.value) / 100;
  const maxIdx = playbackData.statuses.length - 1;
  playbackIndex = Math.floor(pct * maxIdx);
  renderPlaybackFrame();
});

function playPlayback() {
  playbackPlaying = true;
  document.getElementById('pbPlay').textContent = '⏸';
  playbackTimer = setInterval(function() {
    if (playbackIndex >= playbackData.statuses.length - 1) {
      pausePlayback();
      return;
    }
    playbackIndex++;
    renderPlaybackFrame();
    updatePlaybackUI();
  }, 1000 / 30); // 30fps playback
}

function pausePlayback() {
  playbackPlaying = false;
  document.getElementById('pbPlay').textContent = '▶';
  if (playbackTimer) { clearInterval(playbackTimer); playbackTimer = null; }
}

function seekPlaybackBy(deltaSec) {
  if (!playbackData) return;
  const delta = Math.round(deltaSec * 30);
  playbackIndex = Math.max(0, Math.min(playbackData.statuses.length - 1, playbackIndex + delta));
  renderPlaybackFrame();
  updatePlaybackUI();
}

function seekPlayback(index) {
  playbackIndex = Math.max(0, Math.min((playbackData ? playbackData.statuses.length - 1 : 0), index));
  renderPlaybackFrame();
  updatePlaybackUI();
}

function stopPlayback() {
  pausePlayback();
  playbackActive = false;
  playbackData = null;
  document.getElementById('playbackBar').classList.remove('visible');
  // Resume live WebSocket
}

function renderPlaybackFrame() {
  if (!playbackData || !playbackData.statuses[playbackIndex]) return;
  const status = playbackData.statuses[playbackIndex].status;
  latestStatus = status;
  renderCards(status, currentPreset);
  renderAllPanels(status);
}

function updatePlaybackUI() {
  if (!playbackData) return;
  const total = playbackData.statuses.length;
  const current = playbackIndex;
  const pct = total > 1 ? (current / (total - 1)) * 100 : 0;

  document.getElementById('pbSeek').value = String(Math.round(pct));
  document.getElementById('pbTime').textContent =
    formatTime(current / 30) + ' / ' + formatTime(total / 30);
}

function formatTime(seconds) {
  const m = Math.floor(seconds / 60);
  const s = Math.floor(seconds % 60);
  return m + ':' + String(s).padStart(2, '0');
}
```

- [ ] **Step 2: 验证**
  - 录制一段数据
  - 加载回放
  - 确认暂停/播放/快进/快退/拖动进度条功能正常
  - 确认 L1 卡片和 L3 面板随回放帧更新

- [ ] **Step 3: Commit**

```bash
git add tools/pc_receiver_js/public/dashboard_playback.js
git commit -m "feat: add playback controls with seek, play/pause, and frame stepping"
```

---

### Task 7: 快捷调参面板

**文件:**
- 新建: `tools/pc_receiver_js/public/dashboard_params.js`

- [ ] **Step 1: 写入 dashboard_params.js**

```javascript
// dashboard_params.js — 快捷调参侧边栏

const PARAM_DEFS = [
  { key: 'pid_common_kp', label: '公共 Kp', min: 0, max: 10, step: 0.01 },
  { key: 'pid_common_ki', label: '公共 Ki', min: 0, max: 5, step: 0.01 },
  { key: 'pid_common_kd', label: '公共 Kd', min: 0, max: 5, step: 0.01 },
  { key: 'pid_left_kp', label: '左轮 Kp', min: 0, max: 10, step: 0.01 },
  { key: 'pid_left_ki', label: '左轮 Ki', min: 0, max: 5, step: 0.01 },
  { key: 'pid_left_kd', label: '左轮 Kd', min: 0, max: 5, step: 0.01 },
  { key: 'pid_right_kp', label: '右轮 Kp', min: 0, max: 10, step: 0.01 },
  { key: 'pid_right_ki', label: '右轮 Ki', min: 0, max: 5, step: 0.01 },
  { key: 'pid_right_kd', label: '右轮 Kd', min: 0, max: 5, step: 0.01 },
  { key: 'pid_common_desired_base_speed', label: '目标基础速度', min: 0, max: 500, step: 1 },
];

let currentParamValues = {};

function buildParamsPanel() {
  const container = document.getElementById('paramsContent');
  let html = '';
  PARAM_DEFS.forEach(def => {
    html += `<label>${def.label}</label>
      <div style="display:flex;align-items:center;gap:8px;">
        <input type="range" id="param_${def.key}" min="${def.min}" max="${def.max}" step="${def.step}" value="0" style="flex:1;">
        <span class="param-val" id="param_${def.key}_val">--</span>
      </div>`;
  });
  container.innerHTML = html;

  // Bind sliders
  PARAM_DEFS.forEach(def => {
    const slider = document.getElementById('param_' + def.key);
    const display = document.getElementById('param_' + def.key + '_val');
    slider.addEventListener('input', function() {
      display.textContent = this.value;
      currentParamValues[def.key] = parseFloat(this.value);
    });
    slider.addEventListener('change', function() {
      display.textContent = this.value;
      currentParamValues[def.key] = parseFloat(this.value);
    });
  });
}

function syncParamsFromStatus(status) {
  PARAM_DEFS.forEach(def => {
    const val = status[def.key];
    if (val !== null && val !== undefined) {
      const slider = document.getElementById('param_' + def.key);
      const display = document.getElementById('param_' + def.key + '_val');
      if (slider && display) {
        slider.value = String(val);
        display.textContent = String(val);
        currentParamValues[def.key] = parseFloat(val);
      }
    }
  });
}

// ===== Panel toggle =====
document.getElementById('paramsBtn').addEventListener('click', function() {
  const panel = document.getElementById('paramsPanel');
  panel.classList.toggle('visible');
  if (panel.classList.contains('visible')) {
    syncParamsFromStatus(latestStatus);
  }
});

document.getElementById('paramsCloseBtn').addEventListener('click', function() {
  document.getElementById('paramsPanel').classList.remove('visible');
});

// ===== Smart apply =====
document.getElementById('paramsApplyBtn').addEventListener('click', async function() {
  const resultEl = document.getElementById('paramsResult');
  resultEl.textContent = '正在应用...';
  try {
    const resp = await fetch('/api/config/apply', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ params: currentParamValues }),
    });
    const j = await resp.json();
    if (j.ok) {
      resultEl.textContent = '已热更新成功';
      resultEl.style.color = '#6ee7b7';
    } else if (j.offline_ok) {
      resultEl.textContent = '热更新不可用，已通过 SSH 离线写入';
      resultEl.style.color = '#fcd34d';
    } else {
      resultEl.textContent = '应用失败: ' + (j.message || '未知错误');
      resultEl.style.color = '#fca5a5';
    }
  } catch (e) {
    resultEl.textContent = '请求失败: ' + e.message;
    resultEl.style.color = '#fca5a5';
  }
});

// Initialize
buildParamsPanel();
```

- [ ] **Step 2: 验证**
  - 点击参数按钮，确认侧边栏滑出
  - 滑块值应和当前 status 值同步
  - 拖动滑块并点击应用
  - 确认"正在应用" → 成功/降级离线的结果提示

- [ ] **Step 3: Commit**

```bash
git add tools/pc_receiver_js/public/dashboard_params.js
git commit -m "feat: add quick param tuning sidebar with smart apply (hot/offline)"
```

---

### Task 8: config.html 精简

**文件:**
- 修改: `tools/pc_receiver_js/public/config.html`
- 修改: `tools/pc_receiver_js/public/config_app.js`

- [ ] **Step 1: 合并"应用到主板"和"离线 SSH 写入主板"为一个智能按钮**

在 config.html 中：
- 删除"应用到主板"和"离线 SSH 写入主板"两个按钮
- 新增一个"智能应用到主板"按钮
- 按钮逻辑：点击后先尝试热更新 `/api/config/apply`，失败返回 502 时自动调用 SSH 写入

在 config_app.js 中，新增函数：

```javascript
async function smartApply() {
  setStatus('正在热更新...', 'warn');
  try {
    // Try hot update first
    const hotResp = await fetch('/api/config/apply', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ toml: configEditor.value }),
    });
    const hotJson = await hotResp.json();
    if (hotJson.ok) {
      setStatus('热更新成功', 'ok');
      return;
    }
  } catch (e) { /* fall through to offline */ }

  // Fallback to SSH offline
  setStatus('热更新不可用，正在通过 SSH 离线写入...', 'warn');
  try {
    const sshResp = await fetch('/api/config/ssh_push', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ toml: configEditor.value }),
    });
    const sshJson = await sshResp.json();
    if (sshJson.ok) {
      setStatus('已通过 SSH 离线写入（需重启主板生效）', 'warn');
    } else {
      setStatus('SSH 写入失败: ' + (sshJson.message || ''), 'error');
    }
  } catch (e) {
    setStatus('离线写入失败: ' + e.message, 'error');
  }
}
```

- [ ] **Step 2: 验证**
  - 主板在线时点击智能应用 → 热更新成功
  - 主板离线时点击智能应用 → 自动降级 SSH 写入
  - 两者都不可用时 → 显示错误

- [ ] **Step 3: Commit**

```bash
git add tools/pc_receiver_js/public/config.html tools/pc_receiver_js/public/config_app.js
git commit -m "feat: merge hot-update and offline SSH into one smart apply button"
```

---

### Task 9: server.js 补充 API

**文件:**
- 修改: `tools/pc_receiver_js/server.js`

需要在 server.js 中新增两个 API 端点：

- [ ] **Step 1: 添加录制保存 API**

```javascript
// POST /api/recording/save — 保存录制数据
app.post('/api/recording/save', express.json(), (req, res) => {
  try {
    const { folder, statuses } = req.body;
    const folderPath = path.join(recordingBaseDir, folder);
    fs.mkdirSync(folderPath, { recursive: true });
    fs.writeFileSync(path.join(folderPath, 'status.json'), JSON.stringify(statuses, null, 2), 'utf8');
    sendJson(res, 200, { ok: true, folder: folder });
  } catch (e) {
    sendJson(res, 500, { ok: false, message: String(e.message) });
  }
});
```

- [ ] **Step 2: 添加录制加载 API**

```javascript
// GET /api/recording/load/:folder — 加载录制数据
app.get('/api/recording/load/:folder', (req, res) => {
  try {
    const folderPath = path.join(recordingBaseDir, req.params.folder);
    const statusPath = path.join(folderPath, 'status.json');
    if (!fs.existsSync(statusPath)) {
      sendJson(res, 404, { ok: false, message: 'recording not found' });
      return;
    }
    const statuses = JSON.parse(fs.readFileSync(statusPath, 'utf8'));
    sendJson(res, 200, { ok: true, statuses: statuses });
  } catch (e) {
    sendJson(res, 500, { ok: false, message: String(e.message) });
  }
});
```

- [ ] **Step 3: 添加智能热更新 API**

```javascript
// POST /api/config/apply — 智能应用（先热更新，失败返回特定状态码）
app.post('/api/config/apply', express.json(), async (req, res) => {
  try {
    const result = await proxyToBoard('/api/config/apply', req.body);
    sendJson(res, 200, result);
  } catch (e) {
    sendJson(res, 502, { ok: false, message: String(e.message), offline_available: !!boardSshHost });
  }
});
```

- [ ] **Step 4: 验证**
  - 录制一段数据后确认 `status.json` 写入磁盘
  - 通过 API 加载录制数据确认返回正确

- [ ] **Step 5: Commit**

```bash
git add tools/pc_receiver_js/server.js
git commit -m "feat: add recording save/load and smart config apply API endpoints"
```

---

### Task 10: 端到端集成验证

- [ ] **Step 1: 启动服务器**
  ```bash
  cd tools/pc_receiver_js && node server.js
  ```

- [ ] **Step 2: 验证三级数据渲染**
  - L1 卡片：6 个卡片值与 WebSocket 推送的 status 一致
  - L2 图像：灰度图和 IPM 图正常刷新，叠加层随 checkbox 切换
  - L3 面板：6 个面板均能展开并显示正确数据

- [ ] **Step 3: 验证三预设切换**
  - 驾驶模式 → L1 显示转速相关，面板全部折叠
  - 视觉调试 → L1 显示推理/中线相关，元素状态机/绕行/dir 面板展开
  - 车控调试 → L1 显示速度/PID 相关，PID/陀螺仪面板展开

- [ ] **Step 4: 验证录制回放**
  - 录制 10 秒 → 停止 → 显示已保存
  - 加载回放 → 播放控制条显示 → 播放/暂停/快进快退正常

- [ ] **Step 5: 验证快捷调参**
  - 打开侧边栏 → 滑块值与当前 status 一致 → 修改值 → 应用 → 确认结果提示

- [ ] **Step 6: 验证 1820x1200 一页无滚动**
  - 浏览器窗口调整为 1820x1200
  - 确认所有内容可见，无水平或垂直滚动条

- [ ] **Step 7: Commit 最终调整**

```bash
git add -A && git commit -m "chore: end-to-end integration fixes and final adjustments"
```
