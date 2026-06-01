# PID 三级串级实时折线图 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在网页"历史角点"栏下方添加位置环、角速度环、速度环三个 VOFA 风格实时折线图栏目，所有数据存贮和绘图在浏览器端完成。

**Architecture:** 在 `pullStatus()` 中采集 PID 字段到环形缓冲区（最多 300 点），用独立 Canvas 绘制多迹线折线图。每个图表在 120ms 轮询周期中增量重绘，新数据从右侧进入、旧数据向左滚动（VOFA 风格）。纯前端实现，不修改服务端。

**Tech Stack:** HTML5 Canvas 2D, vanilla JavaScript (no chart library), CSS Grid 布局

---

## 文件结构

| 文件 | 职责 |
|------|------|
| `tools/pc_receiver_js/public/index.html` | 新增 HTML 结构、CSS 样式、环形缓冲区、图表渲染函数、pullStatus 采集钩子 |

只修改这一个文件。

---

## 数据字段映射

从 `latestStatus` 对象中读取的字段（已在 TCP 状态流中存在）：

**位置环（Position Loop）：**
| 迹线 | 字段 | fallback | 说明 |
|------|------|----------|------|
| 目标值 | `pid_common_position_pid_target` | — | 恒为 0（期望车在赛道中心） |
| 实际值 | `pid_common_control_error_px` | — | 经死区/降增益处理后的像素偏差 |

**角速度环（Yaw Rate Loop）：**
| 迹线 | 字段 | fallback | 说明 |
|------|------|----------|------|
| 目标值 | `pid_common_yaw_rate_ref_dps` | — | 期望横摆角速度（dps） |
| 实际值 | `pid_common_measured_yaw_rate_dps` | — | 陀螺仪实测横摆角速度（dps） |

**速度环左轮（Speed Loop Left）：**
| 迹线 | 字段 | fallback 链 | 说明 |
|------|------|------------|------|
| 目标值 | `pid_left_target_count` | `left_target_count` | 左轮目标编码器增量 |
| 实际值 | `pid_left_feedback` | `left_filtered_count` → `pid_left_current_count` → `left_current_count` | 左轮滤波后反馈 |

**速度环右轮（Speed Loop Right）：**
| 迹线 | 字段 | fallback 链 | 说明 |
|------|------|------------|------|
| 目标值 | `pid_right_target_count` | `right_target_count` | 右轮目标编码器增量 |
| 实际值 | `pid_right_feedback` | `left_filtered_count` → `pid_right_current_count` → `right_current_count` | 右轮滤波后反馈 |

---

## 环形缓冲区设计

```javascript
const RING_CAPACITY = 300; // 约 36 秒 @ 120ms 轮询
const pidHistory = {
  // 位置环
  pos_target:   new Float32Array(RING_CAPACITY),
  pos_actual:   new Float32Array(RING_CAPACITY),
  // 角速度环
  yaw_target:   new Float32Array(RING_CAPACITY),
  yaw_actual:   new Float32Array(RING_CAPACITY),
  // 速度环 - 左轮
  spd_left_target:  new Float32Array(RING_CAPACITY),
  spd_left_actual:  new Float32Array(RING_CAPACITY),
  // 速度环 - 右轮
  spd_right_target: new Float32Array(RING_CAPACITY),
  spd_right_actual: new Float32Array(RING_CAPACITY),
  // 时间戳（用于 X 轴标签）
  timestamps: new Float64Array(RING_CAPACITY),
  head: 0,    // 下一个写入位置
  count: 0,   // 当前已写入点数（≤ RING_CAPACITY）
};
```

写入逻辑：
```javascript
function pushPidHistory(status) {
  const i = pidHistory.head;
  const fn = (chain) => firstValue(...chain);

  pidHistory.pos_target[i]   = fn([status.pid_common_position_pid_target])    || 0;
  pidHistory.pos_actual[i]   = fn([status.pid_common_control_error_px])      || 0;
  pidHistory.yaw_target[i]   = fn([status.pid_common_yaw_rate_ref_dps])      || 0;
  pidHistory.yaw_actual[i]   = fn([status.pid_common_measured_yaw_rate_dps]) || 0;
  pidHistory.spd_left_target[i]  = fn([status.pid_left_target_count, status.left_target_count]) || 0;
  pidHistory.spd_left_actual[i]  = fn([status.pid_left_feedback, status.left_filtered_count, status.pid_left_current_count, status.left_current_count]) || 0;
  pidHistory.spd_right_target[i] = fn([status.pid_right_target_count, status.right_target_count]) || 0;
  pidHistory.spd_right_actual[i] = fn([status.pid_right_feedback, status.right_filtered_count, status.pid_right_current_count, status.right_current_count]) || 0;
  pidHistory.timestamps[i] = Date.now();

  pidHistory.head = (i + 1) % RING_CAPACITY;
  if (pidHistory.count < RING_CAPACITY) pidHistory.count++;
}
```

调用点：`pullStatus()` 中，在 `latestStatus = j;` 之后立即调用 `pushPidHistory(j)`，然后调用 `renderPidCharts()`。

---

### Task 1: 添加 HTML 结构——三个详情面板

**Files:**
- Modify: `tools/pc_receiver_js/public/index.html`

在"原始状态全文" `<details>` 之前（约 line 1978）插入三个新面板。

- [ ] **Step 1: 插入位置环面板 HTML**

```html
<details class="detail-panel pid-chart-panel" id="posChartPanel" open>
  <summary>位置环 — 目标值 vs 实际偏差 (px)</summary>
  <div class="detail-panel-body single">
    <div class="card chart-card">
      <div class="chart-legend">
        <span class="legend-dot" style="background:#fbbf24;"></span> 目标值(0)
        <span class="legend-dot" style="background:#38bdf8;"></span> 实际偏差(px)
      </div>
      <canvas id="posChartCanvas" width="900" height="220"></canvas>
    </div>
  </div>
</details>
```

- [ ] **Step 2: 插入角速度环面板 HTML**

```html
<details class="detail-panel pid-chart-panel" id="yawChartPanel" open>
  <summary>角速度环 — 目标值 vs 实际值 (dps)</summary>
  <div class="detail-panel-body single">
    <div class="card chart-card">
      <div class="chart-legend">
        <span class="legend-dot" style="background:#fbbf24;"></span> 目标角速度(dps)
        <span class="legend-dot" style="background:#38bdf8;"></span> 实测角速度(dps)
      </div>
      <canvas id="yawChartCanvas" width="900" height="220"></canvas>
    </div>
  </div>
</details>
```

- [ ] **Step 3: 插入速度环面板 HTML（含左右两幅图）**

```html
<details class="detail-panel pid-chart-panel" id="spdChartPanel" open>
  <summary>速度环 — 左右轮 目标值 vs 实际反馈 (counts/5ms)</summary>
  <div class="detail-panel-body">
    <div class="card chart-card">
      <div class="chart-legend">
        <span class="legend-dot" style="background:#fbbf24;"></span> 左轮目标
        <span class="legend-dot" style="background:#38bdf8;"></span> 左轮反馈
      </div>
      <canvas id="spdLeftChartCanvas" width="680" height="200"></canvas>
    </div>
    <div class="card chart-card">
      <div class="chart-legend">
        <span class="legend-dot" style="background:#fbbf24;"></span> 右轮目标
        <span class="legend-dot" style="background:#38bdf8;"></span> 右轮反馈
      </div>
      <canvas id="spdRightChartCanvas" width="680" height="200"></canvas>
    </div>
  </div>
</details>
```

---

### Task 2: 添加 CSS 样式

**Files:**
- Modify: `tools/pc_receiver_js/public/index.html`

在 `<style>` 块中 `.detail-panel-body canvas` 规则之后（约 line 1599）追加。

- [ ] **Step 4: 追加图表相关 CSS**

```css
.pid-chart-panel .chart-card {
  padding: 8px 10px;
}
.chart-legend {
  display: flex;
  align-items: center;
  gap: 16px;
  margin-bottom: 4px;
  font-size: 11px;
  color: var(--console-dim);
}
.legend-dot {
  display: inline-block;
  width: 10px;
  height: 10px;
  border-radius: 2px;
  margin-right: 2px;
}
```

---

### Task 3: 实现环形缓冲区与数据采集

**Files:**
- Modify: `tools/pc_receiver_js/public/index.html`

在 `<script>` 块顶部（约 line 2450，`let latestStatus = {};` 附近）添加缓冲区定义，以及 `pushPidHistory()` 函数。

- [ ] **Step 5: 定义环形缓冲区常量与存储**

```javascript
const PID_CHART_RING_CAPACITY = 300;
const pidHistory = {
  pos_target:   new Float32Array(PID_CHART_RING_CAPACITY),
  pos_actual:   new Float32Array(PID_CHART_RING_CAPACITY),
  yaw_target:   new Float32Array(PID_CHART_RING_CAPACITY),
  yaw_actual:   new Float32Array(PID_CHART_RING_CAPACITY),
  spd_left_target:  new Float32Array(PID_CHART_RING_CAPACITY),
  spd_left_actual:  new Float32Array(PID_CHART_RING_CAPACITY),
  spd_right_target: new Float32Array(PID_CHART_RING_CAPACITY),
  spd_right_actual: new Float32Array(PID_CHART_RING_CAPACITY),
  timestamps: new Float64Array(PID_CHART_RING_CAPACITY),
  head: 0,
  count: 0,
};
```

- [ ] **Step 6: 实现 pushPidHistory 函数**

```javascript
function pushPidHistory(status) {
  const i = pidHistory.head;
  const fn = (chain) => {
    for (let k = 0; k < chain.length; k++) {
      const v = chain[k];
      if (v !== undefined && v !== null && Number.isFinite(v)) return v;
    }
    return 0;
  };

  pidHistory.pos_target[i]   = fn([status.pid_common_position_pid_target]) || 0;
  pidHistory.pos_actual[i]   = fn([status.pid_common_control_error_px]) || 0;
  pidHistory.yaw_target[i]   = fn([status.pid_common_yaw_rate_ref_dps]) || 0;
  pidHistory.yaw_actual[i]   = fn([status.pid_common_measured_yaw_rate_dps]) || 0;
  pidHistory.spd_left_target[i]  = fn([status.pid_left_target_count, status.left_target_count]) || 0;
  pidHistory.spd_left_actual[i]  = fn([status.pid_left_feedback, status.left_filtered_count, status.pid_left_current_count, status.left_current_count]) || 0;
  pidHistory.spd_right_target[i] = fn([status.pid_right_target_count, status.right_target_count]) || 0;
  pidHistory.spd_right_actual[i] = fn([status.pid_right_feedback, status.right_filtered_count, status.pid_right_current_count, status.right_current_count]) || 0;
  pidHistory.timestamps[i] = Date.now();

  pidHistory.head = (i + 1) % PID_CHART_RING_CAPACITY;
  if (pidHistory.count < PID_CHART_RING_CAPACITY) pidHistory.count++;
}
```

- [ ] **Step 7: 在 pullStatus 中挂入数据采集钩子**

在 `pullStatus()` 函数中 `latestStatus = j;` 之后插入：

```javascript
pushPidHistory(j);
renderPidCharts();
```

---

### Task 4: 实现 VOFA 风格折线图渲染器

**Files:**
- Modify: `tools/pc_receiver_js/public/index.html`

在 `<script>` 块中定义核心渲染函数。放在 `pushPidHistory` 之后。

- [ ] **Step 8: 实现单图多迹线渲染函数 drawPidChart**

```javascript
function drawPidChart(canvas, seriesList, options) {
  // seriesList: [{ name, values: Float32Array, color, count, head }]
  // options: { yMin, yMax, yLabel, title }
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  const M = { left: 50, right: 14, top: 14, bottom: 28 };
  const pw = Math.max(1, w - M.left - M.right);
  const ph = Math.max(1, h - M.top - M.bottom);

  // 背景
  ctx.fillStyle = '#0d1116';
  ctx.fillRect(0, 0, w, h);

  const yMin = options.yMin, yMax = options.yMax;
  const yRange = yMax - yMin || 1;
  const yAt = (v) => M.top + ((yMax - v) / yRange) * ph;
  const xAt = (idx) => M.left + (idx / (PID_CHART_RING_CAPACITY - 1)) * pw;

  // 网格（5×5）
  ctx.strokeStyle = 'rgba(51,65,85,0.4)';
  ctx.lineWidth = 0.5;
  const GRID = 5;
  for (let i = 0; i <= GRID; i++) {
    const t = i / GRID;
    const gx = M.left + t * pw;
    const gy = M.top + t * ph;
    ctx.beginPath(); ctx.moveTo(gx, M.top); ctx.lineTo(gx, M.top + ph); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(M.left, gy); ctx.lineTo(M.left + pw, gy); ctx.stroke();
  }

  // Y 轴标签
  ctx.fillStyle = '#64748b';
  ctx.font = '10px "Noto Sans SC", "Microsoft YaHei", sans-serif';
  ctx.textAlign = 'right';
  for (let i = 0; i <= GRID; i++) {
    const v = yMax - (i / GRID) * yRange;
    ctx.fillText(v.toFixed(1), M.left - 6, M.top + (i / GRID) * ph + 4);
  }

  // 零线
  if (yMin < 0 && yMax > 0) {
    const yz = yAt(0);
    ctx.strokeStyle = '#475569';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(M.left, yz); ctx.lineTo(M.left + pw, yz); ctx.stroke();
    ctx.setLineDash([]);
  }

  // 标题
  ctx.fillStyle = '#94a3b8';
  ctx.font = '11px "Noto Sans SC", "Microsoft YaHei", sans-serif';
  ctx.textAlign = 'left';
  ctx.fillText(options.title || '', M.left, M.top - 2);

  const totalCount = pidHistory.count;
  if (totalCount < 2) {
    ctx.fillStyle = '#64748b';
    ctx.textAlign = 'center';
    ctx.fillText('等待数据...', w / 2, h / 2);
    return;
  }

  // 从 head 往前读取 count 个点，按时间顺序绘制
  const head = pidHistory.head;
  const cap = PID_CHART_RING_CAPACITY;

  for (const series of seriesList) {
    const vals = series.values;
    ctx.strokeStyle = series.color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let started = false;
    for (let j = 0; j < totalCount; j++) {
      const idx = (head - totalCount + j + cap) % cap;
      const v = Math.max(yMin, Math.min(yMax, vals[idx] || 0));
      const sx = xAt(j);
      const sy = yAt(v);
      if (!started) { ctx.moveTo(sx, sy); started = true; }
      else ctx.lineTo(sx, sy);
    }
    ctx.stroke();
  }

  // 图例
  ctx.textAlign = 'left';
  let legendX = M.left;
  for (const series of seriesList) {
    ctx.fillStyle = series.color;
    ctx.fillRect(legendX, M.top + ph + 10, 10, 10);
    ctx.fillStyle = '#94a3b8';
    ctx.fillText(series.name, legendX + 14, M.top + ph + 20);
    legendX += ctx.measureText(series.name).width + 36;
  }
}
```

- [ ] **Step 9: 实现 renderPidCharts 调度函数**

```javascript
function renderPidCharts() {
  const h = pidHistory;

  // 位置环
  drawPidChart(document.getElementById('posChartCanvas'), [
    { name: '目标', values: h.pos_target, color: '#fbbf24' },
    { name: '实际', values: h.pos_actual, color: '#38bdf8' },
  ], { yMin: -80, yMax: 80, title: '位置环 (px)' });

  // 角速度环
  drawPidChart(document.getElementById('yawChartCanvas'), [
    { name: '目标', values: h.yaw_target, color: '#fbbf24' },
    { name: '实际', values: h.yaw_actual, color: '#38bdf8' },
  ], { yMin: -400, yMax: 400, title: '角速度环 (dps)' });

  // 速度环 - 左轮
  drawPidChart(document.getElementById('spdLeftChartCanvas'), [
    { name: '左目标', values: h.spd_left_target, color: '#fbbf24' },
    { name: '左反馈', values: h.spd_left_actual, color: '#38bdf8' },
  ], { yMin: -100, yMax: 600, title: '左轮速度环 (counts/5ms)' });

  // 速度环 - 右轮
  drawPidChart(document.getElementById('spdRightChartCanvas'), [
    { name: '右目标', values: h.spd_right_target, color: '#fbbf24' },
    { name: '右反馈', values: h.spd_right_actual, color: '#38bdf8' },
  ], { yMin: -100, yMax: 600, title: '右轮速度环 (counts/5ms)' });
}
```

---

### Task 5: 自适应 Y 轴范围

- [ ] **Step 10: 实现自动 Y 轴缩放函数**

在 `renderPidCharts` 之前添加自适应逻辑，每 50 个采样点重新计算一次 Y 轴范围，避免数据超出显示区域。

```javascript
let pidChartAutoRangeCounter = 0;
const pidChartRanges = {
  pos:  { min: -40, max: 40 },
  yaw:  { min: -200, max: 200 },
  spdL: { min: -50, max: 300 },
  spdR: { min: -50, max: 300 },
};

function updateAutoRange(series1, series2, range, margin) {
  const total = pidHistory.count;
  if (total < 2) return;
  const cap = PID_CHART_RING_CAPACITY;
  const head = pidHistory.head;
  let vmin = Infinity, vmax = -Infinity;
  for (let j = 0; j < total; j++) {
    const idx = (head - total + j + cap) % cap;
    const a = series1[idx], b = series2[idx];
    if (Number.isFinite(a)) { if (a < vmin) vmin = a; if (a > vmax) vmax = a; }
    if (Number.isFinite(b)) { if (b < vmin) vmin = b; if (b > vmax) vmax = b; }
  }
  if (!isFinite(vmin) || !isFinite(vmax)) return;
  const pad = Math.max((vmax - vmin) * (margin || 0.15), 1);
  range.min = vmin - pad;
  range.max = vmax + pad;
}
```

在 `renderPidCharts` 开头调用：

```javascript
pidChartAutoRangeCounter++;
if (pidChartAutoRangeCounter % 50 === 1) {
  updateAutoRange(pidHistory.pos_target, pidHistory.pos_actual, pidChartRanges.pos);
  updateAutoRange(pidHistory.yaw_target, pidHistory.yaw_actual, pidChartRanges.yaw);
  updateAutoRange(pidHistory.spd_left_target, pidHistory.spd_left_actual, pidChartRanges.spdL);
  updateAutoRange(pidHistory.spd_right_target, pidHistory.spd_right_actual, pidChartRanges.spdR);
}
```

然后将 `drawPidChart` 的 `yMin`/`yMax` 改为使用 `pidChartRanges` 的值。

---

### Task 6: 面板折叠时暂停绘制优化

- [ ] **Step 11: 折叠检测**

在 `renderPidCharts` 开头添加折叠检测，折叠的 panel 跳过绘制：

```javascript
function renderPidCharts() {
  if (document.getElementById('posChartPanel').open) {
    drawPidChart(/* pos chart */);
  }
  if (document.getElementById('yawChartPanel').open) {
    drawPidChart(/* yaw chart */);
  }
  if (document.getElementById('spdChartPanel').open) {
    drawPidChart(/* spd left */);
    drawPidChart(/* spd right */);
  }
}
```

---

### Task 7: 验证

- [ ] **Step 12: 启动 mock 发送器验证数据流**

```bash
cd tools/pc_receiver_js
node scripts/mock_board_sender.js
```

- [ ] **Step 13: 启动接收端服务器并打开浏览器**

```bash
node server.js
# 浏览器打开 http://localhost:8080
```

验证点：
1. 三个新面板出现在"历史角点"栏下方
2. 折线图随时间滚动，新数据从右侧进入
3. 位置环目标值线在 0 附近，实际值线随 mock 数据波动
4. 角速度环两条线随 mock 数据变化
5. 速度环左右轮各自显示目标/反馈两条线
6. 折叠面板时图表暂停更新（性能优化）
7. Y 轴范围每 50 个点自动调整

- [ ] **Step 14: 检查回放模式兼容性**

在回放模式下，`pullStatus()` 被跳过（line 5666: `if (replayMode) return;`），所以图表在回放时不会更新——这是预期行为，因为回放有自己的状态渲染路径。如果需要回放支持，后续单独处理。

- [ ] **Step 15: Commit**

```bash
git add tools/pc_receiver_js/public/index.html
git commit -m "feat: add PID cascade real-time charts (position/yaw/speed loops)

Add VOFA-style scrolling line charts for all three PID loops below
the history corner ROI section. Position and yaw rate loops show
target vs actual traces. Speed loop shows left/right wheel target
vs feedback in two sub-charts. All buffering and rendering is
client-side with a 300-point ring buffer."
```
