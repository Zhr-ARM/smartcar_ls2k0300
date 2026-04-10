(() => {
  const WEB_DATA_PROFILE_FULL = 0;
  const WEB_DATA_PROFILE_RAW_MINIMAL = 1;

  function imageEndpointByMode(mode) {
    if (mode === 'rgb') return '/api/frame_rgb.jpg';
    if (mode === 'binary') return '/api/frame_binary.jpg';
    return '/api/frame_gray.jpg';
  }

  function frameUrlForMode(mode) {
    return `${imageEndpointByMode(mode)}?t=${Date.now()}`;
  }

  async function fetchJsonNoStore(url) {
    const response = await fetch(url, { cache: 'no-store' });
    if (!response.ok) {
      throw new Error(`${url} -> ${response.status}`);
    }
    return response.json();
  }

  function formatValue(value) {
    if (typeof value === 'number') {
      return Number.isFinite(value) ? value.toFixed(3) : String(value);
    }
    if (value === undefined || value === null) {
      return 'N/A';
    }
    return String(value);
  }

  function renderStatusList(container, rows) {
    if (!container) return;
    container.innerHTML = '';
    for (const [name, value] of rows) {
      const item = document.createElement('div');
      item.className = 'status-item';
      item.innerHTML = `<strong>${name}</strong>: ${formatValue(value)}`;
      container.appendChild(item);
    }
  }

  function hasValue(value) {
    return value !== undefined && value !== null;
  }

  function isFullDebugProfile(profile) {
    return Number(profile) === WEB_DATA_PROFILE_FULL;
  }

  function isRawMinimalProfile(profile) {
    return Number(profile) === WEB_DATA_PROFILE_RAW_MINIMAL;
  }

  function setElementVisible(el, visible) {
    if (!el) return;
    el.style.display = visible ? '' : 'none';
  }

  function frameSlotReady(slot) {
    return !!(slot && slot.frameId >= 0 && slot.width > 0 && slot.height > 0);
  }

  function formatArrayInline(arr) {
    if (!Array.isArray(arr)) return '[]';
    return `[${arr.map((v) => Array.isArray(v) ? `[${v.join(', ')}]` : formatValue(v)).join(', ')}]`;
  }

  function formatTrackMethod(method) {
    if (method === 0) return 'fixed';
    if (method === 1) return 'weighted';
    if (method === 2) return 'speed';
    return `unknown(${method})`;
  }

  function formatCenterlineSource(source) {
    if (source === 0) return 'left_shift';
    if (source === 1) return 'right_shift';
    if (source === 2) return 'auto';
    return `unknown(${source})`;
  }

  function formatCenterlineSourceLabel(source) {
    if (source === 0) return '透视中线来源：自动选择（左边界平移）';
    if (source === 1) return '透视中线来源：自动选择（右边界平移）';
    if (source === 2) return '透视中线来源：无偏好（待本帧自动选择）';
    return '透视中线来源：自动选择（未知）';
  }

  function formatRouteMainState(state) {
    if (state === 0) return '正常巡线';
    if (state === 1) return '左环岛';
    if (state === 2) return '右环岛';
    if (state === 3) return '交叉口';
    return `未知主状态(${state})`;
  }

  function formatRouteSubState(state) {
    if (state === 0) return '无子状态';
    if (state === 1) return '十字-准备进入';
    if (state === 2) return '十字-穿越中';
    if (state === 3) return '左环岛-开始';
    if (state === 4) return '左环岛-入环';
    if (state === 5) return '左环岛-运行';
    if (state === 6) return '左环岛-出环';
    if (state === 7) return '左环岛-结束';
    if (state === 8) return '右环岛-开始';
    if (state === 9) return '右环岛-入环';
    if (state === 10) return '右环岛-运行';
    if (state === 11) return '右环岛-出环';
    if (state === 12) return '右环岛-结束';
    return `未知子状态(${state})`;
  }

  function formatRoutePreferredSource(source) {
    if (source === -1) return '自动';
    if (source === 0) return '左边界';
    if (source === 1) return '右边界';
    return `未知(${source})`;
  }

  function pointDistance(pointA, pointB) {
    if (!Array.isArray(pointA) || !Array.isArray(pointB) || pointA.length < 2 || pointB.length < 2) return null;
    const ax = Number(pointA[0]);
    const ay = Number(pointA[1]);
    const bx = Number(pointB[0]);
    const by = Number(pointB[1]);
    if (!Number.isFinite(ax) || !Number.isFinite(ay) || !Number.isFinite(bx) || !Number.isFinite(by)) return null;
    return Math.hypot(ax - bx, ay - by);
  }

  function buildRouteStateSummary(status) {
    const mainStateRaw = Number(status && status.route_main_state);
    const subStateRaw = Number(status && status.route_sub_state);
    const preferredSourceRaw = Number(status && status.route_preferred_source);
    const encoderSinceEnterRaw = Number(status && status.route_encoder_since_enter);
    const crossLossCountRaw = Number(status && status.route_cross_loss_count);
    const leftLossCountRaw = Number(status && status.route_left_loss_count);
    const leftGainCountRaw = Number(status && status.route_left_gain_count);
    const rightLossCountRaw = Number(status && status.route_right_loss_count);
    const rightGainCountRaw = Number(status && status.route_right_gain_count);
    const mainState = Number.isFinite(mainStateRaw) ? mainStateRaw : null;
    const subState = Number.isFinite(subStateRaw) ? subStateRaw : null;
    const preferredSource = Number.isFinite(preferredSourceRaw) ? preferredSourceRaw : null;
    const encoderSinceEnter = Number.isFinite(encoderSinceEnterRaw) ? encoderSinceEnterRaw : null;
    const crossLossCount = Number.isFinite(crossLossCountRaw) ? crossLossCountRaw : null;
    const leftLossCount = Number.isFinite(leftLossCountRaw) ? leftLossCountRaw : null;
    const leftGainCount = Number.isFinite(leftGainCountRaw) ? leftGainCountRaw : null;
    const rightLossCount = Number.isFinite(rightLossCountRaw) ? rightLossCountRaw : null;
    const rightGainCount = Number.isFinite(rightGainCountRaw) ? rightGainCountRaw : null;
    const leftCornerFound = !!(status && status.cross_lower_left_corner_found);
    const rightCornerFound = !!(status && status.cross_lower_right_corner_found);
    const leftStraight = !leftCornerFound && !toBool(status && status.src_left_trace_has_frame_wall);
    const rightStraight = !rightCornerFound && !toBool(status && status.src_right_trace_has_frame_wall);
    const leftCornerPoint = Array.isArray(status && status.cross_lower_left_corner_point) ? status.cross_lower_left_corner_point : null;
    const rightCornerPoint = Array.isArray(status && status.cross_lower_right_corner_point) ? status.cross_lower_right_corner_point : null;
    const cornerDistance = pointDistance(leftCornerPoint, rightCornerPoint);
    const leftEntryFeature = toBool(status && status.src_circle_left_entry);
    const rightEntryFeature = toBool(status && status.src_circle_right_entry);
    const exitClearFeature = toBool(status && status.src_circle_exit_clear);
    const leftHasFrameWall = toBool(status && status.src_left_trace_has_frame_wall);
    const rightHasFrameWall = toBool(status && status.src_right_trace_has_frame_wall);

    function toBool(value) {
      if (value === true || value === false) return value;
      const n = Number(value);
      return Number.isFinite(n) ? n !== 0 : !!value;
    }

    const clues = [];
    clues.push(`原图角点：左=${leftCornerFound ? '有' : '无'}，右=${rightCornerFound ? '有' : '无'}`);
    clues.push(`边框墙：左=${leftHasFrameWall ? '有' : '无'}，右=${rightHasFrameWall ? '有' : '无'}`);
    clues.push(`入口特征：左=${leftEntryFeature ? '命中' : '未命中'}，右=${rightEntryFeature ? '命中' : '未命中'}`);
    clues.push(`出环清空：${exitClearFeature ? '命中' : '未命中'}`);
    if (cornerDistance !== null) {
      clues.push(`双角点距离：${cornerDistance.toFixed(1)} px`);
    }

    const judgments = [];
    let nextStateLabel = '保持当前状态';
    const nextReasons = [];
    const conditionLines = [];

    const pushCondition = (label, value, detail) => {
      conditionLines.push({
        label,
        value,
        detail
      });
    };

    if (mainState === 0) {
      if (leftCornerFound && rightCornerFound) {
        judgments.push(cornerDistance !== null && cornerDistance < 40
          ? '双角点都存在且距离已接近交叉阈值，当前更接近交叉口判定。'
          : '双角点都存在，但距离还未压到交叉阈值，当前仍保持正常巡线。');
      } else if (leftCornerFound && rightStraight) {
        judgments.push('左侧出现角点、右侧仍保持直边，当前更像左环岛进入前的候选状态。');
      } else if (rightCornerFound && leftStraight) {
        judgments.push('右侧出现角点、左侧仍保持直边，当前更像右环岛进入前的候选状态。');
      } else {
        judgments.push('当前没有同时触发角点与直边的强切换条件，状态机继续留在正常巡线。');
      }

      pushCondition('左环入口特征', leftEntryFeature ? '命中' : '未命中', '单侧角点 + 同侧连续边框墙 + 对侧局部无边框墙');
      pushCondition('右环入口特征', rightEntryFeature ? '命中' : '未命中', '单侧角点 + 同侧连续边框墙 + 对侧局部无边框墙');
      pushCondition('双角点交叉候选', (leftCornerFound && rightCornerFound && cornerDistance !== null && cornerDistance < 40) ? '命中' : '未命中', '双侧都检测到角点且距离 < 40');

      if (leftEntryFeature) {
        nextStateLabel = '左环岛-开始';
        nextReasons.push('当前滑窗累计左环入口命中时，下一步会进入左环岛开始态。');
      } else if (rightEntryFeature) {
        nextStateLabel = '右环岛-开始';
        nextReasons.push('当前滑窗累计右环入口命中时，下一步会进入右环岛开始态。');
      } else if (leftCornerFound && rightCornerFound && cornerDistance !== null && cornerDistance < 40) {
        nextStateLabel = '十字-准备进入';
        nextReasons.push('双角点距离已压到交叉阈值内。');
      } else {
        nextReasons.push('当前入口特征未形成稳定命中，保持正常巡线。');
      }
    } else if (mainState === 3) {
      if (subState === 1) {
        judgments.push('当前处于 cross_begin：等待任一侧角点原图 y 超过阈值后，下一帧切入 cross_in。');
        nextStateLabel = '十字-穿越中';
        nextReasons.push('任一侧角点 y 超过 45 后，从 cross_begin 切到 cross_in。');
        pushCondition('左角点 y > 45', Number(status && status.cross_lower_left_corner_point && status.cross_lower_left_corner_point[1]) > 45 ? '命中' : '未命中', `当前 y=${Array.isArray(status && status.cross_lower_left_corner_point) ? status.cross_lower_left_corner_point[1] : '--'}`);
        pushCondition('右角点 y > 45', Number(status && status.cross_lower_right_corner_point && status.cross_lower_right_corner_point[1]) > 45 ? '命中' : '未命中', `当前 y=${Array.isArray(status && status.cross_lower_right_corner_point) ? status.cross_lower_right_corner_point[1] : '--'}`);
      } else if (subState === 2) {
        judgments.push(`当前处于 cross_in：正在按保存起始行向下搜线，当前探测停止行=${crossLossCount !== null ? crossLossCount : -1}（>=80 将退出）。`);
        nextStateLabel = (crossLossCount !== null && crossLossCount >= 80) ? '正常巡线' : '十字-穿越中';
        nextReasons.push('cross_detected_stop_row >= 80 时退出十字。');
        pushCondition('cross_detected_stop_row >= 80', (crossLossCount !== null && crossLossCount >= 80) ? '命中' : '未命中', `当前=${crossLossCount !== null ? crossLossCount : '--'}`);
      } else {
        judgments.push(`交叉口状态下，当前探测停止行为 ${crossLossCount !== null ? crossLossCount : -1}，用于判断何时退出交叉口。`);
        nextReasons.push('等待 cross_begin / cross_in 细分判据推进。');
      }
    } else if (mainState === 1) {
      judgments.push(`左环岛状态下，优先使用 ${formatRoutePreferredSource(preferredSource)} 构造中线。`);
      pushCondition('左侧丢线计数', leftLossCount !== null ? leftLossCount : '--', 'BEGIN 阶段需要先累计丢线');
      pushCondition('左侧恢复计数', leftGainCount !== null ? leftGainCount : '--', 'BEGIN 阶段丢线后重新看到左边界');
      pushCondition('出环清空特征', exitClearFeature ? '命中' : '未命中', '双侧无角点且双侧无边框墙');
      if (subState === 3) {
        nextStateLabel = '左环岛-入环';
        nextReasons.push('左侧先丢线，再恢复到最少可见点数后，进入入环阶段。');
      } else if (subState === 4) {
        nextStateLabel = '左环岛-运行';
        nextReasons.push('编码器累计超过 40000 后切到运行阶段。');
        pushCondition('encoder > 40000', encoderSinceEnter !== null && encoderSinceEnter > 40000 ? '命中' : '未命中', `当前=${encoderSinceEnter !== null ? encoderSinceEnter : '--'}`);
      } else if (subState === 5) {
        nextStateLabel = '左环岛-出环';
        nextReasons.push('出环清空特征连续命中后切到出环阶段。');
      } else if (subState === 6) {
        nextStateLabel = '左环岛-结束';
        nextReasons.push('出环清空特征成立时切到结束阶段。');
      } else if (subState === 7) {
        nextStateLabel = '正常巡线';
        nextReasons.push('结束阶段继续命中出环清空特征时回到正常巡线。');
      } else {
        nextReasons.push('等待左环内部子状态推进。');
      }
    } else if (mainState === 2) {
      judgments.push(`右环岛状态下，优先使用 ${formatRoutePreferredSource(preferredSource)} 构造中线。`);
      pushCondition('右侧丢线计数', rightLossCount !== null ? rightLossCount : '--', 'BEGIN 阶段需要先累计丢线');
      pushCondition('右侧恢复计数', rightGainCount !== null ? rightGainCount : '--', 'BEGIN 阶段丢线后重新看到右边界');
      pushCondition('出环清空特征', exitClearFeature ? '命中' : '未命中', '双侧无角点且双侧无边框墙');
      if (subState === 8) {
        nextStateLabel = '右环岛-入环';
        nextReasons.push('右侧先丢线，再恢复到最少可见点数后，进入入环阶段。');
      } else if (subState === 9) {
        nextStateLabel = '右环岛-运行';
        nextReasons.push('编码器累计超过 40000 后切到运行阶段。');
        pushCondition('encoder > 40000', encoderSinceEnter !== null && encoderSinceEnter > 40000 ? '命中' : '未命中', `当前=${encoderSinceEnter !== null ? encoderSinceEnter : '--'}`);
      } else if (subState === 10) {
        nextStateLabel = '右环岛-出环';
        nextReasons.push('出环清空特征连续命中后切到出环阶段。');
      } else if (subState === 11) {
        nextStateLabel = '右环岛-结束';
        nextReasons.push('出环清空特征成立时切到结束阶段。');
      } else if (subState === 12) {
        nextStateLabel = '正常巡线';
        nextReasons.push('结束阶段继续命中出环清空特征时回到正常巡线。');
      } else {
        nextReasons.push('等待右环内部子状态推进。');
      }
    } else {
      judgments.push('等待状态机首次更新。');
      nextReasons.push('等待有效状态数据。');
    }

    const counters = [
      ['交叉停止行', Number.isFinite(crossLossCount) ? crossLossCount : null],
      ['左侧丢线', Number.isFinite(leftLossCount) ? leftLossCount : null],
      ['左侧恢复', Number.isFinite(leftGainCount) ? leftGainCount : null],
      ['右侧丢线', Number.isFinite(rightLossCount) ? rightLossCount : null],
      ['右侧恢复', Number.isFinite(rightGainCount) ? rightGainCount : null]
    ].filter(([, value]) => value !== null);

    return {
      mainState,
      subState,
      preferredSource,
      encoderSinceEnter,
      mainStateLabel: mainState !== null ? formatRouteMainState(mainState) : '--',
      subStateLabel: subState !== null ? formatRouteSubState(subState) : '--',
      preferredSourceLabel: preferredSource !== null ? formatRoutePreferredSource(preferredSource) : '--',
      crossLossCount,
      leftLossCount,
      leftGainCount,
      rightLossCount,
      rightGainCount,
      cornerDistance,
      nextStateLabel,
      nextReasons,
      conditionLines,
      clues,
      judgments,
      counters
    };
  }

  function drawCurveChartToCanvas(canvas, ctx, rawValues, options = {}) {
    const w = canvas.width;
    const h = canvas.height;
    ctx.fillStyle = options.backgroundColor || '#020617';
    ctx.fillRect(0, 0, w, h);

    const minY = Number(options.yMin);
    const maxY = Number(options.yMax);
    if (!Number.isFinite(minY) || !Number.isFinite(maxY) || minY >= maxY) return false;
    const series = Array.isArray(rawValues) ? rawValues : [];

    const marginLeft = 36;
    const marginRight = 10;
    const marginTop = 10;
    const marginBottom = 24;
    const plotW = Math.max(1, w - marginLeft - marginRight);
    const plotH = Math.max(1, h - marginTop - marginBottom);

    const yAt = (v) => marginTop + ((maxY - v) * plotH) / (maxY - minY);
    const yZero = yAt(0);

    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(marginLeft, marginTop);
    ctx.lineTo(marginLeft, marginTop + plotH);
    ctx.lineTo(marginLeft + plotW, marginTop + plotH);
    ctx.stroke();

    ctx.strokeStyle = '#64748b';
    ctx.beginPath();
    ctx.moveTo(marginLeft, yZero);
    ctx.lineTo(marginLeft + plotW, yZero);
    ctx.stroke();

    const divs = 5;
    ctx.setLineDash([3, 3]);
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.35)';
    ctx.lineWidth = 1;
    for (let i = 0; i <= divs; i += 1) {
      const t = i / divs;
      const gx = marginLeft + t * plotW;
      const gy = marginTop + t * plotH;

      ctx.beginPath();
      ctx.moveTo(gx, marginTop);
      ctx.lineTo(gx, marginTop + plotH);
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(marginLeft, gy);
      ctx.lineTo(marginLeft + plotW, gy);
      ctx.stroke();
    }
    ctx.setLineDash([]);

    ctx.fillStyle = '#94a3b8';
    for (let i = 0; i <= divs; i += 1) {
      const t = i / divs;
      const gx = marginLeft + t * plotW;
      const gy = marginTop + t * plotH;

      ctx.beginPath();
      ctx.arc(gx, marginTop + plotH, 2, 0, Math.PI * 2);
      ctx.fill();

      ctx.beginPath();
      ctx.arc(marginLeft, gy, 2, 0, Math.PI * 2);
      ctx.fill();
    }

    ctx.fillStyle = '#94a3b8';
    ctx.font = '12px "Noto Sans SC", "Microsoft YaHei", sans-serif';
    ctx.fillText('0', marginLeft - 10, marginTop + plotH + 16);
    ctx.fillText(series.length > 0 ? String(series.length - 1) : 'N/A', marginLeft + plotW - 28, marginTop + plotH + 16);
    ctx.fillText(maxY.toFixed(3), 6, marginTop + 8);
    ctx.fillText('0', 12, yZero + 4);
    ctx.fillText(minY.toFixed(3), 2, marginTop + plotH);

    if (series.length < 1) {
      ctx.fillStyle = '#64748b';
      ctx.fillText('No data', marginLeft + plotW * 0.5 - 20, marginTop + plotH * 0.5);
      return false;
    }

    const values = series.map((v) => Math.max(minY, Math.min(maxY, Number(v) || 0)));
    const xAt = (idx) => marginLeft + (idx * plotW) / Math.max(1, values.length - 1);

    ctx.strokeStyle = options.lineColor || '#38bdf8';
    ctx.lineWidth = options.lineWidth || 1.5;
    ctx.beginPath();
    for (let i = 0; i < values.length; i += 1) {
      const x = xAt(i);
      const y = yAt(Number(values[i]) || 0);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    return true;
  }

  function normalizePointSeries(pts) {
    if (!Array.isArray(pts)) return [];
    const normalized = [];
    let hasNonZeroPoint = false;
    for (const p of pts) {
      if (!Array.isArray(p) || p.length < 2) continue;
      const x = Number(p[0] || 0);
      const y = Number(p[1] || 0);
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      if (x !== 0 || y !== 0) hasNonZeroPoint = true;
      normalized.push([x, y]);
    }
    if (!hasNonZeroPoint) return normalized;
    return normalized.filter(([x, y]) => x !== 0 || y !== 0);
  }

  function drawPolyline(ctx, pts, color, lineWidth) {
    const series = normalizePointSeries(pts);
    if (series.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.beginPath();
    ctx.moveTo(series[0][0], series[0][1]);
    for (let i = 1; i < series.length; i += 1) {
      ctx.lineTo(series[i][0], series[i][1]);
    }
    ctx.stroke();
  }

  function drawPointSet(ctx, pts, color, radius) {
    const series = normalizePointSeries(pts);
    if (series.length < 1) return;
    ctx.fillStyle = color;
    const r = Math.max(1, radius | 0);
    for (let i = 0; i < series.length; i += 1) {
      const p = series[i];
      ctx.beginPath();
      ctx.arc(p[0], p[1], r, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  function drawSeries(ctx, pts, color, lineWidth, pointRadius, pointMode) {
    if (pointMode) {
      drawPointSet(ctx, pts, color, pointRadius);
    } else {
      drawPolyline(ctx, pts, color, lineWidth);
    }
  }

  function getCanvasPoint(ev, canvas) {
    const rect = canvas.getBoundingClientRect();
    const sx = canvas.width / rect.width;
    const sy = canvas.height / rect.height;
    const x = Math.max(0, Math.min(canvas.width - 1, Math.floor((ev.clientX - rect.left) * sx)));
    const y = Math.max(0, Math.min(canvas.height - 1, Math.floor((ev.clientY - rect.top) * sy)));
    return [x, y];
  }

  function sampleRGB(ctx, x, y) {
    const d = ctx.getImageData(x, y, 1, 1).data;
    return [d[0] | 0, d[1] | 0, d[2] | 0];
  }

  function mapCoord(x, y, srcW, srcH, dstW, dstH) {
    if (srcW <= 1 || srcH <= 1 || dstW <= 0 || dstH <= 0) return [0, 0];
    const mx = Math.max(0, Math.min(dstW - 1, Math.round((x * (dstW - 1)) / (srcW - 1))));
    const my = Math.max(0, Math.min(dstH - 1, Math.round((y * (dstH - 1)) / (srcH - 1))));
    return [mx, my];
  }

  function bindPixelProbe(canvas, ctx, infoEl, callbacks = {}) {
    const buildText = callbacks.buildText;
    const resetText = callbacks.resetText || 'x:- y:- | Gray:- Bin:- RGB:(-,-,-)';
    canvas.addEventListener('mousemove', (ev) => {
      const [x, y] = getCanvasPoint(ev, canvas);
      const text = typeof buildText === 'function' ? buildText(x, y, canvas, ctx) : `x:${x} y:${y}`;
      infoEl.textContent = text;
    });
    canvas.addEventListener('mouseleave', () => {
      infoEl.textContent = resetText;
    });
  }

  function buildStatusSummary(status, helpers = {}) {
    const lines = [];
    const getSelectedIpmCenterline = helpers.getSelectedIpmCenterline || (() => []);
    const getLeftBoundaryAngleCos = helpers.getLeftBoundaryAngleCos || (() => []);
    const getRightBoundaryAngleCos = helpers.getRightBoundaryAngleCos || (() => []);

    const push = (name, value) => {
      if (hasValue(value)) lines.push(`${name}: ${formatValue(value)}`);
    };
    const pushCount = (name, value) => {
      if (Array.isArray(value)) lines.push(`${name}: ${value.length}`);
    };

    push('web_data_profile', status.web_data_profile);
    push('line_error', status.line_error);
    push('ts_ms', status.ts_ms);
    push('udp_web_max_fps', status.udp_web_max_fps);
    push('udp_web_send_gray', status.udp_web_send_gray);
    push('udp_web_send_binary', status.udp_web_send_binary);
    push('udp_web_send_rgb', status.udp_web_send_rgb);
    push('udp_web_gray_image_format', status.udp_web_gray_image_format);
    push('udp_web_binary_image_format', status.udp_web_binary_image_format);
    push('udp_web_rgb_image_format', status.udp_web_rgb_image_format);
    push('base_speed', status.base_speed);
    push('adjusted_base_speed', status.adjusted_base_speed);
    push('left_target_count', status.left_target_count);
    push('right_target_count', status.right_target_count);
    push('left_current_count', status.left_current_count);
    push('right_current_count', status.right_current_count);
    push('left_filtered_count', status.left_filtered_count);
    push('right_filtered_count', status.right_filtered_count);
    push('left_error', status.left_error);
    push('right_error', status.right_error);
    push('left_feedforward', status.left_feedforward);
    push('right_feedforward', status.right_feedforward);
    push('left_correction', status.left_correction);
    push('right_correction', status.right_correction);
    push('left_decel_assist', status.left_decel_assist);
    push('right_decel_assist', status.right_decel_assist);
    push('left_duty', status.left_duty);
    push('right_duty', status.right_duty);
    push('left_hardware_duty', status.left_hardware_duty);
    push('right_hardware_duty', status.right_hardware_duty);
    push('left_dir_level', status.left_dir_level);
    push('right_dir_level', status.right_dir_level);
    push('ipm_weighted_decision_point_error', status.ipm_weighted_first_point_error);
    if (Array.isArray(status.ipm_weighted_decision_point)) push('ipm_weighted_decision_point', formatArrayInline(status.ipm_weighted_decision_point));
    if (Array.isArray(status.src_weighted_decision_point)) push('src_weighted_decision_point', formatArrayInline(status.src_weighted_decision_point));
    push('otsu_threshold', status.otsu_threshold);
    push('capture_wait_us', status.capture_wait_us);
    push('preprocess_us', status.preprocess_us);
    push('otsu_us', status.otsu_us);
    push('maze_us', status.maze_us);
    push('total_us', status.total_us);
    push('rx_udp_bytes_per_sec', status.rx_udp_bytes_per_sec);
    push('rx_udp_kib_per_sec', status.rx_udp_kib_per_sec);
    push('rx_udp_mbps', status.rx_udp_mbps);
    push('rx_udp_frames_per_sec', status.rx_udp_frames_per_sec);
    push('rx_udp_gray_fps', status.rx_udp_gray_fps);
    push('rx_udp_binary_fps', status.rx_udp_binary_fps);
    push('rx_udp_rgb_fps', status.rx_udp_rgb_fps);
    push('rx_udp_avg_gray_frame_bytes', status.rx_udp_avg_gray_frame_bytes);
    push('rx_udp_avg_binary_frame_bytes', status.rx_udp_avg_binary_frame_bytes);
    push('rx_udp_avg_rgb_frame_bytes', status.rx_udp_avg_rgb_frame_bytes);
    push('rx_udp_estimated_peak_bytes_per_sec', status.rx_udp_estimated_peak_bytes_per_sec);
    push('rx_udp_estimated_peak_kib_per_sec', status.rx_udp_estimated_peak_kib_per_sec);
    push('rx_udp_estimated_peak_mbps', status.rx_udp_estimated_peak_mbps);
    push('maze_left_points_raw', status.maze_left_points_raw);
    push('maze_right_points_raw', status.maze_right_points_raw);
    push('red_found', status.red_found);
    if (Array.isArray(status.red)) push('red', formatArrayInline(status.red));
    push('roi_valid', status.roi_valid);
    if (Array.isArray(status.roi)) push('roi', formatArrayInline(status.roi));
    push('ipm_track_valid', status.ipm_track_valid);
    if (hasValue(status.ipm_track_method)) push('ipm_track_method', formatTrackMethod(status.ipm_track_method));
    if (hasValue(status.ipm_centerline_source)) push('ipm_centerline_source', formatCenterlineSource(status.ipm_centerline_source));
    if (Number.isFinite(Number(status.route_main_state))) push('route_main_state', formatRouteMainState(Number(status.route_main_state)));
    if (Number.isFinite(Number(status.route_sub_state))) push('route_sub_state', formatRouteSubState(Number(status.route_sub_state)));
    if (Number.isFinite(Number(status.route_preferred_source))) push('route_preferred_source', formatRoutePreferredSource(Number(status.route_preferred_source)));
    if (Number.isFinite(Number(status.route_encoder_since_enter))) push('route_encoder_since_enter', status.route_encoder_since_enter);
    if (Number.isFinite(Number(status.route_cross_loss_count))) push('route_cross_loss_count', status.route_cross_loss_count);
    if (Number.isFinite(Number(status.route_left_loss_count))) push('route_left_loss_count', status.route_left_loss_count);
    if (Number.isFinite(Number(status.route_left_gain_count))) push('route_left_gain_count', status.route_left_gain_count);
    if (Number.isFinite(Number(status.route_right_loss_count))) push('route_right_loss_count', status.route_right_loss_count);
    if (Number.isFinite(Number(status.route_right_gain_count))) push('route_right_gain_count', status.route_right_gain_count);
    push('ipm_track_index', status.ipm_track_index);
    if (Array.isArray(status.ipm_track_point)) push('ipm_track_point', formatArrayInline(status.ipm_track_point));
    push('eight_left_first_frame_touch_index', status.eight_left_first_frame_touch_index);
    push('eight_right_first_frame_touch_index', status.eight_right_first_frame_touch_index);
    if (Array.isArray(status.eight_left_first_frame_touch_point)) push('eight_left_first_frame_touch_point', formatArrayInline(status.eight_left_first_frame_touch_point));
    if (Array.isArray(status.eight_right_first_frame_touch_point)) push('eight_right_first_frame_touch_point', formatArrayInline(status.eight_right_first_frame_touch_point));
    push('left_boundary_corner_found', status.left_boundary_corner_found);
    push('right_boundary_corner_found', status.right_boundary_corner_found);
    if (Array.isArray(status.left_boundary_corner_point)) push('left_boundary_corner_point', formatArrayInline(status.left_boundary_corner_point));
    if (Array.isArray(status.right_boundary_corner_point)) push('right_boundary_corner_point', formatArrayInline(status.right_boundary_corner_point));
    push('ipm_left_boundary_corner_found', status.ipm_left_boundary_corner_found);
    push('ipm_right_boundary_corner_found', status.ipm_right_boundary_corner_found);
    push('ipm_left_boundary_corner_index', status.ipm_left_boundary_corner_index);
    push('ipm_right_boundary_corner_index', status.ipm_right_boundary_corner_index);
    push('ipm_left_boundary_straight', status.ipm_left_boundary_straight);
    push('ipm_right_boundary_straight', status.ipm_right_boundary_straight);
    if (Array.isArray(status.ipm_left_boundary_corner_point)) push('ipm_left_boundary_corner_point', formatArrayInline(status.ipm_left_boundary_corner_point));
    if (Array.isArray(status.ipm_right_boundary_corner_point)) push('ipm_right_boundary_corner_point', formatArrayInline(status.ipm_right_boundary_corner_point));
    push('cross_lower_corner_dir_enabled', status.cross_lower_corner_dir_enabled);
    push('cross_lower_corner_pair_valid', status.cross_lower_corner_pair_valid);
    push('cross_lower_left_corner_found', status.cross_lower_left_corner_found);
    push('cross_lower_right_corner_found', status.cross_lower_right_corner_found);
    push('cross_lower_left_corner_index', status.cross_lower_left_corner_index);
    push('cross_lower_right_corner_index', status.cross_lower_right_corner_index);
    if (Array.isArray(status.cross_lower_left_corner_point)) push('cross_lower_left_corner_point', formatArrayInline(status.cross_lower_left_corner_point));
    if (Array.isArray(status.cross_lower_right_corner_point)) push('cross_lower_right_corner_point', formatArrayInline(status.cross_lower_right_corner_point));
    push('cross_lower_corner_pre_window', status.cross_lower_corner_pre_window);
    push('cross_lower_corner_post_window', status.cross_lower_corner_post_window);
    push('cross_lower_corner_pre_min_votes', status.cross_lower_corner_pre_min_votes);
    push('cross_lower_corner_post_min_votes', status.cross_lower_corner_post_min_votes);
    push('cross_lower_corner_transition_max_len', status.cross_lower_corner_transition_max_len);
    push('cross_lower_corner_transition_max_dir3_count', status.cross_lower_corner_transition_max_dir3_count);
    push('cross_lower_corner_post_max_dir3_count', status.cross_lower_corner_post_max_dir3_count);
    push('cross_lower_corner_pair_y_diff_max', status.cross_lower_corner_pair_y_diff_max);
    if (Array.isArray(status.gray_size)) push('gray_size', formatArrayInline(status.gray_size));
    if (Array.isArray(status.ipm_size)) push('ipm_size', formatArrayInline(status.ipm_size));
    pushCount('left_boundary_count', status.left_boundary);
    pushCount('right_boundary_count', status.right_boundary);
    pushCount('eight_left_trace_count', status.eight_left_trace);
    pushCount('eight_right_trace_count', status.eight_right_trace);
    pushCount('eight_left_trace_dir_count', status.eight_left_trace_dir);
    pushCount('eight_right_trace_dir_count', status.eight_right_trace_dir);
    pushCount('left_boundary_corner_count', status.left_boundary_corner);
    pushCount('right_boundary_corner_count', status.right_boundary_corner);
    pushCount('ipm_left_boundary_count', status.ipm_left_boundary);
    pushCount('ipm_right_boundary_count', status.ipm_right_boundary);
    pushCount('ipm_left_boundary_corner_count', status.ipm_left_boundary_corner);
    pushCount('ipm_right_boundary_corner_count', status.ipm_right_boundary_corner);
    pushCount('ipm_raw_left_boundary_count', status.ipm_raw_left_boundary);
    pushCount('ipm_raw_right_boundary_count', status.ipm_raw_right_boundary);
    pushCount('ipm_centerline_selected_shift_count', getSelectedIpmCenterline(status));
    push('ipm_centerline_selected_count', status.ipm_centerline_selected_count);
    push('src_centerline_selected_count', status.src_centerline_selected_count);
    pushCount('ipm_left_boundary_angle_cos_count', getLeftBoundaryAngleCos(status));
    pushCount('ipm_right_boundary_angle_cos_count', getRightBoundaryAngleCos(status));

    return lines.length > 0 ? lines.join('\n') : 'waiting...';
  }

  window.SharedReceiverCore = {
    WEB_DATA_PROFILE_FULL,
    WEB_DATA_PROFILE_RAW_MINIMAL,
    imageEndpointByMode,
    frameUrlForMode,
    fetchJsonNoStore,
    formatValue,
    renderStatusList,
    hasValue,
    isFullDebugProfile,
    isRawMinimalProfile,
    setElementVisible,
    frameSlotReady,
    formatArrayInline,
    formatTrackMethod,
    formatCenterlineSource,
    formatCenterlineSourceLabel,
    formatRouteMainState,
    formatRouteSubState,
    formatRoutePreferredSource,
    buildRouteStateSummary,
    drawCurveChartToCanvas,
    drawPolyline,
    drawPointSet,
    drawSeries,
    normalizePointSeries,
    getCanvasPoint,
    sampleRGB,
    mapCoord,
    bindPixelProbe,
    buildStatusSummary
  };
})();
