(() => {
  const WEB_DATA_PROFILE_FULL = 0;
  const WEB_DATA_PROFILE_RAW_MINIMAL = 1;

  function imageEndpointByMode(mode) {
    if (mode === 'rgb') return '/api/frame_rgb.jpg';
    if (mode === 'binary') return '/api/frame_binary.jpg';
    if (mode === 'roi64') return '/api/frame_roi64.jpg';
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
    if (state === 1) return '直道态';
    if (state === 2) return '左环岛';
    if (state === 3) return '右环岛';
    if (state === 4) return '交叉口(保留)';
    return `未知主状态(${state})`;
  }

  function formatRouteSubState(state) {
    if (state === 0) return '无子状态';
    if (state === 1) return '十字-准备进入(保留)';
    if (state === 2) return '十字-穿越中(保留)';
    if (state === 3) return '左环岛-circle_1';
    if (state === 4) return '左环岛-circle_2';
    if (state === 5) return '左环岛-circle_3';
    if (state === 6) return '左环岛-circle_4';
    if (state === 7) return '左环岛-circle_5';
    if (state === 8) return '左环岛-circle_6';
    if (state === 9) return '右环岛-circle_1';
    if (state === 10) return '右环岛-circle_2';
    if (state === 11) return '右环岛-circle_3';
    if (state === 12) return '右环岛-circle_4';
    if (state === 13) return '右环岛-circle_5';
    if (state === 14) return '右环岛-circle_6';
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

  function toFiniteNumber(value) {
    const n = Number(value);
    return Number.isFinite(n) ? n : null;
  }

  function countPointSeries(series) {
    if (Array.isArray(series)) return series.length;
    return null;
  }

  function formatHit(hit) {
    return hit ? '命中' : '未命中';
  }

  function describeRouteSubState(subState) {
    if (subState === 3) return '入口观察阶段，等待左侧起始贴左边框累计达到进入阈值。';
    if (subState === 4) return '已贴左边框，等待左侧起始贴边消失，准备进入第一次补右线。';
    if (subState === 5) return '执行左环 stage3，右线由补线接管，等待右侧起始贴边连续增大。';
    if (subState === 6) return '等待右侧角点出现，确认已经推进到出环前半段。';
    if (subState === 7) return '执行左环 stage5，继续补右线，等待右侧重新连续贴右边框。';
    if (subState === 8) return '收尾阶段，搜线起始行抬高，等待双侧重新都成为直边。';
    if (subState === 9) return '入口观察阶段，等待右侧起始贴右边框累计达到进入阈值。';
    if (subState === 10) return '已贴右边框，等待右侧起始贴边消失，准备进入第一次补左线。';
    if (subState === 11) return '执行右环 stage3，左线由补线接管，等待左侧起始贴边连续增大。';
    if (subState === 12) return '等待左侧角点出现，确认已经推进到出环前半段。';
    if (subState === 13) return '执行右环 stage5，继续补左线，等待左侧重新连续贴左边框。';
    if (subState === 14) return '收尾阶段，搜线起始行抬高，等待双侧重新都成为直边。';
    if (subState === 1) return '十字子状态当前保留，主线未使用。';
    if (subState === 2) return '十字子状态当前保留，主线未使用。';
    return '普通巡线阶段。';
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
    const leftStraight = toBool(status && status.left_boundary_straight);
    const rightStraight = toBool(status && status.right_boundary_straight);
    const leftCornerPoint = Array.isArray(status && status.cross_lower_left_corner_point) ? status.cross_lower_left_corner_point : null;
    const rightCornerPoint = Array.isArray(status && status.cross_lower_right_corner_point) ? status.cross_lower_right_corner_point : null;
    const cornerDistance = pointDistance(leftCornerPoint, rightCornerPoint);
    const leftHasFrameWall = toBool(status && status.src_left_trace_has_frame_wall);
    const rightHasFrameWall = toBool(status && status.src_right_trace_has_frame_wall);
    const leftStartFrameRows = toFiniteNumber(status && status.left_start_frame_wall_rows);
    const rightStartFrameRows = toFiniteNumber(status && status.right_start_frame_wall_rows);
    const leftBoundaryCount = countPointSeries(status && status.left_boundary);
    const rightBoundaryCount = countPointSeries(status && status.right_boundary);
    const leftGuideCount = countPointSeries(status && status.left_circle_guide_line);
    const rightGuideCount = countPointSeries(status && status.right_circle_guide_line);
    const leftCornerIndex = toFiniteNumber(status && status.cross_lower_left_corner_index);
    const rightCornerIndex = toFiniteNumber(status && status.cross_lower_right_corner_index);
    const leftCornerY = leftCornerPoint && leftCornerPoint.length >= 2 ? toFiniteNumber(leftCornerPoint[1]) : null;
    const rightCornerY = rightCornerPoint && rightCornerPoint.length >= 2 ? toFiniteNumber(rightCornerPoint[1]) : null;
    const routeCircleDetectionEnabled = toBool(status && status.route_circle_detection_enabled);
    const circleEntryMinBoundaryCount = toFiniteNumber(status && status.route_circle_entry_min_boundary_count);
    const circleEntryCornerTailMargin = toFiniteNumber(status && status.route_circle_entry_corner_tail_margin);
    const circleStageEnterRows = toFiniteNumber(status && status.route_circle_stage_frame_wall_rows_enter);
    const circleStage3RowsTrigger = toFiniteNumber(status && status.route_circle_stage3_frame_wall_rows_trigger);
    const circleStage6MazeStartRow = toFiniteNumber(status && status.route_circle_stage6_maze_start_row);
    const circleGuideMinSegmentLen = toFiniteNumber(status && status.circle_guide_min_frame_wall_segment_len);
    const circleGuideTargetOffsetStage3 = toFiniteNumber(status && status.circle_guide_target_offset_stage3);
    const circleGuideAnchorOffsetStage5 = toFiniteNumber(status && status.circle_guide_anchor_offset_stage5);
    const leftCircleEntryHit =
      routeCircleDetectionEnabled &&
      rightStraight &&
      leftCornerFound &&
      leftCornerY !== null &&
      leftCornerY > 60 &&
      leftCornerIndex !== null &&
      rightBoundaryCount !== null &&
      circleEntryMinBoundaryCount !== null &&
      circleEntryCornerTailMargin !== null &&
      rightBoundaryCount > circleEntryMinBoundaryCount &&
      leftCornerIndex < (rightBoundaryCount - circleEntryCornerTailMargin);
    const rightCircleEntryHit =
      routeCircleDetectionEnabled &&
      leftStraight &&
      rightCornerFound &&
      rightCornerY !== null &&
      rightCornerY > 60 &&
      rightCornerIndex !== null &&
      leftBoundaryCount !== null &&
      circleEntryMinBoundaryCount !== null &&
      circleEntryCornerTailMargin !== null &&
      leftBoundaryCount > circleEntryMinBoundaryCount &&
      rightCornerIndex < (leftBoundaryCount - circleEntryCornerTailMargin);

    function toBool(value) {
      if (value === true || value === false) return value;
      const n = Number(value);
      return Number.isFinite(n) ? n !== 0 : !!value;
    }

    const clues = [];
    clues.push(`阶段说明：${describeRouteSubState(subState)}`);
    clues.push(`原图角点：左=${leftCornerFound ? '有' : '无'}，右=${rightCornerFound ? '有' : '无'}`);
    clues.push(`原图直边：左=${leftStraight ? '有' : '无'}，右=${rightStraight ? '有' : '无'}`);
    clues.push(`边框墙：左=${leftHasFrameWall ? '有' : '无'}，右=${rightHasFrameWall ? '有' : '无'}`);
    if (leftStartFrameRows !== null || rightStartFrameRows !== null) {
      clues.push(`起始贴边：左=${leftStartFrameRows !== null ? leftStartFrameRows : '--'}，右=${rightStartFrameRows !== null ? rightStartFrameRows : '--'}`);
    }
    if (leftBoundaryCount !== null || rightBoundaryCount !== null) {
      clues.push(`边界点数：左=${leftBoundaryCount !== null ? leftBoundaryCount : '--'}，右=${rightBoundaryCount !== null ? rightBoundaryCount : '--'}`);
    }
    if (leftGuideCount !== null || rightGuideCount !== null) {
      clues.push(`圆环辅助线：左=${leftGuideCount !== null ? leftGuideCount : '--'}，右=${rightGuideCount !== null ? rightGuideCount : '--'}`);
    }
    if (leftCornerIndex !== null || rightCornerIndex !== null) {
      clues.push(`角点索引：左=${leftCornerIndex !== null ? leftCornerIndex : '--'}，右=${rightCornerIndex !== null ? rightCornerIndex : '--'}`);
    }
    if (leftCornerY !== null || rightCornerY !== null) {
      clues.push(`角点y：左=${leftCornerY !== null ? leftCornerY : '--'}，右=${rightCornerY !== null ? rightCornerY : '--'}`);
    }
    if (cornerDistance !== null) {
      clues.push(`双角点距离：${cornerDistance.toFixed(1)} px`);
    }
    clues.push(`状态机开关：圆环=${routeCircleDetectionEnabled ? '开' : '关'}`);

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
      const straightStateHit = leftStraight && rightStraight && !leftCornerFound && !rightCornerFound;
      if (straightStateHit) {
        judgments.push('双侧都为直边且双侧都没有角点，下一次状态更新会进入直道态。');
      } else if (!routeCircleDetectionEnabled) {
        judgments.push('圆环状态机当前关闭，页面只展示输入特征，不会进入 circle 状态。');
      } else if (leftCircleEntryHit) {
        judgments.push('左环入口条件已完整命中，下一次状态更新会进入左环岛 circle_1。');
      } else if (rightCircleEntryHit) {
        judgments.push('右环入口条件已完整命中，下一次状态更新会进入右环岛 circle_1。');
      } else if (leftCornerFound && rightStraight) {
        judgments.push('左侧角点和右侧直边已经出现，但边界点数或角点索引余量还没完全满足左环入口阈值。');
      } else if (rightCornerFound && leftStraight) {
        judgments.push('右侧角点和左侧直边已经出现，但边界点数或角点索引余量还没完全满足右环入口阈值。');
      } else {
        judgments.push('当前没有满足圆环入口的组合条件，状态机继续留在正常巡线。');
      }

      pushCondition('直道态入口', formatHit(straightStateHit), '左直边=1 右直边=1 左角点=0 右角点=0');
      pushCondition('左环入口', formatHit(!!leftCircleEntryHit), `右直边=${rightStraight ? 1 : 0} 左角点=${leftCornerFound ? 1 : 0} 左角点y=${leftCornerY !== null ? leftCornerY : '--'}>60 右点数=${rightBoundaryCount !== null ? rightBoundaryCount : '--'}>${circleEntryMinBoundaryCount !== null ? circleEntryMinBoundaryCount : '--'} 左角点索引=${leftCornerIndex !== null ? leftCornerIndex : '--'} < 右点数-${circleEntryCornerTailMargin !== null ? circleEntryCornerTailMargin : '--'}`);
      pushCondition('右环入口', formatHit(!!rightCircleEntryHit), `左直边=${leftStraight ? 1 : 0} 右角点=${rightCornerFound ? 1 : 0} 右角点y=${rightCornerY !== null ? rightCornerY : '--'}>60 左点数=${leftBoundaryCount !== null ? leftBoundaryCount : '--'}>${circleEntryMinBoundaryCount !== null ? circleEntryMinBoundaryCount : '--'} 右角点索引=${rightCornerIndex !== null ? rightCornerIndex : '--'} < 左点数-${circleEntryCornerTailMargin !== null ? circleEntryCornerTailMargin : '--'}`);

      if (straightStateHit) {
        nextStateLabel = '直道态';
        nextReasons.push('当前帧满足双侧直边且双侧无角点，状态机会切到直道态。');
      } else if (leftCircleEntryHit) {
        nextStateLabel = '左环岛-circle_1';
        nextReasons.push('当前帧已经满足左环入口的全部判据，状态机会把偏好源切到右边界。');
      } else if (rightCircleEntryHit) {
        nextStateLabel = '右环岛-circle_1';
        nextReasons.push('当前帧已经满足右环入口的全部判据，状态机会把偏好源切到左边界。');
      } else {
        nextReasons.push('圆环入口条件未全部命中，继续保持正常巡线。');
      }
    } else if (mainState === 1) {
      judgments.push('当前处于直道态：两侧都为直边，且两侧都没有角点。');
      pushCondition('双侧直边且无角点', formatHit(leftStraight && rightStraight && !leftCornerFound && !rightCornerFound), '任一侧出现角点或不再同时为直边时，先退回 normal');
      nextStateLabel = '直道态';
      nextReasons.push('当前仍满足直道态保持条件。');
      if (!(leftStraight && rightStraight && !leftCornerFound && !rightCornerFound)) {
        nextStateLabel = '正常巡线';
        nextReasons[0] = '直道态保持条件被破坏，状态机先退回 normal，再按 normal 规则判断后续状态。';
      }
    } else if (mainState === 4) {
      judgments.push('该状态位当前在网页端仅保留原始显示，不再额外推断旧十字状态机逻辑。');
      nextReasons.push('请以实际 route_main_state / route_sub_state 原始值为准。');
      pushCondition('保留状态', subState !== null ? subState : '--', '当前页面不再扩展旧十字状态说明');
    } else if (mainState === 2) {
      judgments.push(`左环岛状态下，当前固定偏向 ${formatRoutePreferredSource(preferredSource)}。`);
      if (subState === 5 || subState === 7) {
        judgments.push(`当前处于补右线阶段，右侧辅助线点数=${rightGuideCount !== null ? rightGuideCount : 0}。`);
      }
      pushCondition('左起始贴边行数', leftStartFrameRows !== null ? leftStartFrameRows : '--', `circle_1 >= ${circleStageEnterRows !== null ? circleStageEnterRows : '--'}，circle_2 需回到 0`);
      pushCondition('右起始贴边行数', rightStartFrameRows !== null ? rightStartFrameRows : '--', `circle_3 > ${circleStage3RowsTrigger !== null ? circleStage3RowsTrigger : '--'}，circle_5 >= ${circleStageEnterRows !== null ? circleStageEnterRows : '--'}`);
      pushCondition('右侧角点', formatHit(rightCornerFound), 'circle_4 进入 circle_5');
      pushCondition('右侧辅助线', rightGuideCount !== null ? rightGuideCount : '--', `stage3 offset=${circleGuideTargetOffsetStage3 !== null ? circleGuideTargetOffsetStage3 : '--'} stage5 offset=${circleGuideAnchorOffsetStage5 !== null ? circleGuideAnchorOffsetStage5 : '--'} 最短贴边段=${circleGuideMinSegmentLen !== null ? circleGuideMinSegmentLen : '--'}`);
      pushCondition('双侧直边', formatHit(leftStraight && rightStraight), `circle_6 返回正常巡线，maze_start_row >= ${circleStage6MazeStartRow !== null ? circleStage6MazeStartRow : '--'}`);
      if (subState === 3) {
        nextStateLabel = '左环岛-circle_2';
        nextReasons.push(`左边界起始连续贴左边框达到 ${circleStageEnterRows !== null ? circleStageEnterRows : '--'} 行后，进入 circle_2。`);
      } else if (subState === 4) {
        nextStateLabel = '左环岛-circle_3';
        nextReasons.push('左边界起始行不再贴左边框时，进入 circle_3。');
      } else if (subState === 5) {
        nextStateLabel = '左环岛-circle_4';
        nextReasons.push(`右边界起始连续贴右边框超过 ${circleStage3RowsTrigger !== null ? circleStage3RowsTrigger : '--'} 行时，进入 circle_4。`);
      } else if (subState === 6) {
        nextStateLabel = '左环岛-circle_5';
        nextReasons.push('右边界检测到角点后，进入 circle_5。');
      } else if (subState === 7) {
        nextStateLabel = '左环岛-circle_6';
        nextReasons.push(`右边界起始连续贴右边框达到 ${circleStageEnterRows !== null ? circleStageEnterRows : '--'} 行后，进入 circle_6。`);
      } else if (subState === 8) {
        nextStateLabel = '正常巡线';
        nextReasons.push('双侧重新同时识别为直边后，退出左环岛。');
      } else {
        nextReasons.push('等待左环内部子状态推进。');
      }
    } else if (mainState === 3) {
      judgments.push(`右环岛状态下，当前固定偏向 ${formatRoutePreferredSource(preferredSource)}。`);
      if (subState === 11 || subState === 13) {
        judgments.push(`当前处于补左线阶段，左侧辅助线点数=${leftGuideCount !== null ? leftGuideCount : 0}。`);
      }
      pushCondition('右起始贴边行数', rightStartFrameRows !== null ? rightStartFrameRows : '--', `circle_1 >= ${circleStageEnterRows !== null ? circleStageEnterRows : '--'}，circle_2 需回到 0`);
      pushCondition('左起始贴边行数', leftStartFrameRows !== null ? leftStartFrameRows : '--', `circle_3 > ${circleStage3RowsTrigger !== null ? circleStage3RowsTrigger : '--'}，circle_5 >= ${circleStageEnterRows !== null ? circleStageEnterRows : '--'}`);
      pushCondition('左侧角点', formatHit(leftCornerFound), 'circle_4 进入 circle_5');
      pushCondition('左侧辅助线', leftGuideCount !== null ? leftGuideCount : '--', `stage3 offset=${circleGuideTargetOffsetStage3 !== null ? circleGuideTargetOffsetStage3 : '--'} stage5 offset=${circleGuideAnchorOffsetStage5 !== null ? circleGuideAnchorOffsetStage5 : '--'} 最短贴边段=${circleGuideMinSegmentLen !== null ? circleGuideMinSegmentLen : '--'}`);
      pushCondition('双侧直边', formatHit(leftStraight && rightStraight), `circle_6 返回正常巡线，maze_start_row >= ${circleStage6MazeStartRow !== null ? circleStage6MazeStartRow : '--'}`);
      if (subState === 9) {
        nextStateLabel = '右环岛-circle_2';
        nextReasons.push(`右边界起始连续贴右边框达到 ${circleStageEnterRows !== null ? circleStageEnterRows : '--'} 行后，进入 circle_2。`);
      } else if (subState === 10) {
        nextStateLabel = '右环岛-circle_3';
        nextReasons.push('右边界起始行不再贴右边框时，进入 circle_3。');
      } else if (subState === 11) {
        nextStateLabel = '右环岛-circle_4';
        nextReasons.push(`左边界起始连续贴左边框超过 ${circleStage3RowsTrigger !== null ? circleStage3RowsTrigger : '--'} 行时，进入 circle_4。`);
      } else if (subState === 12) {
        nextStateLabel = '右环岛-circle_5';
        nextReasons.push('左边界检测到角点后，进入 circle_5。');
      } else if (subState === 13) {
        nextStateLabel = '右环岛-circle_6';
        nextReasons.push(`左边界起始连续贴左边框达到 ${circleStageEnterRows !== null ? circleStageEnterRows : '--'} 行后，进入 circle_6。`);
      } else if (subState === 14) {
        nextStateLabel = '正常巡线';
        nextReasons.push('双侧重新同时识别为直边后，退出右环岛。');
      } else {
        nextReasons.push('等待右环内部子状态推进。');
      }
    } else {
      judgments.push('等待状态机首次更新。');
      nextReasons.push('等待有效状态数据。');
    }

    const counters = [
      ['左边界点数', leftBoundaryCount],
      ['右边界点数', rightBoundaryCount],
      ['左辅助线点数', leftGuideCount],
      ['右辅助线点数', rightGuideCount],
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
    if (hasValue(status.route_circle_detection_enabled)) push('route_circle_detection_enabled', status.route_circle_detection_enabled);
    if (Number.isFinite(Number(status.route_circle_entry_min_boundary_count))) push('route_circle_entry_min_boundary_count', status.route_circle_entry_min_boundary_count);
    if (Number.isFinite(Number(status.route_circle_entry_corner_tail_margin))) push('route_circle_entry_corner_tail_margin', status.route_circle_entry_corner_tail_margin);
    if (Number.isFinite(Number(status.route_circle_stage_frame_wall_rows_enter))) push('route_circle_stage_frame_wall_rows_enter', status.route_circle_stage_frame_wall_rows_enter);
    if (Number.isFinite(Number(status.route_circle_stage3_frame_wall_rows_trigger))) push('route_circle_stage3_frame_wall_rows_trigger', status.route_circle_stage3_frame_wall_rows_trigger);
    if (Number.isFinite(Number(status.route_circle_stage6_maze_start_row))) push('route_circle_stage6_maze_start_row', status.route_circle_stage6_maze_start_row);
    if (Number.isFinite(Number(status.circle_guide_min_frame_wall_segment_len))) push('circle_guide_min_frame_wall_segment_len', status.circle_guide_min_frame_wall_segment_len);
    if (Number.isFinite(Number(status.circle_guide_target_offset_stage3))) push('circle_guide_target_offset_stage3', status.circle_guide_target_offset_stage3);
    if (Number.isFinite(Number(status.circle_guide_anchor_offset_stage5))) push('circle_guide_anchor_offset_stage5', status.circle_guide_anchor_offset_stage5);
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
