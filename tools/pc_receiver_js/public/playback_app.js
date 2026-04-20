(() => {
  const receiverCore = window.SharedReceiverCore;
  const params = new URLSearchParams(window.location.search || '');

  const playbackToggleBtn = document.getElementById('playbackToggleBtn');
  const playbackRestartBtn = document.getElementById('playbackRestartBtn');
  const playbackBackwardBtn = document.getElementById('playbackBackwardBtn');
  const playbackForwardBtn = document.getElementById('playbackForwardBtn');
  const playbackSpeedBtn = document.getElementById('playbackSpeedBtn');
  const refreshBtn = document.getElementById('refreshBtn');
  const playbackTimeLabel = document.getElementById('playbackTimeLabel');
  const timeSubLabel = document.getElementById('timeSubLabel');
  const sourceLabel = document.getElementById('sourceLabel');
  const playbackGrid = document.getElementById('playbackGrid');
  const metaPills = document.getElementById('metaPills');
  const overviewGrid = document.getElementById('overviewGrid');
  const rawStatus = document.getElementById('rawStatus');
  const routeMainStateValue = document.getElementById('routeMainStateValue');
  const routeSubStateValue = document.getElementById('routeSubStateValue');
  const routePreferredSourceValue = document.getElementById('routePreferredSourceValue');
  const routeEncoderValue = document.getElementById('routeEncoderValue');
  const straightFocusPanel = document.getElementById('straightFocusPanel');
  const straightFocusBadge = document.getElementById('straightFocusBadge');
  const straightFocusCenterlineCount = document.getElementById('straightFocusCenterlineCount');
  const straightFocusLastIndex = document.getElementById('straightFocusLastIndex');
  const straightFocusErrorSum = document.getElementById('straightFocusErrorSum');
  const straightFocusErrorMax = document.getElementById('straightFocusErrorMax');
  const straightFocusDetail = document.getElementById('straightFocusDetail');
  const crossFocusPanel = document.getElementById('crossFocusPanel');
  const crossFocusBadge = document.getElementById('crossFocusBadge');
  const crossFocusLeftRows = document.getElementById('crossFocusLeftRows');
  const crossFocusRightRows = document.getElementById('crossFocusRightRows');
  const crossFocusGap = document.getElementById('crossFocusGap');
  const crossFocusAux = document.getElementById('crossFocusAux');
  const crossFocusDetail = document.getElementById('crossFocusDetail');
  const speedFocusPanel = document.getElementById('speedFocusPanel');
  const speedFocusBadge = document.getElementById('speedFocusBadge');
  const speedFocusCenterlineCount = document.getElementById('speedFocusCenterlineCount');
  const speedFocusDesiredSpeed = document.getElementById('speedFocusDesiredSpeed');
  const speedFocusMinCount = document.getElementById('speedFocusMinCount');
  const speedFocusErrorSum = document.getElementById('speedFocusErrorSum');
  const speedFocusErrorThreshold = document.getElementById('speedFocusErrorThreshold');
  const speedFocusDetail = document.getElementById('speedFocusDetail');
  const routeStateJudgment = document.getElementById('routeStateJudgment');
  const routeNextStateValue = document.getElementById('routeNextStateValue');
  const routeNextReason = document.getElementById('routeNextReason');
  const routeConditionLines = document.getElementById('routeConditionLines');
  const routeRawFlagLines = document.getElementById('routeRawFlagLines');
  const routeJudgeMetricLines = document.getElementById('routeJudgeMetricLines');
  const routeStateClues = document.getElementById('routeStateClues');
  const routeStateCounters = document.getElementById('routeStateCounters');
  const pidCommonList = document.getElementById('pidCommonList');
  const pidLeftList = document.getElementById('pidLeftList');
  const pidRightList = document.getElementById('pidRightList');
  const pidSlowdownCards = document.getElementById('pidSlowdownCards');

  let playbackVideoEntries = [];
  let playbackPrimaryKey = '';
  let isSyncingPlayback = false;
  let lastRecording = null;
  let transferChannel = null;
  const playbackRates = [0.25, 0.5, 0.75, 1.0, 1.25, 1.5];
  let playbackRate = 1.0;
  const SPEED_SCHEME_MODE = 'friction_circle';

  function hasValue(value) {
    return value !== undefined && value !== null;
  }

  function fmtPlaybackTime(sec) {
    const safeSec = Math.max(0, Number(sec) || 0);
    const minutes = Math.floor(safeSec / 60);
    const seconds = Math.floor(safeSec % 60);
    const centiseconds = Math.floor((safeSec - Math.floor(safeSec)) * 100);
    return `${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}.${String(centiseconds).padStart(2, '0')}`;
  }

  function fmtPlaybackRate(rate) {
    return `${Number(rate).toFixed(2)}x`;
  }

  function fmtPidValue(value) {
    if (typeof value === 'boolean') return value ? '1' : '0';
    if (typeof value === 'number' && Number.isFinite(value)) {
      if (Math.abs(value) >= 100 || Number.isInteger(value)) return value.toFixed(0);
      return value.toFixed(3);
    }
    if (Array.isArray(value)) return receiverCore.formatArrayInline(value);
    if (value === undefined || value === null || value === '') return 'N/A';
    return String(value);
  }

  function slowdownWinnerLabel(winner) {
    if (winner === 1) return 'speed_scheme_branch';
    return 'none';
  }

  function getSpeedSchemeRearExpLambda(status) {
    const raw = Number(status && status.pid_common_speed_scheme_rear_exp_lambda);
    return Number.isFinite(raw) ? raw : null;
  }

  function getSpeedSchemeSplitRatio(status) {
    const raw = Number(status && status.pid_common_speed_scheme_split_ratio);
    if (Number.isFinite(raw)) return raw;
    return 0.6;
  }

  function getSpeedSchemeErrorScaleRaw(status) {
    const raw = Number(status && status.pid_common_speed_scheme_error_scale_raw);
    return Number.isFinite(raw) ? raw : null;
  }

  function getSpeedSchemeFrictionCircleN(status) {
    const raw = Number(status && status.pid_common_speed_scheme_friction_circle_n);
    return Number.isFinite(raw) ? raw : null;
  }

  function getSpeedSchemeRealtimeSpeed(status) {
    const raw = Number(status && status.pid_common_speed_scheme_realtime_speed);
    return Number.isFinite(raw) ? raw : null;
  }

  function getSpeedSchemeFrictionCoupling(status) {
    const raw = Number(status && status.pid_common_speed_scheme_friction_coupling);
    return Number.isFinite(raw) ? raw : null;
  }

  function renderSlowdownCards(status, container) {
    if (!container) return;
    const winner = Number(status && status.pid_common_speed_scheme_winner_branch) || 0;
    const splitRatio = getSpeedSchemeSplitRatio(status);
    const rearExpLambda = getSpeedSchemeRearExpLambda(status);
    const errorScaleRaw = getSpeedSchemeErrorScaleRaw(status);
    const frictionCircleN = getSpeedSchemeFrictionCircleN(status);
    const realtimeSpeed = getSpeedSchemeRealtimeSpeed(status);
    const frictionCoupling = getSpeedSchemeFrictionCoupling(status);
    const branches = [
      {
        id: 1,
        name: 'Friction Circle',
        lines: [
          `模式: <span>${SPEED_SCHEME_MODE}</span>`,
          `split_ratio: <span>${fmtPidValue(splitRatio)}</span>（前段 0~split，后段 split~1）`,
          `后段指数权重 lambda: <span>${fmtPidValue(rearExpLambda)}</span>`,
          `目标角度(abs deg): <span>${fmtPidValue(status && status.pid_common_speed_scheme_blended_abs_error_sum)}</span>`,
          `摩擦圆常数 n: <span>${fmtPidValue(frictionCircleN)}</span>`,
          `实时速度(用于耦合): <span>${fmtPidValue(realtimeSpeed)}</span>`,
          `耦合项(角度*实时速度): <span>${fmtPidValue(frictionCoupling)}</span>`,
          `比例(raw/final): <span>${fmtPidValue(errorScaleRaw)} / ${fmtPidValue(status && status.pid_common_speed_scheme_final_speed_scale)}</span>`,
          `中线点数 current: <span>${fmtPidValue(status && status.pid_common_speed_scheme_point_count)}</span>`,
          `单周期升降速(rise/drop): <span>${fmtPidValue(status && status.pid_common_speed_scheme_max_rise_ratio_per_cycle)} / ${fmtPidValue(status && status.pid_common_speed_scheme_max_drop_ratio_per_cycle)}</span>`,
          `判定(ready/triggered): <span>${fmtPidValue(status && status.pid_common_speed_scheme_ready)} / ${fmtPidValue(status && status.pid_common_speed_scheme_triggered)}</span>`
        ]
      }
    ];
    container.innerHTML = branches.map((branch) => {
      const active = (branch.id === 1 && winner === 1);
      const chip = active ? '选中支路' : '未选中';
      return `
        <div class="pid-slowdown-card${active ? ' active' : ''}">
          <div class="pid-slowdown-card-head">
            <div class="pid-slowdown-name">${branch.name}</div>
            <div class="pid-slowdown-chip${active ? ' active' : ''}">${chip}</div>
          </div>
          <div class="pid-slowdown-lines">${branch.lines.map((line) => `<div>${line}</div>`).join('')}</div>
        </div>
      `;
    }).join('');
  }

  function renderStraightFocus(status) {
    if (!straightFocusPanel) return;
    const centerlineCount = Number(status && status.straight_selected_centerline_count);
    const lastIndex = Number(status && status.straight_required_last_index);
    const errorSum = Number(status && status.straight_abs_error_sum);
    const errorMax = Number(status && status.straight_abs_error_sum_max);
    const readyNow = !!(status && status.straight_state_ready_now);
    const hasData = Number.isFinite(centerlineCount) || Number.isFinite(lastIndex) || Number.isFinite(errorSum);

    straightFocusPanel.classList.remove('ready', 'not-ready');
    straightFocusPanel.classList.add(readyNow ? 'ready' : 'not-ready');
    straightFocusBadge.textContent = !hasData ? '等待数据' : (readyNow ? '满足直道条件' : '未满足直道条件');
    straightFocusCenterlineCount.textContent = Number.isFinite(centerlineCount) ? String(centerlineCount) : '--';
    straightFocusLastIndex.textContent = Number.isFinite(lastIndex) ? String(lastIndex) : '--';
    straightFocusErrorSum.textContent = Number.isFinite(errorSum) ? errorSum.toFixed(1) : '--';
    straightFocusErrorMax.textContent = Number.isFinite(errorMax) ? errorMax.toFixed(1) : '--';

    if (!hasData) {
      straightFocusDetail.textContent = '等待直道判定数据...';
      return;
    }

    const reasons = [];
    if (!Number.isFinite(lastIndex) || lastIndex < 0) {
      reasons.push('当前还没有有效的“最后索引”');
    } else if (!Number.isFinite(centerlineCount) || centerlineCount <= lastIndex) {
      reasons.push(`中线点数不足：${Number.isFinite(centerlineCount) ? centerlineCount : '--'} <= ${lastIndex}`);
    } else {
      reasons.push(`点数条件满足：${centerlineCount} > ${lastIndex}`);
    }

    if (!Number.isFinite(errorSum) || !Number.isFinite(errorMax)) {
      reasons.push('误差和或阈值数据无效');
    } else if (errorSum >= errorMax) {
      reasons.push(`误差和超阈值：${errorSum.toFixed(1)} >= ${errorMax.toFixed(1)}`);
    } else {
      reasons.push(`误差和条件满足：${errorSum.toFixed(1)} < ${errorMax.toFixed(1)}`);
    }

    const enterFrames = Number(status && status.straight_enter_consecutive_frames);
    if (Number.isFinite(enterFrames)) {
      reasons.push(`进入 straight 还需要连续满足 ${enterFrames} 帧`);
    }
    straightFocusDetail.textContent = reasons.join('；');
  }

  function renderCrossFocus(status) {
    if (!crossFocusPanel) return;
    const leftRows = Number(status && status.cross_left_corner_post_frame_wall_rows);
    const rightRows = Number(status && status.cross_right_corner_post_frame_wall_rows);
    const gap = Number(status && status.cross_start_boundary_gap_x);
    const entryReady = !!(status && status.cross_state_entry_ready_now);
    const stage2Ready = !!(status && status.cross_state_stage2_ready_now);
    const stage3Ready = !!(status && status.cross_state_stage3_ready_now);
    const exitReady = !!(status && status.cross_state_exit_ready_now);
    const mainState = Number(status && status.route_main_state);
    const subState = Number(status && status.route_sub_state);
    const leftAux = !!(status && status.cross_left_aux_found);
    const rightAux = !!(status && status.cross_right_aux_found);
    const leftUpper = !!(status && status.cross_left_upper_corner_found);
    const rightUpper = !!(status && status.cross_right_upper_corner_found);
    const leftAuxTraceCount = Number(status && status.cross_left_aux_trace_count);
    const rightAuxTraceCount = Number(status && status.cross_right_aux_trace_count);
    const leftAuxRegularCount = Number(status && status.cross_left_aux_regular_count);
    const rightAuxRegularCount = Number(status && status.cross_right_aux_regular_count);
    const leftAuxDir5Count = Number(status && status.cross_left_aux_dir5_count);
    const rightAuxDir5Count = Number(status && status.cross_right_aux_dir5_count);
    const hasData = Number.isFinite(leftRows) || Number.isFinite(rightRows) || Number.isFinite(gap);

    crossFocusPanel.classList.remove('ready', 'not-ready');
    crossFocusPanel.classList.add((entryReady || stage2Ready || stage3Ready || exitReady || mainState === 4) ? 'ready' : 'not-ready');
    let crossStateLabel = '十字-cross_1 运行中';
    if (subState === 2) crossStateLabel = '十字-cross_2 运行中';
    if (subState === 3) crossStateLabel = '十字-cross_3 运行中';
    crossFocusBadge.textContent = !hasData
      ? '等待数据'
      : (mainState === 4 ? crossStateLabel : (entryReady ? '满足十字入口' : '未满足十字入口'));
    crossFocusLeftRows.textContent = Number.isFinite(leftRows) ? String(leftRows) : '--';
    crossFocusRightRows.textContent = Number.isFinite(rightRows) ? String(rightRows) : '--';
    crossFocusGap.textContent = Number.isFinite(gap) ? String(gap) : '--';
    crossFocusAux.textContent = `L${leftAux ? '1' : '0'} R${rightAux ? '1' : '0'}`;

    if (!hasData) {
      crossFocusDetail.textContent = '等待十字判定数据...';
      return;
    }

    const details = [];
    details.push(`入口=${entryReady ? '命中' : '未命中'}`);
    details.push(`cross_1->2=${stage2Ready ? '命中' : '未命中'}`);
    details.push(`cross_2->3=${stage3Ready ? '命中' : '未命中'}`);
    details.push(`cross_3->normal=${exitReady ? '命中' : '未命中'}`);
    details.push(`辅助线 L=${leftAux ? '有' : '无'} R=${rightAux ? '有' : '无'}`);
    details.push(`辅线点数 L=${Number.isFinite(leftAuxTraceCount) ? leftAuxTraceCount : '--'}/${Number.isFinite(leftAuxRegularCount) ? leftAuxRegularCount : '--'} R=${Number.isFinite(rightAuxTraceCount) ? rightAuxTraceCount : '--'}/${Number.isFinite(rightAuxRegularCount) ? rightAuxRegularCount : '--'}`);
    details.push(`辅线 dir5 L=${Number.isFinite(leftAuxDir5Count) ? leftAuxDir5Count : '--'} R=${Number.isFinite(rightAuxDir5Count) ? rightAuxDir5Count : '--'}`);
    details.push(`上角点 L=${leftUpper ? '有' : '无'} R=${rightUpper ? '有' : '无'}`);
    crossFocusDetail.textContent = details.join('；');
  }

  function renderSpeedFocus(status) {
    if (!speedFocusPanel) return;
    const centerlineCount = Number(status && status.straight_selected_centerline_count);
    const targetAbsAngle = Number(status && status.pid_common_speed_scheme_blended_abs_error_sum);
    const frictionCircleN = getSpeedSchemeFrictionCircleN(status);
    const frictionCoupling = getSpeedSchemeFrictionCoupling(status);
    const profileBaseSpeed = Number(status && status.pid_common_profile_base_speed);
    const desiredBaseSpeed = Number(status && status.pid_common_desired_base_speed);
    const appliedBaseSpeed = Number(status && status.pid_common_applied_base_speed);
    const hasData = Number.isFinite(centerlineCount) ||
      Number.isFinite(targetAbsAngle) ||
      Number.isFinite(frictionCircleN) ||
      Number.isFinite(frictionCoupling) ||
      Number.isFinite(desiredBaseSpeed);

    speedFocusPanel.classList.remove('ready', 'not-ready');
    speedFocusPanel.classList.add(hasData ? 'ready' : 'not-ready');

    let modeLabel = '等待数据';
    if (hasData) {
      if (Number.isFinite(desiredBaseSpeed) && Number.isFinite(appliedBaseSpeed)) {
        const speedDelta = desiredBaseSpeed - appliedBaseSpeed;
        if (speedDelta > 1.0) modeLabel = '加速中';
        else if (speedDelta < -1.0) modeLabel = '减速中';
        else modeLabel = '速度持平';
      } else {
        modeLabel = '条件评估中';
      }
    }

    speedFocusBadge.textContent = modeLabel;
    speedFocusCenterlineCount.textContent = Number.isFinite(centerlineCount) ? String(centerlineCount) : '--';
    if (speedFocusDesiredSpeed) {
      speedFocusDesiredSpeed.textContent = Number.isFinite(desiredBaseSpeed) ? desiredBaseSpeed.toFixed(1) : '--';
    }
    speedFocusMinCount.textContent = Number.isFinite(frictionCircleN) ? frictionCircleN.toFixed(3) : '--';
    speedFocusErrorSum.textContent = Number.isFinite(targetAbsAngle) ? targetAbsAngle.toFixed(1) : '--';
    speedFocusErrorThreshold.textContent = Number.isFinite(frictionCoupling) ? frictionCoupling.toFixed(3) : '--';

    if (!hasData) {
      speedFocusDetail.textContent = '等待加减速数据...';
      return;
    }

    const details = [];
    details.push('速度依据：v_target^2 + (angle*realtime_speed)^2 = n');
    if (Number.isFinite(targetAbsAngle)) {
      details.push(`目标角度(abs deg)=${targetAbsAngle.toFixed(1)}`);
    }
    if (Number.isFinite(frictionCircleN) && Number.isFinite(frictionCoupling)) {
      details.push(`摩擦圆输入: n=${frictionCircleN.toFixed(3)}, coupling=${frictionCoupling.toFixed(3)}`);
    }
    if (Number.isFinite(profileBaseSpeed) && Number.isFinite(desiredBaseSpeed) && Number.isFinite(appliedBaseSpeed)) {
      const speedDelta = desiredBaseSpeed - appliedBaseSpeed;
      details.push(`速度(profile/desired/applied)=${profileBaseSpeed.toFixed(1)}/${desiredBaseSpeed.toFixed(1)}/${appliedBaseSpeed.toFixed(1)}`);
      details.push(`速度差(目标-当前)=${speedDelta.toFixed(1)}`);
    }
    speedFocusDetail.textContent = details.join('；');
  }

  function renderMetaPills(recording) {
    const pills = [];
    if (recording && recording.folder) pills.push(`目录: ${recording.folder}`);
    if (recording && hasValue(recording.frameCount)) pills.push(`状态快照: ${recording.frameCount}`);
    if (recording && hasValue(recording.durationMs)) pills.push(`时长: ${(Number(recording.durationMs) / 1000).toFixed(2)}s`);
    const views = recording && recording.sessionMeta && Array.isArray(recording.sessionMeta.recorded_views)
      ? recording.sessionMeta.recorded_views.map((item) => item.title || item.key).filter(Boolean)
      : [];
    if (views.length > 0) pills.push(`录制视图: ${views.join(' / ')}`);
    if (recording && recording.sessionMeta && recording.sessionMeta.ui_state) {
      const ui = recording.sessionMeta.ui_state;
      pills.push(`录制选项: local_compute=${ui.localComputeEnabled ? '1' : '0'} gray_boundaries=${ui.showGrayBoundaries ? '1' : '0'} mean_centerline=${ui.showMeanCenterline ? '1' : '0'}`);
    }
    metaPills.innerHTML = pills.map((text) => `<div class="pill">${text}</div>`).join('');
  }

  function renderRouteStatePanel(status) {
    const route = receiverCore.buildRouteStateSummary(status || {});
    renderStraightFocus(status);
    renderCrossFocus(status);
    renderSpeedFocus(status);
    routeMainStateValue.textContent = route.mainStateLabel || '--';
    routeSubStateValue.textContent = route.subStateLabel || '--';
    routePreferredSourceValue.textContent = route.preferredSourceLabel || '--';
    routeEncoderValue.textContent = route.encoderSinceEnter !== null ? `${route.encoderSinceEnter}` : '--';
    routeStateJudgment.textContent = route.judgments.length > 0 ? route.judgments.join(' ') : '等待状态机数据...';
    routeNextStateValue.textContent = route.nextStateLabel || '--';

    routeNextReason.innerHTML = '';
    const nextReasons = route.nextReasons.length > 0 ? route.nextReasons : ['等待状态机数据...'];
    nextReasons.forEach((text) => {
      const div = document.createElement('div');
      div.className = 'route-line';
      div.textContent = text;
      routeNextReason.appendChild(div);
    });

    routeConditionLines.innerHTML = '';
    const conditionLines = route.conditionLines.length > 0
      ? route.conditionLines
      : [{ label: '判定条件', value: '--', detail: '等待状态机数据...' }];
    conditionLines.forEach((item) => {
      const div = document.createElement('div');
      div.className = 'route-line';
      div.innerHTML = `<strong>${item.label}</strong>${item.value}${item.detail ? ` | ${item.detail}` : ''}`;
      routeConditionLines.appendChild(div);
    });

    if (routeRawFlagLines) {
      routeRawFlagLines.innerHTML = '';
      const rawLines = Array.isArray(route.rawFlagLines) && route.rawFlagLines.length > 0
        ? route.rawFlagLines
        : [{ label: '原图标志层', value: '--', detail: '等待状态机输入层数据...' }];
      rawLines.forEach((item) => {
        const div = document.createElement('div');
        div.className = 'route-line';
        div.innerHTML = `<strong>${item.label}</strong>${item.value}${item.detail ? ` | ${item.detail}` : ''}`;
        routeRawFlagLines.appendChild(div);
      });
    }

    if (routeJudgeMetricLines) {
      routeJudgeMetricLines.innerHTML = '';
      const judgeLines = Array.isArray(route.judgeMetricLines) && route.judgeMetricLines.length > 0
        ? route.judgeMetricLines
        : [{ label: '状态判断', value: '--', detail: '等待状态机输入层数据...' }];
      judgeLines.forEach((item) => {
        const div = document.createElement('div');
        div.className = 'route-line';
        div.innerHTML = `<strong>${item.label}</strong>${item.value}${item.detail ? ` | ${item.detail}` : ''}`;
        routeJudgeMetricLines.appendChild(div);
      });
    }

    routeStateClues.innerHTML = '';
    const clues = route.clues.length > 0 ? route.clues : ['暂无可用判定线索'];
    clues.forEach((text) => {
      const div = document.createElement('div');
      div.className = 'route-chip';
      div.textContent = text;
      routeStateClues.appendChild(div);
    });

    routeStateCounters.innerHTML = '';
    const counters = route.counters.length > 0 ? route.counters : [['状态计数', null]];
    counters.forEach(([label, value]) => {
      const div = document.createElement('div');
      div.className = 'route-counter';
      div.innerHTML = `<strong>${label}</strong>${value === null ? '--' : value}`;
      routeStateCounters.appendChild(div);
    });
  }

  function renderPidColumn(container, rows) {
    if (!container) return;
    if (!rows || rows.length < 1) {
      container.innerHTML = '<div class="pid-empty">当前录制里没有 PID 调试数据。</div>';
      return;
    }
    container.innerHTML = rows.map(([label, value]) =>
      `<div class="pid-item"><strong>${label}</strong><span>${fmtPidValue(value)}</span></div>`).join('');
  }

  function renderPidStatusPanel(status) {
    const commonRows = [
      ['vision_updated', status.pid_common_vision_updated],
      ['imu_updated', status.pid_common_imu_updated],
      ['route_main_state', status.pid_common_route_main_state],
      ['route_sub_state', status.pid_common_route_sub_state],
      ['normal_speed_reference', status.pid_common_normal_speed_reference],
      ['profile_base_speed', status.pid_common_profile_base_speed],
      ['desired_base_speed', status.pid_common_desired_base_speed],
      ['applied_base_speed', status.pid_common_applied_base_speed],
      ['raw_error_px', status.pid_common_raw_error_px],
      ['normalized_error', status.pid_common_normalized_error],
      ['filtered_error_norm', status.pid_common_filtered_error_norm],
      ['filtered_error_px', status.pid_common_filtered_error_px],
      ['abs_filtered_error_px', status.pid_common_abs_filtered_error_px],
      ['control_error_norm', status.pid_common_control_error_norm],
      ['track_point', status.pid_common_track_point],
      ['track_angle(current/filtered)', [status.pid_common_current_track_point_angle_deg, status.pid_common_filtered_track_point_angle_deg]],
      ['yaw_rate(measured/ref/error)', [status.pid_common_measured_yaw_rate_dps, status.pid_common_yaw_rate_ref_dps, status.pid_common_yaw_rate_error_dps]],
      ['yaw_rate_error_norm', status.pid_common_yaw_rate_error_norm],
      ['dynamic_position_kp', status.pid_common_dynamic_position_kp],
      ['dynamic_yaw_rate_kp', status.pid_common_dynamic_yaw_rate_kp],
      ['applied_yaw_rate_kp', status.pid_common_applied_yaw_rate_kp],
      ['position_pid(kp/ki/kd)', [status.pid_common_position_pid_kp, status.pid_common_position_pid_ki, status.pid_common_position_pid_kd]],
      ['position_pid(target/error/output)', [status.pid_common_position_pid_target, status.pid_common_position_pid_error, status.pid_common_position_pid_output]],
      ['position_pid(integral/max_i/max_o)', [status.pid_common_position_pid_integral, status.pid_common_position_pid_max_integral, status.pid_common_position_pid_max_output]],
      ['yaw_pid(kp/ki/kd)', [status.pid_common_yaw_pid_kp, status.pid_common_yaw_pid_ki, status.pid_common_yaw_pid_kd]],
      ['yaw_pid(target/error/output)', [status.pid_common_yaw_pid_target, status.pid_common_yaw_pid_error, status.pid_common_yaw_pid_output]],
      ['yaw_pid(integral/max_i/max_o)', [status.pid_common_yaw_pid_integral, status.pid_common_yaw_pid_max_integral, status.pid_common_yaw_pid_max_output]],
      ['route(yaw_gain/ref_limit/steer_max)', [status.pid_common_route_yaw_rate_ref_gain, status.pid_common_route_yaw_rate_ref_limit, status.pid_common_route_steering_max_output]],
      ['yaw_kp_enable_error_threshold_px', status.pid_common_route_yaw_rate_kp_enable_error_threshold_px],
      ['mean_abs_path_error', status.pid_common_mean_abs_path_error],
      ['speed_scheme_mode', SPEED_SCHEME_MODE],
      ['speed_scheme_split_ratio', getSpeedSchemeSplitRatio(status)],
      ['speed_scheme_rear_exp_lambda', getSpeedSchemeRearExpLambda(status)],
      ['speed_scheme_friction_circle_n', getSpeedSchemeFrictionCircleN(status)],
      ['speed_scheme_realtime_speed', getSpeedSchemeRealtimeSpeed(status)],
      ['speed_scheme_friction_coupling', getSpeedSchemeFrictionCoupling(status)],
      ['speed_scheme_target_abs_angle_deg', status.pid_common_speed_scheme_blended_abs_error_sum],
      ['speed_scheme_error_scale_raw', getSpeedSchemeErrorScaleRaw(status)],
      ['speed_scheme_scale(current)', status.pid_common_speed_scheme_final_speed_scale],
      ['speed_scheme_point_count', status.pid_common_speed_scheme_point_count],
      ['speed_scheme_ramp(rise/drop)', [status.pid_common_speed_scheme_max_rise_ratio_per_cycle, status.pid_common_speed_scheme_max_drop_ratio_per_cycle]],
      ['speed_scheme_state(ready/triggered/winner)', [status.pid_common_speed_scheme_ready, status.pid_common_speed_scheme_triggered, slowdownWinnerLabel(status.pid_common_speed_scheme_winner_branch)]],
      ['steering(raw/clamped/applied)', [status.pid_common_raw_steering_output, status.pid_common_clamped_steering_output, status.pid_common_applied_steering_output]],
      ['line_follow_targets(L/R)', [status.pid_common_left_target_count_from_line_follow, status.pid_common_right_target_count_from_line_follow]],
      ['loop_dt_ms(vision/imu)', [status.pid_common_vision_dt_ms, status.pid_common_imu_dt_ms]],
      ['motor_cfg(integral/max_step/correction/duty)', [status.pid_common_motor_integral_limit, status.pid_common_motor_max_output_step, status.pid_common_motor_correction_limit, status.pid_common_motor_duty_limit]],
      ['motor_cfg(bias_thr/decel_thr/gain/limit)', [status.pid_common_motor_feedforward_bias_threshold, status.pid_common_motor_decel_error_threshold, status.pid_common_motor_decel_duty_gain, status.pid_common_motor_decel_duty_limit]]
    ].filter(([, value]) => value !== undefined && value !== null);

    const leftRows = [
      ['motor_pid(kp/ki/kd)', [status.pid_left_motor_pid_kp, status.pid_left_motor_pid_ki, status.pid_left_motor_pid_kd]],
      ['feedforward(gain/bias)', [status.pid_left_motor_feedforward_gain, status.pid_left_motor_feedforward_bias]],
      ['pid(target/error/output)', [status.pid_left_motor_pid_target, status.pid_left_motor_pid_error, status.pid_left_motor_pid_output]],
      ['pid(output_min/max)', [status.pid_left_motor_pid_output_min, status.pid_left_motor_pid_output_max]],
      ['pid(integral_limit/max_step)', [status.pid_left_motor_pid_integral_limit, status.pid_left_motor_pid_max_output_step]],
      ['target_count', status.pid_left_target_count],
      ['current_count', status.pid_left_current_count],
      ['feedback', status.pid_left_feedback],
      ['error', status.pid_left_error],
      ['feedforward', status.pid_left_feedforward],
      ['correction', status.pid_left_correction],
      ['decel_assist', status.pid_left_decel_assist],
      ['duty', status.pid_left_duty],
      ['hardware_duty', status.pid_left_hardware_duty],
      ['dir_level', status.pid_left_dir_level]
    ].filter(([, value]) => value !== undefined && value !== null);

    const rightRows = [
      ['motor_pid(kp/ki/kd)', [status.pid_right_motor_pid_kp, status.pid_right_motor_pid_ki, status.pid_right_motor_pid_kd]],
      ['feedforward(gain/bias)', [status.pid_right_motor_feedforward_gain, status.pid_right_motor_feedforward_bias]],
      ['pid(target/error/output)', [status.pid_right_motor_pid_target, status.pid_right_motor_pid_error, status.pid_right_motor_pid_output]],
      ['pid(output_min/max)', [status.pid_right_motor_pid_output_min, status.pid_right_motor_pid_output_max]],
      ['pid(integral_limit/max_step)', [status.pid_right_motor_pid_integral_limit, status.pid_right_motor_pid_max_output_step]],
      ['target_count', status.pid_right_target_count],
      ['current_count', status.pid_right_current_count],
      ['feedback', status.pid_right_feedback],
      ['error', status.pid_right_error],
      ['feedforward', status.pid_right_feedforward],
      ['correction', status.pid_right_correction],
      ['decel_assist', status.pid_right_decel_assist],
      ['duty', status.pid_right_duty],
      ['hardware_duty', status.pid_right_hardware_duty],
      ['dir_level', status.pid_right_dir_level]
    ].filter(([, value]) => value !== undefined && value !== null);

    renderPidColumn(pidCommonList, commonRows);
    renderPidColumn(pidLeftList, leftRows);
    renderPidColumn(pidRightList, rightRows);
    renderSlowdownCards(status, pidSlowdownCards);
  }

  function renderOverview(status, video, duration) {
    const splitRatio = getSpeedSchemeSplitRatio(status);
    const splitRatioText = Number.isFinite(splitRatio) ? fmtPidValue(splitRatio) : 'N/A';
    const speedSchemeRearExpLambda = getSpeedSchemeRearExpLambda(status);
    const speedSchemeFrictionCircleN = getSpeedSchemeFrictionCircleN(status);
    const speedSchemeRealtimeSpeed = getSpeedSchemeRealtimeSpeed(status);
    const speedSchemeFrictionCoupling = getSpeedSchemeFrictionCoupling(status);
    const speedSchemeErrorScaleRaw = getSpeedSchemeErrorScaleRaw(status);
    const rows = [
      ['当前时间', `${fmtPlaybackTime(video ? (video.currentTime || 0) : 0)} / ${fmtPlaybackTime(duration || 0)}`],
      ['主时间轴', playbackPrimaryKey || '--'],
      ['line_error', hasValue(status.line_error) ? fmtPidValue(status.line_error) : 'N/A'],
      ['速度方案', SPEED_SCHEME_MODE],
      ['分段比例/后段指数lambda', `split=${splitRatioText}, lambda=${fmtPidValue(speedSchemeRearExpLambda)}`],
      ['摩擦圆参数', `n=${fmtPidValue(speedSchemeFrictionCircleN)}, v_rt=${fmtPidValue(speedSchemeRealtimeSpeed)}`],
      ['中线点数', hasValue(status.pid_common_speed_scheme_point_count) ? fmtPidValue(status.pid_common_speed_scheme_point_count) : 'N/A'],
      ['目标夹角/error_scale/current', `${hasValue(status.pid_common_speed_scheme_blended_abs_error_sum) ? fmtPidValue(status.pid_common_speed_scheme_blended_abs_error_sum) : 'N/A'} / ${hasValue(speedSchemeErrorScaleRaw) ? fmtPidValue(speedSchemeErrorScaleRaw) : 'N/A'} / ${hasValue(status.pid_common_speed_scheme_final_speed_scale) ? fmtPidValue(status.pid_common_speed_scheme_final_speed_scale) : 'N/A'}`],
      ['耦合项(角度*实时速度)', hasValue(speedSchemeFrictionCoupling) ? fmtPidValue(speedSchemeFrictionCoupling) : 'N/A'],
      ['单周期升降速', `${hasValue(status.pid_common_speed_scheme_max_rise_ratio_per_cycle) ? fmtPidValue(status.pid_common_speed_scheme_max_rise_ratio_per_cycle) : 'N/A'} / ${hasValue(status.pid_common_speed_scheme_max_drop_ratio_per_cycle) ? fmtPidValue(status.pid_common_speed_scheme_max_drop_ratio_per_cycle) : 'N/A'} (rise/drop)`],
      ['巡线输出', hasValue(status.pid_common_applied_steering_output) ? fmtPidValue(status.pid_common_applied_steering_output) : 'N/A'],
      ['基础速度', hasValue(status.pid_common_applied_base_speed) ? fmtPidValue(status.pid_common_applied_base_speed) : 'N/A'],
      ['状态机', [status.route_main_state, status.route_sub_state].filter(hasValue).join(' / ') || 'N/A'],
      ['直道判定', hasValue(status.straight_state_ready_now) ? (status.straight_state_ready_now ? '满足' : '未满足') : 'N/A'],
      ['十字入口判定', hasValue(status.cross_state_entry_ready_now) ? (status.cross_state_entry_ready_now ? '满足' : '未满足') : 'N/A'],
      ['红框/ROI', `red=${hasValue(status.red_found) ? status.red_found : 'N/A'} roi=${hasValue(status.roi_valid) ? status.roi_valid : 'N/A'}`],
      ['推理', hasValue(status.ncnn_top_label) ? `${status.ncnn_top_label}${hasValue(status.ncnn_top_score) ? ` (${fmtPidValue((Number(status.ncnn_top_score) || 0) * 100)}%)` : ''}` : 'N/A'],
      ['推理耗时', hasValue(status.ncnn_infer_us) ? `${status.ncnn_infer_us} us` : 'N/A'],
      ['录制视图数', lastRecording ? String(Object.keys(lastRecording.urls || {}).filter((key) => key !== 'status').length) : '0']
    ];
    overviewGrid.innerHTML = rows.map(([label, value]) =>
      `<div class="overview-item"><strong>${label}</strong><span>${value}</span></div>`).join('');
  }

  function setRawStatus(status) {
    rawStatus.textContent = JSON.stringify(status || {}, null, 2);
  }

  function currentPlaybackVideo() {
    if (playbackPrimaryKey) {
      const hit = playbackVideoEntries.find((item) => item.key === playbackPrimaryKey && item.videoEl && item.videoEl.src);
      if (hit) return hit.videoEl;
    }
    const first = playbackVideoEntries.find((item) => item.videoEl && item.videoEl.src);
    if (first) {
      playbackPrimaryKey = first.key;
      return first.videoEl;
    }
    return null;
  }

  function allPlaybackVideos() {
    return playbackVideoEntries.filter((item) => item.videoEl && item.videoEl.src).map((item) => item.videoEl);
  }

  function syncPlaybackVideos(sourceVideo) {
    const videos = allPlaybackVideos();
    if (!sourceVideo || videos.length < 2) return;
    isSyncingPlayback = true;
    try {
      videos.forEach((video) => {
        if (video === sourceVideo) return;
        video.playbackRate = playbackRate;
        if (Math.abs((video.currentTime || 0) - (sourceVideo.currentTime || 0)) > 0.08) {
          video.currentTime = sourceVideo.currentTime || 0;
        }
        if (sourceVideo.paused && !video.paused) video.pause();
      });
    } finally {
      isSyncingPlayback = false;
    }
  }

  function playbackStatusAt(timeSec) {
    if (!lastRecording || !Array.isArray(lastRecording.statusFrames) || lastRecording.statusFrames.length < 1) return {};
    const targetMs = Math.max(0, timeSec * 1000);
    let best = lastRecording.statusFrames[0];
    let bestDiff = Number.POSITIVE_INFINITY;
    for (const frame of lastRecording.statusFrames) {
      const frameOffset = Math.max(0, Number(frame.client_ts_ms) - Number(lastRecording.startedAtMs));
      const diff = Math.abs(frameOffset - targetMs);
      if (diff < bestDiff) {
        bestDiff = diff;
        best = frame;
      }
    }
    return best ? (best.status || {}) : {};
  }

  function refreshPlaybackView() {
    const video = currentPlaybackVideo();
    const duration = video && Number.isFinite(video.duration)
      ? video.duration
      : (lastRecording ? (Number(lastRecording.durationMs) || 0) / 1000 : 0);
    const current = video ? (video.currentTime || 0) : 0;
    const status = playbackStatusAt(current);
    playbackTimeLabel.textContent = `${fmtPlaybackTime(current)} / ${fmtPlaybackTime(duration)}`;
    playbackToggleBtn.textContent = video && !video.paused ? '暂停' : '播放';
    if (playbackSpeedBtn) playbackSpeedBtn.textContent = `倍速 ${fmtPlaybackRate(playbackRate)}`;
    timeSubLabel.textContent = video
      ? `${video.paused ? '回放已暂停' : '回放中'}，当前主时间轴为 ${playbackPrimaryKey || '--'}，倍速 ${fmtPlaybackRate(playbackRate)}`
      : '等待回放内容...';
    renderOverview(status, video, duration);
    renderRouteStatePanel(status);
    renderPidStatusPanel(status);
    setRawStatus(status);
  }

  function bindPlaybackVideoEvents(video, key, wrap) {
    if (!video) return;
    video.playbackRate = playbackRate;
    video.addEventListener('click', () => {
      playbackPrimaryKey = key;
      playbackVideoEntries.forEach((item) => item.wrapEl.classList.toggle('active', item.key === key));
      refreshPlaybackView();
    });
    ['timeupdate', 'seeked', 'play', 'pause'].forEach((name) => {
      video.addEventListener(name, () => {
        if (isSyncingPlayback) return;
        if (video === currentPlaybackVideo()) {
          syncPlaybackVideos(video);
          refreshPlaybackView();
        }
      });
    });
    video.addEventListener('ended', () => {
      if (video === currentPlaybackVideo()) refreshPlaybackView();
    });
    wrap.addEventListener('click', () => {
      playbackPrimaryKey = key;
      playbackVideoEntries.forEach((item) => item.wrapEl.classList.toggle('active', item.key === key));
      refreshPlaybackView();
    });
  }

  function rebuildPlaybackGrid(urls, labels) {
    playbackGrid.innerHTML = '';
    playbackVideoEntries = [];
    const keys = Object.keys(urls || {});
    keys.forEach((key) => {
      const url = urls[key];
      if (!url || key === 'status') return;
      const wrap = document.createElement('article');
      wrap.className = 'video-card';
      wrap.dataset.recordKey = key;

      const title = document.createElement('h3');
      title.textContent = labels && labels[key] ? labels[key] : key;

      const video = document.createElement('video');
      video.src = url;
      video.muted = true;
      video.playsInline = true;

      const note = document.createElement('div');
      note.className = 'video-note';
      note.textContent = '点击卡片后，该视频会成为同步基准和当前时间显示来源。';

      wrap.appendChild(title);
      wrap.appendChild(video);
      wrap.appendChild(note);
      playbackGrid.appendChild(wrap);
      playbackVideoEntries.push({ key, wrapEl: wrap, videoEl: video });
      bindPlaybackVideoEvents(video, key, wrap);
    });

    if (!playbackPrimaryKey && playbackVideoEntries.length > 0) {
      playbackPrimaryKey = playbackVideoEntries[0].key;
    }
    playbackVideoEntries.forEach((item) => item.wrapEl.classList.toggle('active', item.key === playbackPrimaryKey));
  }

  async function setPlaybackPlaying(playing) {
    const videos = allPlaybackVideos();
    if (videos.length < 1) return;
    videos.forEach((video) => {
      video.playbackRate = playbackRate;
    });
    if (playing) {
      for (const video of videos) {
        try {
          await video.play();
        } catch (_) {
          video.pause();
        }
      }
    } else {
      videos.forEach((video) => video.pause());
    }
    refreshPlaybackView();
  }

  function seekPlaybackBy(deltaSec) {
    const sourceVideo = currentPlaybackVideo();
    if (!sourceVideo) return;
    const duration = Number.isFinite(sourceVideo.duration)
      ? sourceVideo.duration
      : ((Number(lastRecording && lastRecording.durationMs) || 0) / 1000);
    const maxTime = Math.max(0, Number.isFinite(duration) ? duration : 0);
    const nextTime = Math.max(0, Math.min(maxTime, (sourceVideo.currentTime || 0) + deltaSec));
    sourceVideo.currentTime = nextTime;
    syncPlaybackVideos(sourceVideo);
    refreshPlaybackView();
  }

  function setPlaybackRate(nextRate) {
    playbackRate = nextRate;
    allPlaybackVideos().forEach((video) => {
      video.playbackRate = playbackRate;
    });
    refreshPlaybackView();
  }

  function cyclePlaybackRate() {
    const idx = playbackRates.indexOf(playbackRate);
    const nextIdx = idx >= 0 ? ((idx + 1) % playbackRates.length) : playbackRates.indexOf(1.0);
    setPlaybackRate(playbackRates[nextIdx]);
  }

  function normalizeRecordingPayload(payload) {
    return {
      savedAtMs: Number(payload && payload.savedAtMs) || Date.now(),
      startedAtMs: Number(payload && payload.startedAtMs) || Date.now(),
      durationMs: Number(payload && payload.durationMs) || 0,
      frameCount: Number(payload && payload.frameCount) || (Array.isArray(payload && payload.statusFrames) ? payload.statusFrames.length : 0),
      statusFrames: Array.isArray(payload && payload.statusFrames) ? payload.statusFrames : [],
      videoLabels: (payload && payload.videoLabels && typeof payload.videoLabels === 'object') ? payload.videoLabels : {},
      urls: (payload && payload.urls && typeof payload.urls === 'object') ? payload.urls : {},
      sessionMeta: (payload && payload.sessionMeta && typeof payload.sessionMeta === 'object') ? payload.sessionMeta : {},
      folder: typeof (payload && payload.folder) === 'string' ? payload.folder : ''
    };
  }

  function applyRecording(recording, sourceText) {
    lastRecording = normalizeRecordingPayload(recording);
    document.title = lastRecording.folder ? `Vision Playback - ${lastRecording.folder}` : 'Vision Playback';
    sourceLabel.textContent = sourceText || (lastRecording.folder ? `来源：已保存目录 ${lastRecording.folder}` : '来源：当前会话录制');
    renderMetaPills(lastRecording);
    rebuildPlaybackGrid(lastRecording.urls || {}, lastRecording.videoLabels || {});
    if (playbackVideoEntries.length < 1) {
      playbackGrid.innerHTML = '<div class="panel empty-state">当前录制没有可播放的视频，只保留了状态数据。</div>';
    }
    refreshPlaybackView();
  }

  async function loadSavedFolder(folder) {
    const j = await receiverCore.fetchJsonNoStore(`/api/recordings/load?folder=${encodeURIComponent(folder)}`);
    if (!j.ok) throw new Error(j.error || 'load failed');
    const loadedStatus = j.status || {};
    applyRecording({
      savedAtMs: Date.now(),
      startedAtMs: Number(loadedStatus.recorded_at_ms) || Date.now(),
      durationMs: Number(loadedStatus.duration_ms) || 0,
      frameCount: Number(loadedStatus.frame_count) || ((Array.isArray(loadedStatus.statuses) ? loadedStatus.statuses.length : 0)),
      statusFrames: Array.isArray(loadedStatus.statuses) ? loadedStatus.statuses : [],
      videoLabels: (j.meta && j.meta.video_labels && typeof j.meta.video_labels === 'object')
        ? j.meta.video_labels
        : ((loadedStatus.video_labels && typeof loadedStatus.video_labels === 'object') ? loadedStatus.video_labels : {}),
      urls: Object.assign({}, j.videos || {}),
      sessionMeta: loadedStatus.session_meta || ((j.meta && j.meta.session_meta) ? j.meta.session_meta : {}),
      folder: j.folder || folder
    }, `来源：已保存目录 ${folder}`);
  }

  function listenTransferChannel(channelId) {
    if (!channelId) return;
    const channelName = `pc_receiver_playback_${channelId}`;
    transferChannel = new BroadcastChannel(channelName);
    transferChannel.onmessage = (ev) => {
      const data = ev && ev.data;
      if (!data || data.type !== 'payload') return;
      applyRecording(data.payload || {}, '来源：当前会话录制');
      transferChannel.postMessage({ type: 'payload_received' });
    };
    sourceLabel.textContent = '来源：等待主页面传输当前会话录制...';
    rawStatus.textContent = '等待主页面传输回放数据...';
    transferChannel.postMessage({ type: 'ready' });
  }

  playbackToggleBtn.addEventListener('click', async () => {
    const video = currentPlaybackVideo();
    if (!video) return;
    await setPlaybackPlaying(video.paused);
  });
  playbackRestartBtn.addEventListener('click', () => {
    allPlaybackVideos().forEach((video) => {
      video.pause();
      video.currentTime = 0;
    });
    refreshPlaybackView();
  });
  playbackBackwardBtn.addEventListener('click', () => {
    seekPlaybackBy(-1.0);
  });
  playbackForwardBtn.addEventListener('click', () => {
    seekPlaybackBy(1.0);
  });
  playbackSpeedBtn.addEventListener('click', () => {
    cyclePlaybackRate();
  });
  refreshBtn.addEventListener('click', () => {
    const folder = params.get('folder') || '';
    if (folder) {
      loadSavedFolder(folder).catch((err) => {
        rawStatus.textContent = `加载回放失败: ${err}`;
      });
      return;
    }
    window.location.reload();
  });

  window.addEventListener('beforeunload', () => {
    if (transferChannel) {
      try {
        transferChannel.close();
      } catch (_) {}
    }
  });

  const folder = params.get('folder') || '';
  const channel = params.get('channel') || '';
  if (folder) {
    loadSavedFolder(folder).catch((err) => {
      rawStatus.textContent = `加载回放失败: ${err}`;
      sourceLabel.textContent = `来源：目录 ${folder} 加载失败`;
    });
  } else if (channel) {
    listenTransferChannel(channel);
  } else {
    rawStatus.textContent = '没有提供回放来源，请从主页面重新打开录制回放。';
    sourceLabel.textContent = '来源：无';
  }
})();
