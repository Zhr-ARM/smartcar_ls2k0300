(() => {
  function nearestFrame(frames, playbackMs) {
    if (!Array.isArray(frames) || frames.length < 1) return null;
    let lo = 0;
    let hi = frames.length - 1;
    while (lo < hi) {
      const mid = Math.floor((lo + hi) / 2);
      const midTs = Number(frames[mid].client_ts_ms) || 0;
      if (midTs < playbackMs) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    const next = frames[lo];
    const prev = frames[Math.max(0, lo - 1)];
    const nextDiff = Math.abs((Number(next && next.client_ts_ms) || 0) - playbackMs);
    const prevDiff = Math.abs((Number(prev && prev.client_ts_ms) || 0) - playbackMs);
    return prevDiff <= nextDiff ? prev : next;
  }

  function createReceiverFrameSource(receiverCore) {
    let mode = 'live';
    let replayFrames = {};
    let replayStartMs = 0;
    let replayNowMs = 0;

    return {
      setLive() {
        mode = 'live';
        replayFrames = {};
        replayStartMs = 0;
        replayNowMs = 0;
      },
      setReplay(recording, playbackMs) {
        mode = 'replay';
        replayFrames = (recording && recording.source_frames) || {};
        replayStartMs = Number(recording && recording.recorded_at_ms) || 0;
        replayNowMs = Number(playbackMs) || 0;
      },
      setReplayTime(playbackMs) {
        replayNowMs = Number(playbackMs) || 0;
      },
      urlForMode(frameMode) {
        if (mode === 'live') return receiverCore.frameUrlForMode(frameMode);
        const frames = replayFrames[frameMode] || [];
        const frame = nearestFrame(frames, replayStartMs + replayNowMs);
        return frame && frame.url ? frame.url : '';
      },
      frameMetaForMode(frameMode) {
        if (mode === 'live') return null;
        const frames = replayFrames[frameMode] || [];
        return nearestFrame(frames, replayStartMs + replayNowMs);
      },
      isReplay() {
        return mode === 'replay';
      }
    };
  }

  window.ReceiverFrameSource = { createReceiverFrameSource };
})();
