(() => {
  function nowUrl(url) {
    const join = url.includes('?') ? '&' : '?';
    return `${url}${join}t=${Date.now()}`;
  }

  function nearestFrame(frames, playbackMs) {
    if (!Array.isArray(frames) || frames.length < 1) return null;
    let best = frames[0];
    let bestDiff = Number.POSITIVE_INFINITY;
    for (const frame of frames) {
      const diff = Math.abs((Number(frame.client_ts_ms) || 0) - playbackMs);
      if (diff < bestDiff) {
        best = frame;
        bestDiff = diff;
      }
    }
    return best;
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
        return frame && frame.url ? nowUrl(frame.url) : '';
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
