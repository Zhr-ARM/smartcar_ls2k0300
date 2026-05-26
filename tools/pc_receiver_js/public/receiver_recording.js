(() => {
  const FRAME_MODES = ['gray', 'binary', 'rgb', 'roi64'];

  async function blobToBase64(blob) {
    const arrayBuffer = await blob.arrayBuffer();
    const bytes = new Uint8Array(arrayBuffer);
    let binary = '';
    for (let i = 0; i < bytes.length; i += 1) {
      binary += String.fromCharCode(bytes[i]);
    }
    return btoa(binary);
  }

  async function fetchFrameForRecording(mode, receiverCore) {
    const response = await fetch(receiverCore.frameUrlForMode(mode), { cache: 'no-store' });
    if (!response.ok) return null;
    const blob = await response.blob();
    return {
      mode,
      client_ts_ms: Date.now(),
      frame_id: -1,
      width: 0,
      height: 0,
      mime: blob.type || 'image/jpeg',
      data_b64: await blobToBase64(blob)
    };
  }

  function createSourceRecorder(receiverCore, getStatus) {
    let state = null;

    return {
      active() {
        return !!(state && state.active);
      },
      start() {
        state = {
          active: true,
          startedAtMs: Date.now(),
          statusFrames: [],
          sourceFrames: { gray: [], binary: [], rgb: [], roi64: [] }
        };
      },
      async captureTick() {
        if (!state || !state.active) return;
        const status = getStatus();
        state.statusFrames.push({
          client_ts_ms: Date.now(),
          status: JSON.parse(JSON.stringify(status || {}))
        });
        await Promise.all(FRAME_MODES.map(async (mode) => {
          const frame = await fetchFrameForRecording(mode, receiverCore);
          if (frame) state.sourceFrames[mode].push(frame);
        }));
      },
      stop() {
        if (!state) return null;
        state.active = false;
        const finished = state;
        state = null;
        return finished;
      }
    };
  }

  function buildPersistableSourceRecording(recording) {
    const savedAtMs = Date.now();
    const startedAtMs = Number(recording && recording.startedAtMs) || savedAtMs;
    const statusFrames = Array.isArray(recording && recording.statusFrames)
      ? recording.statusFrames
      : [];
    return {
      recorded_at_ms: startedAtMs,
      duration_ms: Math.max(0, savedAtMs - startedAtMs),
      frame_count: statusFrames.length,
      statuses: statusFrames,
      source_frames: (recording && recording.sourceFrames) || {},
      session_meta: {
        recording_kind: 'source_frames',
        recorded_views: [
          { key: 'gray', title: 'Gray source' },
          { key: 'binary', title: 'Binary source' },
          { key: 'rgb', title: 'RGB source' },
          { key: 'roi64', title: 'ROI64 source' }
        ]
      }
    };
  }

  function statusAt(recordingStatus, playbackMs) {
    const frames = recordingStatus && Array.isArray(recordingStatus.statuses)
      ? recordingStatus.statuses
      : [];
    if (frames.length < 1) return {};
    const startMs = Number(recordingStatus.recorded_at_ms) || Number(frames[0].client_ts_ms) || 0;
    const target = startMs + playbackMs;
    let best = frames[0];
    let bestDiff = Number.POSITIVE_INFINITY;
    for (const frame of frames) {
      const diff = Math.abs((Number(frame.client_ts_ms) || 0) - target);
      if (diff < bestDiff) {
        best = frame;
        bestDiff = diff;
      }
    }
    return best.status || {};
  }

  window.ReceiverRecording = {
    createSourceRecorder,
    buildPersistableSourceRecording,
    statusAt
  };
})();
