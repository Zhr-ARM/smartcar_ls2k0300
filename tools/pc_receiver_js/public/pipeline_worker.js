self.onmessage = (ev) => {
  const payload = ev.data || {};
  const imageData = payload.imageData;
  const status = payload.status || {};
  const mode = payload.mode || 'gray';

  const result = {
    overlayPoints: [],
    summary: '',
    limitations: [],
    recommendation: ''
  };

  if (imageData && imageData.width > 0 && imageData.height > 0) {
    const w = imageData.width;
    const h = imageData.height;
    result.overlayPoints.push([Math.floor(w / 2), Math.floor(h - 1)]);
    if (typeof status.otsu_threshold === 'number') {
      const probeY = Math.max(0, Math.min(h - 1, Math.floor(h * 0.7)));
      result.overlayPoints.push([Math.floor(w * 0.5), probeY]);
    }
  }

  result.summary = 'Local pipeline workspace is active. Raw frame and raw status are available in the browser worker.';
  result.limitations = [
    '当前 worker 还没有移植车端 maze/IPM/角点/辅助线/中线算法。',
    '因此它现在只是本地复算骨架，不会和车端现有效果完全一致。',
    '若要做到一模一样，最稳路径是把现有 C++ 视觉链抽成纯算法层并编译为 WebAssembly。'
  ];
  result.recommendation = `建议优先迁移顺序：OTSU -> maze trace -> IPM point transform -> boundary postprocess -> corner/aux line -> shifted centerline -> line_error。当前模式=${mode}`;

  self.postMessage(result);
};
