let wasmModuleState = 'not_loaded';
let wasmModuleError = '';

function resultWithDefaults(status) {
  return {
    leftBoundary: [],
    rightBoundary: [],
    ipmLeftBoundary: [],
    ipmRightBoundary: [],
    centerline: [],
    ipmCenterline: [],
    leftBoundaryCorner: [],
    rightBoundaryCorner: [],
    ipmLeftBoundaryCorner: [],
    ipmRightBoundaryCorner: [],
    leftAuxiliaryLine: [],
    rightAuxiliaryLine: [],
    centerlineCurvature: [],
    leftBoundaryCurvature: [],
    rightBoundaryCurvature: [],
    leftBoundaryAngleCos: [],
    rightBoundaryAngleCos: [],
    binaryProbePoints: [],
    summary: '',
    limitations: [],
    recommendation: '',
    workerThreshold: Number(status.otsu_threshold) || 0,
    rowSamples: 0,
    wasm_state: wasmModuleState,
    wasm_error: '',
    valid: false,
    line_error: 0,
    ipm_track_valid: false,
    ipm_track_index: -1,
    ipm_track_point: [0, 0]
  };
}

async function ensureVisionWasmModule() {
  if (self.__visionWasmModule) {
    return self.__visionWasmModule;
  }
  if (wasmModuleState === 'failed') {
    throw new Error(wasmModuleError || 'vision wasm module load failed');
  }
  if (wasmModuleState === 'loading') {
    throw new Error('vision wasm module is still loading');
  }

  wasmModuleState = 'loading';
  try {
    importScripts('/wasm/vision_pipeline.js');
    if (typeof self.createVisionPipelineModule !== 'function') {
      throw new Error('createVisionPipelineModule is not available');
    }
    self.__visionWasmModule = await self.createVisionPipelineModule({
      locateFile(path) {
        return `/wasm/${path}`;
      }
    });
    wasmModuleState = 'ready';
    return self.__visionWasmModule;
  } catch (err) {
    wasmModuleState = 'failed';
    wasmModuleError = String(err && err.message ? err.message : err);
    throw err;
  }
}

function rgbaToBgrBuffer(imageData) {
  const src = imageData && imageData.data;
  const total = imageData.width * imageData.height;
  const out = new Uint8Array(total * 3);
  if (!src) return out;
  let si = 0;
  let di = 0;
  for (let i = 0; i < total; i += 1) {
    const r = src[si];
    const g = src[si + 1];
    const b = src[si + 2];
    out[di] = b;
    out[di + 1] = g;
    out[di + 2] = r;
    si += 4;
    di += 3;
  }
  return out;
}

function invokeVisionBridge(module, imageData, status) {
  const bgr = rgbaToBgrBuffer(imageData);
  const ptr = module._malloc(bgr.length);
  try {
    module.HEAPU8.set(bgr, ptr);
    const tsMs = BigInt(Math.trunc(Number(status.ts_ms) || 0));
    const ok = module._web_vision_abi_process_bgr(
      ptr,
      imageData.width | 0,
      imageData.height | 0,
      1,
      bgr.length >>> 0,
      Number(status.left_current_count) || 0,
      Number(status.right_current_count) || 0,
      Number(status.left_target_count) || 0,
      Number(status.right_target_count) || 0,
      Number(status.left_filtered_count) || 0,
      Number(status.right_filtered_count) || 0,
      Number(status.base_speed) || 0,
      Number(status.adjusted_base_speed) || 0,
      Number(status.otsu_threshold) || 0,
      tsMs
    );
    const jsonPtr = module._web_vision_abi_last_result_json();
    const jsonText = module.UTF8ToString(jsonPtr);
    const parsed = jsonText ? JSON.parse(jsonText) : {};
    parsed.valid = !!parsed.valid && !!ok;
    parsed.wasm_state = wasmModuleState;
    parsed.wasm_error = '';
    return parsed;
  } finally {
    module._free(ptr);
  }
}

self.onmessage = async (ev) => {
  const payload = ev.data || {};
  const imageData = payload.imageData;
  const status = payload.status || {};
  const result = resultWithDefaults(status);

  if (!imageData || imageData.width <= 0 || imageData.height <= 0) {
    result.summary = 'Worker did not receive a valid gray frame.';
    result.limitations = [
      '当前没有可用于本地复算的灰度图输入。',
      '因此浏览器侧无法调用同源算法模块。'
    ];
    result.recommendation = '先确保主板正在发送灰度图。';
    self.postMessage(result);
    return;
  }

  try {
    const wasmModule = await ensureVisionWasmModule();
    const bridgeResult = invokeVisionBridge(wasmModule, imageData, status);
    Object.assign(result, bridgeResult);
    result.wasm_state = wasmModuleState;
    result.summary = bridgeResult.summary || bridgeResult.status_message || 'WASM bridge executed.';
  } catch (err) {
    const errorText = String(err && err.message ? err.message : err);
    result.wasm_state = wasmModuleState;
    result.wasm_error = errorText;
    result.summary = wasmModuleState === 'failed'
      ? 'WASM module failed to initialize.'
      : 'WASM module loaded but execution failed.';
    result.limitations = [
      '浏览器端目标是调用与车端同源的 C++ 算法 WebAssembly 模块。',
      '当前失败点更可能是 worker/ABI 调用链，而不是页面骨架本身。'
    ];
    result.recommendation = `检查 wasm_error 并按需重新构建 wasm 后再刷新页面。当前错误：${errorText}`;
  }

  self.postMessage(result);
};
