#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
BRIDGE_DIR="$ROOT_DIR/tools/pc_receiver_local_compute/wasm_bridge"
BUILD_DIR="$BRIDGE_DIR/build_wasm"
OUTPUT_DIR="$ROOT_DIR/tools/pc_receiver_js/public/wasm"
SYNC_META_PATH="$OUTPUT_DIR/vision_pipeline.sync.json"

if ! command -v emcmake >/dev/null 2>&1 || ! command -v emcc >/dev/null 2>&1; then
  echo "Emscripten toolchain not found. Please install/activate emsdk first."
  exit 1
fi

mkdir -p "$OUTPUT_DIR"
emcmake cmake -S "$BRIDGE_DIR" -B "$BUILD_DIR" -DOpenCV_DIR="${OpenCV_DIR:-}"
cmake --build "$BUILD_DIR" -j"$(nproc)"

cp -f "$BUILD_DIR/vision_pipeline.js" "$OUTPUT_DIR/vision_pipeline.js"
cp -f "$BUILD_DIR/vision_pipeline.wasm" "$OUTPUT_DIR/vision_pipeline.wasm"
node "$ROOT_DIR/tools/pc_receiver_local_compute/wasm_sync_meta.js" \
  --root "$ROOT_DIR" \
  --write "$SYNC_META_PATH" >/dev/null

echo "WASM artifacts copied to $OUTPUT_DIR"
