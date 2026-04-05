#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TOOLS_DIR="$ROOT_DIR/tools"
F1_DASHBOARD_DIR="$TOOLS_DIR/f1_dashboard"
PC_RECEIVER_JS_DIR="$TOOLS_DIR/pc_receiver_js"
LOCAL_COMPUTE_DIR="$TOOLS_DIR/pc_receiver_local_compute"

usage() {
  cat <<'EOF'
Usage:
  tools/build_all_tools.sh setup
  tools/build_all_tools.sh wasm-opencv
  tools/build_all_tools.sh wasm
  tools/build_all_tools.sh all

Commands:
  setup        Install npm dependencies for tools projects.
  wasm-opencv  Build the local Emscripten-target OpenCV toolchain.
  wasm         Build vision_pipeline.js/.wasm using the local toolchain.
  all          Run setup + wasm-opencv + wasm.
EOF
}

run_setup() {
  echo "[1/3] Installing npm dependencies for tools/f1_dashboard"
  (cd "$F1_DASHBOARD_DIR" && npm ci)

  echo "[2/3] Installing npm dependencies for tools/pc_receiver_local_compute"
  (cd "$LOCAL_COMPUTE_DIR" && npm ci)

  if [ -f "$PC_RECEIVER_JS_DIR/package-lock.json" ]; then
    echo "[3/3] tools/pc_receiver_js has no declared npm dependencies, skipping npm ci"
  else
    echo "[3/3] tools/pc_receiver_js has no package-lock.json, skipping npm ci"
  fi
}

run_wasm_opencv() {
  echo "Building local OpenCV-for-WASM toolchain"
  "$LOCAL_COMPUTE_DIR/build_opencv_wasm.sh"
}

run_wasm() {
  echo "Building local-compute WASM artifacts"
  "$LOCAL_COMPUTE_DIR/build_wasm.sh"
}

cmd="${1:-all}"

case "$cmd" in
  setup)
    run_setup
    ;;
  wasm-opencv)
    run_wasm_opencv
    ;;
  wasm)
    run_wasm
    ;;
  all)
    run_setup
    run_wasm_opencv
    run_wasm
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown command: $cmd" >&2
    usage >&2
    exit 1
    ;;
esac
