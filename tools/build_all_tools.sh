#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TOOLS_DIR="$ROOT_DIR/tools"
PC_RECEIVER_JS_DIR="$TOOLS_DIR/pc_receiver_js"

usage() {
  cat <<'EOF'
Usage:
  tools/build_all_tools.sh setup
  tools/build_all_tools.sh all

Commands:
  setup        Install npm dependencies for tools projects.
  all          Run setup.
EOF
}

run_setup() {
  if [ -f "$PC_RECEIVER_JS_DIR/package-lock.json" ]; then
    echo "Installing npm dependencies for tools/pc_receiver_js"
    (cd "$PC_RECEIVER_JS_DIR" && npm ci)
  else
    echo "tools/pc_receiver_js has no package-lock.json, skipping npm ci"
  fi
}

cmd="${1:-all}"

case "$cmd" in
  setup)
    run_setup
    ;;
  all)
    run_setup
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
