#!/usr/bin/env bash
export DISPLAY="${DISPLAY:-:0}"
export EGL_PLATFORM=x11
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

pushd "$SCRIPT_DIR" > /dev/null
trap 'popd > /dev/null' EXIT

"$SCRIPT_DIR/pnr/bin/python" \
    "$SCRIPT_DIR/run_batch.py" \
    --maze-seed 42 \
    --seed 42 \
    --workers 15 \
    --max-steps 1000 \
    --gpus 0 1 \
    "$@"