#!/usr/bin/env bash
# Fold-preserving display preview; physical tuning candidates are not promoted.
set -euo pipefail
PREVIEW_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PREVIEW_ROOT"
export OPENBLAS_NUM_THREADS=1
exec "$PREVIEW_ROOT/scripts/python.sh" "$PREVIEW_ROOT/scripts/live_isaac.py" \
  --quality robotiq-primitive --physics-device cuda:0 \
  --scene assets/demo_scene_robotiq_board4_flat_panels.usda \
  --solver-config outputs/local_rom/adaptive_jacobi24/run_config.json \
  --replay-directory outputs/local_rom/adaptive_jacobi24 \
  --small-bend-scale 16 --small-bend-knee .75 --small-bend-end 14.75 "$@"
