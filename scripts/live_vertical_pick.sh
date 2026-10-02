#!/usr/bin/env bash
# Fixed downward gripper pose through grasp, vertical lift, squeeze and release.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export OPENBLAS_NUM_THREADS=1
exec "$ROOT/scripts/python.sh" "$ROOT/scripts/run_candidate.py" gui \
    --profile config/vertical_pick.json \
    --replay-directory outputs/vertical_pick/reference "$@"
