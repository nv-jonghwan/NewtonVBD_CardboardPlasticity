#!/usr/bin/env bash
set -euo pipefail
TRIAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export CARDBOARD_PYTHON="$TRIAL_ROOT/.workspace/mujoco-vbd-trial1/runtime/bin/python"
exec "$TRIAL_ROOT/scripts/python.sh" "$@"
