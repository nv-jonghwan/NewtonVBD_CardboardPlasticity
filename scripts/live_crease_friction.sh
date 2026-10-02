#!/usr/bin/env bash
# Compatibility entry point for the reviewed production candidate.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export OPENBLAS_NUM_THREADS=1
exec "$ROOT/scripts/python.sh" "$ROOT/scripts/run_candidate.py" gui "$@"
