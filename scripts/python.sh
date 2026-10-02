#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export PXR_PLUGINPATH_NAME="$ROOT/schemas/cardboard/resources${PXR_PLUGINPATH_NAME:+:$PXR_PLUGINPATH_NAME}"
export WARP_CACHE_PATH="${WARP_CACHE_PATH:-$ROOT/.cache/warp}"
case "${1:-}" in
    */live_isaac.py|live_isaac.py|*/render_isaac.py|render_isaac.py|*/render_snapshot.py|render_snapshot.py|*/render_replay.py|render_replay.py)
        PYTHON_BIN="${CARDBOARD_ISAAC_PYTHON:-}"
        if [[ -z "$PYTHON_BIN" && -x "$ROOT/.workspace/toolchain/bin/isaac-python" ]]; then
            PYTHON_BIN="$ROOT/.workspace/toolchain/bin/isaac-python"
        elif [[ -z "$PYTHON_BIN" && -x "$ROOT/.venv-gui/bin/python" ]]; then
            PYTHON_BIN="$ROOT/.venv-gui/bin/python"
        fi
        HINT='Set CARDBOARD_ISAAC_PYTHON to an Isaac Sim 6.0.1 Python executable.'
        ;;
    *)
        PYTHON_BIN="${CARDBOARD_PYTHON:-}"
        if [[ -z "$PYTHON_BIN" && -x "$ROOT/.workspace/toolchain/bin/python" ]]; then
            PYTHON_BIN="$ROOT/.workspace/toolchain/bin/python"
        elif [[ -z "$PYTHON_BIN" && -x "$ROOT/.venv/bin/python" ]]; then
            PYTHON_BIN="$ROOT/.venv/bin/python"
        fi
        HINT='Set CARDBOARD_PYTHON or install requirements-lock.txt into .venv.'
        ;;
esac
if [[ -z "$PYTHON_BIN" || ! -x "$PYTHON_BIN" ]]; then
    printf 'Cardboard Python is unavailable. %s\n' "$HINT" >&2
    exit 2
fi
exec "$PYTHON_BIN" "$@"
