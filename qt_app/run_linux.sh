#!/usr/bin/env bash
set -euo pipefail

QT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
BACKEND_ROOT=$(cd "$QT_ROOT/.." && pwd)
VENV_ROOT="$QT_ROOT/.venv-linux"
BOOTSTRAP_PYTHON=${PYTHON:-python3}
PREFLIGHT_ONLY=0

if [[ "${1:-}" == "--preflight-only" ]]; then
    PREFLIGHT_ONLY=1
    shift
fi

if ! "$BOOTSTRAP_PYTHON" -c 'import sys; raise SystemExit(0 if (3, 8) <= sys.version_info[:2] < (3, 13) else 1)'; then
    echo "Error: Python >=3.8,<3.13 is required" >&2
    exit 1
fi

if [[ ! -x "$VENV_ROOT/bin/python" ]]; then
    echo "Creating Linux desktop environment at $VENV_ROOT"
    "$BOOTSTRAP_PYTHON" -m venv "$VENV_ROOT"
fi

if ! "$VENV_ROOT/bin/python" -c 'import importlib.metadata as m; import PyQt6, flask, werkzeug, numpy, scipy, websocket, zeroconf, carm; raise SystemExit(0 if m.version("carm") >= "0.1.20260819" else 1)' >/dev/null 2>&1; then
    echo "Installing desktop runtime dependencies"
    PIP_DISABLE_PIP_VERSION_CHECK=1 "$VENV_ROOT/bin/python" -m pip install -r "$QT_ROOT/requirements.txt"
fi

if [[ -n "${CARM_SDK_ROOT:-}" ]]; then
    if [[ ! -f "$CARM_SDK_ROOT/pyproject.toml" ]]; then
        echo "Error: CARM_SDK_ROOT does not contain pyproject.toml: $CARM_SDK_ROOT" >&2
        exit 1
    fi
    echo "Installing pinned Carm SDK from $CARM_SDK_ROOT"
    PIP_DISABLE_PIP_VERSION_CHECK=1 "$VENV_ROOT/bin/python" -m pip install -e "$CARM_SDK_ROOT"
fi

"$VENV_ROOT/bin/python" "$QT_ROOT/preflight.py"
if [[ "$PREFLIGHT_ONLY" -eq 1 ]]; then
    exit 0
fi

cd "$BACKEND_ROOT"
exec "$VENV_ROOT/bin/python" qt_app/main.py "$@"
