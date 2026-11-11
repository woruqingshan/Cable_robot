#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "$0")/common.sh"
exec "$VLA_PYTHON" "$VLA_SCRIPT_ROOT/bootstrap.py" download-model "$@"
