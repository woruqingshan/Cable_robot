#!/usr/bin/env bash
set -euo pipefail
VLA_SCRIPT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VLA_REPO_ROOT="$(cd -- "$VLA_SCRIPT_ROOT/../../.." && pwd)"
export PYTHONPATH="$VLA_REPO_ROOT/vla_protocol/src:$VLA_REPO_ROOT/vla_server/src${PYTHONPATH:+:$PYTHONPATH}"
VLA_PYTHON="${VLA_PYTHON:-python3}"
