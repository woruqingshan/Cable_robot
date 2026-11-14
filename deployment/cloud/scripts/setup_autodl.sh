#!/usr/bin/env bash
# IDC-VLA / AutoDL first-time GPU preparation.
# Run from the Cable_robot checkout: bash deployment/cloud/scripts/setup_autodl.sh
# Safe to re-run on a matching checkout/environment; does NOT start an inference server.
set -Eeuo pipefail

REPO_DIR="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$REPO_DIR" || ! -f "$REPO_DIR/vla_protocol/pyproject.toml" || ! -f "$REPO_DIR/vla_server/pyproject.toml" ]]; then
  echo '[ERROR] Run this script from a Cable_robot Git checkout containing vla_protocol and vla_server.' >&2
  exit 2
fi

if [[ -z "${VLA_BASE:-}" ]]; then
  if [[ -d /root/autodl-tmp ]]; then
    VLA_BASE=/root/autodl-tmp/idc-vla
  else
    echo '[ERROR] Set VLA_BASE to a persistent directory with enough free space (e.g. /data/idc-vla).' >&2
    exit 2
  fi
fi
VLA_BASE="$(realpath -m "$VLA_BASE")"
VENV_DIR="${VLA_VENV:-$VLA_BASE/venvs/vla-model}"
LEROBOT_DIR="${VLA_LEROBOT_DIR:-$VLA_BASE/third_party/lerobot}"
MODEL_DIR="${VLA_MODEL_DIR:-$VLA_BASE/models/smolvla_base}"
RUN_DIR="$VLA_BASE/runs/bootstrap"
LEROBOT_REF="${VLA_LEROBOT_REF:-ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2}"
TORCH_VERSION="${VLA_TORCH_VERSION:-2.8.0}"
TORCHVISION_VERSION="${VLA_TORCHVISION_VERSION:-0.23.0}"
CUDA_WHEEL="${VLA_CUDA_WHEEL:-cu128}"
MODE=install
SKIP_MODEL=0
SKIP_LEROBOT_INSTALL=0

usage() {
  cat <<'HELP'
Usage: bash setup_autodl_vla.sh [--check-only] [--skip-model] [--skip-lerobot-install]

Environment:
  VLA_BASE          persistent data root (default /root/autodl-tmp/idc-vla)
  VLA_VENV          existing/new Python 3.12 environment directory
  VLA_LEROBOT_DIR   existing/new LeRobot checkout
  VLA_MODEL_DIR     SmolVLA checkpoint directory
  VLA_LEROBOT_REF   exact LeRobot SHA (default reviewed reference)
  VLA_SMOL_REV      exact Hugging Face checkpoint SHA (optional; resolved when absent)

Notes:
  - Requires an already-cloned Cable_robot checkout and functioning network.
  - Does not install OS/GPU drivers, alter Git branches, or run a real model.
  - Does not replace a conflicting existing LeRobot checkout or checkpoint revision.
  - For a running installation, finish/stop that command before rerunning this script.
HELP
}
while (( $# )); do
  case "$1" in
    --check-only) MODE=check ;;
    --skip-model) SKIP_MODEL=1 ;;
    --skip-lerobot-install) SKIP_LEROBOT_INSTALL=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

log() { printf '\n[%s] %s\n' "$(date '+%H:%M:%S')" "$*"; }
die() { echo "[ERROR] $*" >&2; exit 2; }
trap 'echo "[ERROR] Failed at line $LINENO: $BASH_COMMAND" >&2' ERR

command -v git >/dev/null || die 'git is required'
command -v nvidia-smi >/dev/null || die 'nvidia-smi is missing: check the GPU image'
log 'GPU inventory'; nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv
log 'Disk inventory'; df -h "$REPO_DIR" "$(dirname "$VLA_BASE")" | awk '!seen[$1]++'

if [[ "$MODE" == check ]]; then
  echo "Repo: $REPO_DIR"
  echo "Data root: $VLA_BASE"
  if [[ -x "$VENV_DIR/bin/python" ]]; then
    "$VENV_DIR/bin/python" - <<'PY'
import sys
print('Python:', sys.version)
try:
 import torch
 print('Torch:', torch.__version__, 'CUDA:', torch.version.cuda, 'available:', torch.cuda.is_available())
except Exception as e: print('Torch import failed:', repr(e))
try:
 import lerobot
 print('LeRobot:', lerobot.__file__)
except Exception as e: print('LeRobot import failed:', repr(e))
PY
  else
    echo 'No VLA Python environment created yet.'
  fi
  [[ -d "$LEROBOT_DIR/.git" ]] && git -C "$LEROBOT_DIR" log -1 --format='LeRobot HEAD: %H' || true
  [[ -f "$MODEL_DIR/.idc-vla-revision" ]] && cat "$MODEL_DIR/.idc-vla-revision" || true
  exit 0
fi

mkdir -p "$VLA_BASE/venvs" "$VLA_BASE/third_party" "$VLA_BASE/models" "$VLA_BASE/cache/huggingface" "$VLA_BASE/cache/pip" "$RUN_DIR"
export HF_HOME="$VLA_BASE/cache/huggingface"
export HF_HUB_CACHE="$HF_HOME/hub"
export PIP_CACHE_DIR="$VLA_BASE/cache/pip"
export PYTHONNOUSERSITE=1

# Python env: reuse explicitly matching venv/conda env; do not tamper with system Python.
if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  if [[ -e "$VENV_DIR" ]]; then die "Non-Python environment directory exists: $VENV_DIR"; fi
  if command -v python3.12 >/dev/null; then
    PYBASE="$(command -v python3.12)"
  elif python -c 'import sys; assert sys.version_info[:2] == (3,12)' >/dev/null 2>&1; then
    PYBASE="$(command -v python)"
  else
    die 'Python 3.12 not found. Select the AutoDL Python 3.12 image or install Python 3.12 first.'
  fi
  log "Creating Python 3.12 venv at $VENV_DIR"
  "$PYBASE" -m venv "$VENV_DIR" || die 'venv creation failed; check python3.12-venv availability'
fi
PY="$VENV_DIR/bin/python"
"$PY" -c 'import sys; assert sys.version_info[:2] == (3, 12), sys.version' || die 'Existing VLA environment is not Python 3.12'
log "Python: $($PY --version)"
"$PY" -m pip --version >/dev/null || die 'pip missing in the model environment'
"$PY" -m pip install --upgrade pip setuptools wheel

# PyTorch: only install if missing; incompatible existing torch is a stop, not silently overwritten.
if "$PY" -c 'import torch' >/dev/null 2>&1; then
  log 'PyTorch already installed; verifying requested version/CUDA'
else
  log "Installing PyTorch $TORCH_VERSION ($CUDA_WHEEL)"
  "$PY" -m pip install "torch==$TORCH_VERSION" "torchvision==$TORCHVISION_VERSION" \
    --index-url "https://download.pytorch.org/whl/$CUDA_WHEEL"
fi
"$PY" - "$TORCH_VERSION" "$CUDA_WHEEL" <<'PY'
import sys, torch
expected, wheel = sys.argv[1:]
print('Torch:', torch.__version__, 'Torch CUDA:', torch.version.cuda, 'GPU visible:', torch.cuda.is_available())
if torch.__version__.split('+')[0] != expected: raise SystemExit('Unexpected Torch version; use a fresh venv or explicitly review dependencies')
if not torch.cuda.is_available(): raise SystemExit('Torch cannot access GPU; check driver and image first')
if not torch.version.cuda or not torch.version.cuda.startswith('12.8' if wheel == 'cu128' else '12.6'):
    raise SystemExit('CUDA wheel/runtime differs from the requested target')
print('GPU:', torch.cuda.get_device_name(0))
PY

# Framework checkout: preserve any existing checkout, reject wrong pinned revision.
if [[ -d "$LEROBOT_DIR/.git" ]]; then
  log "Reusing LeRobot checkout: $LEROBOT_DIR"
  actual="$(git -C "$LEROBOT_DIR" rev-parse HEAD)"
  [[ "$actual" == "$LEROBOT_REF" ]] || die "LeRobot HEAD is $actual, expected $LEROBOT_REF. No automatic reset performed."
  if [[ -n "$(git -C "$LEROBOT_DIR" status --porcelain --untracked-files=no)" ]]; then
    die 'LeRobot tracked files are modified; refusing to overwrite.'
  fi
elif [[ -e "$LEROBOT_DIR" ]]; then
  die "Directory exists and is not a Git checkout: $LEROBOT_DIR"
else
  log 'Cloning official LeRobot independently'
  git clone https://github.com/huggingface/lerobot.git "$LEROBOT_DIR"
  git -C "$LEROBOT_DIR" checkout --detach "$LEROBOT_REF"
fi
if [[ "$SKIP_LEROBOT_INSTALL" == 0 ]]; then
  log 'Installing LeRobot SmolVLA dependencies (editable)'
  "$PY" -m pip install -e "$LEROBOT_DIR[smolvla]"
else
  log 'Skipping LeRobot pip install by request'
fi

log 'Installing Cable_robot Python packages (GPU side only; no ROS)'
"$PY" -m pip install -e "$REPO_DIR/vla_protocol" -e "$REPO_DIR/vla_server"
"$PY" -m pip check
"$PY" - "$TORCH_VERSION" <<'PYTORCH'
import sys, torch
expected=sys.argv[1]
if torch.__version__.split('+')[0] != expected or not torch.cuda.is_available():
    raise SystemExit('LeRobot or app dependencies changed the Torch version or broke CUDA; check pip freeze before continuing')
print('Final Torch GPU check OK:', torch.__version__, torch.cuda.get_device_name(0))
PYTORCH

# Verify installed imports actually point to the independent checkout.
"$PY" - "$LEROBOT_DIR" <<'PY'
import sys, pathlib, torch
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
import lerobot, vla_protocol, vla_server
root=pathlib.Path(sys.argv[1]).resolve()
path=pathlib.Path(lerobot.__file__).resolve()
if not path.is_relative_to(root): raise SystemExit('LeRobot import resolved outside the pinned checkout: ' + str(path))
if not torch.cuda.is_available(): raise SystemExit('GPU disappeared after dependency install')
print('Imports OK. LeRobot:', path, 'Torch:', torch.__version__)
PY

if [[ "$SKIP_MODEL" == 0 ]]; then
  log 'Resolving and downloading official SmolVLA checkpoint'
  "$PY" -m pip install 'huggingface-hub>=1.31,<3'
  # Exact commit from user, existing receipt, or official Hub API.
  if [[ -z "${VLA_SMOL_REV:-}" && -f "$MODEL_DIR/.idc-vla-revision" ]]; then
    VLA_SMOL_REV="$(cat "$MODEL_DIR/.idc-vla-revision")"
  fi
  if [[ -z "${VLA_SMOL_REV:-}" ]]; then
    VLA_SMOL_REV="$("$PY" -c 'from huggingface_hub import HfApi; print(HfApi().model_info("lerobot/smolvla_base").sha)')"
  fi
  [[ "$VLA_SMOL_REV" =~ ^[0-9a-f]{40}$ ]] || die 'Model revision must be an exact 40-character SHA'
  if [[ -f "$MODEL_DIR/.idc-vla-revision" && "$(cat "$MODEL_DIR/.idc-vla-revision")" != "$VLA_SMOL_REV" ]]; then
    die 'Existing model directory is pinned to a different revision; refusing overwrite.'
  fi
  "$PY" - "$MODEL_DIR" "$VLA_SMOL_REV" <<'PY'
import sys
from pathlib import Path
from huggingface_hub import snapshot_download
folder, rev=Path(sys.argv[1]),sys.argv[2]
folder.mkdir(parents=True,exist_ok=True)
snapshot_download(repo_id='lerobot/smolvla_base', revision=rev, local_dir=str(folder))
(folder/'.idc-vla-revision').write_text(rev+'\n')
print('Model downloaded and pinned:', folder, rev)
PY
else
  log 'Skipping model download by request'
fi

log 'Saving bootstrap environment record'
"$PY" -m pip freeze > "$RUN_DIR/pip-freeze.txt"
"$PY" - <<'PY' > "$RUN_DIR/python-torch.txt"
import sys, torch
print('Python:', sys.version)
print('Torch:', torch.__version__)
print('Torch CUDA:', torch.version.cuda)
print('GPU:', torch.cuda.get_device_name(0))
PY
nvidia-smi > "$RUN_DIR/nvidia-smi.txt"
git -C "$REPO_DIR" rev-parse HEAD > "$RUN_DIR/cable-robot-sha.txt"
git -C "$LEROBOT_DIR" rev-parse HEAD > "$RUN_DIR/lerobot-sha.txt"
[[ -f "$MODEL_DIR/.idc-vla-revision" ]] && cp "$MODEL_DIR/.idc-vla-revision" "$RUN_DIR/smolvla-sha.txt" || true
printf 'BASE=%s\nVENV=%s\nLEROBOT=%s\nMODEL=%s\n' "$VLA_BASE" "$VENV_DIR" "$LEROBOT_DIR" "$MODEL_DIR" > "$RUN_DIR/paths.txt"

log 'SETUP COMPLETE (dependencies and checkpoint only; real inference NOT yet tested)'
echo "Activate: source '$VENV_DIR/bin/activate'"
echo "Check:    bash '$REPO_DIR/deployment/cloud/scripts/setup_autodl.sh' --check-only"
echo "Evidence: $RUN_DIR"
echo 'Next: verify official checkpoint loading + genuine predict_action_chunk, then fill the repository cloud manifest/config.'
