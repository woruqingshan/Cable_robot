
#!/usr/bin/env bash
# IDC-VLA Stage 2: GPU environment setup from uploaded local resources.
#
# No Git clone/fetch, no Hugging Face downloads,
# no system CUDA/driver installation.
#
# Run from the Cable_robot repository:
#   bash deployment/cloud/scripts/setup_autodl.sh
#
# Check only:
#   bash deployment/cloud/scripts/setup_autodl.sh --check-only

set -Eeuo pipefail

# ============================================================
# 1. Paths and versions
# ============================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"

BASE="${VLA_BASE:-/root/autodl-tmp/idc-vla}"
PYENV="${VLA_VENV:-$BASE/venvs/vla-model}"

LEROBOT="${VLA_LEROBOT_DIR:-$BASE/third_party/lerobot}"
MODEL="${VLA_MODEL_DIR:-$BASE/models/smolvla_base}"

EXPECTED_LEROBOT="${VLA_LEROBOT_REF:-ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2}"

PYPI_INDEX="${VLA_PYPI_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"
TORCH_INDEX="${VLA_TORCH_INDEX:-https://download.pytorch.org/whl/cu128}"

TORCH_VERSION="${VLA_TORCH_VERSION:-2.8.0}"
TV_VERSION="${VLA_TORCHVISION_VERSION:-0.23.0}"

LOGDIR="$BASE/runs/bootstrap"

CHECK_ONLY=0
INSTALL_TORCH=1

# ============================================================
# 2. Utility functions
# ============================================================

say() {
    printf '\n[%s] %s\n' "$(date '+%H:%M:%S')" "$*"
}

fail() {
    printf '[ERROR] %s\n' "$*" >&2
    exit 2
}

usage() {
    cat <<'HELP'
IDC-VLA GPU Environment Setup

Usage:
  bash deployment/cloud/scripts/setup_autodl.sh

Options:
  --check-only
      Verify the environment without installing packages.

  --skip-torch-install
      Never download PyTorch. Fail if it is missing.

  --help
      Show this help.

Environment variables:
  VLA_BASE
  VLA_VENV
  VLA_LEROBOT_DIR
  VLA_MODEL_DIR
  VLA_LEROBOT_REF
  VLA_PYPI_INDEX
  VLA_TORCH_INDEX
  VLA_TORCH_VERSION
  VLA_TORCHVISION_VERSION

This script does not download Git repositories or model weights.
It does not reset Git, modify source files, or execute the robot.
HELP
}

trap 'echo "[ERROR] Failed at line $LINENO: $BASH_COMMAND" >&2' ERR

# ============================================================
# 3. Parse arguments
# ============================================================

while (( $# )); do
    case "$1" in
        --check-only)
            CHECK_ONLY=1
            ;;
        --skip-torch-install)
            INSTALL_TORCH=0
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            fail "Unknown argument: $1"
            ;;
    esac
    shift
done

# ============================================================
# 4. Validate repository and uploaded resources
# ============================================================

say "Checking Cable_robot repository"

[[ -f "$REPO/vla_protocol/pyproject.toml" ]] ||
    fail "Missing vla_protocol in $REPO"

[[ -f "$REPO/vla_server/pyproject.toml" ]] ||
    fail "Missing vla_server in $REPO"

command -v git >/dev/null ||
    fail "Git is not installed"

command -v nvidia-smi >/dev/null ||
    fail "NVIDIA driver is not available"

say "Checking uploaded LeRobot source"

[[ -d "$LEROBOT/.git" ]] ||
    fail "LeRobot Git directory missing: $LEROBOT"

[[ -f "$LEROBOT/pyproject.toml" ]] ||
    fail "LeRobot pyproject.toml missing"

[[ -f "$LEROBOT/src/lerobot/policies/smolvla/modeling_smolvla.py" ]] ||
    fail "SmolVLA source code missing"

[[ "$EXPECTED_LEROBOT" =~ ^[a-f0-9]{40}$ ]] ||
    fail "Expected LeRobot revision must be a 40-character SHA"

ACTUAL_LEROBOT="$(git -C "$LEROBOT" rev-parse HEAD)"

say "LeRobot actual revision: $ACTUAL_LEROBOT"
say "LeRobot expected revision: $EXPECTED_LEROBOT"

[[ "$ACTUAL_LEROBOT" == "$EXPECTED_LEROBOT" ]] ||
    fail "LeRobot version mismatch. Restore or approve the intended revision before installation."

[[ -z "$(git -C "$LEROBOT" status --porcelain --untracked-files=no)" ]] ||
    fail "LeRobot tracked files are modified or missing. Check git status before installation."

say "Checking uploaded SmolVLA checkpoint"

[[ -f "$MODEL/config.json" ]] ||
    fail "Missing SmolVLA config.json"

[[ -f "$MODEL/model.safetensors" ]] ||
    fail "Missing SmolVLA model.safetensors"

[[ -f "$MODEL/policy_preprocessor.json" ]] ||
    fail "Missing policy_preprocessor.json"

[[ -f "$MODEL/policy_postprocessor.json" ]] ||
    fail "Missing policy_postprocessor.json"

REVISION_FILE="$BASE/models/smolvla_base_revision.txt"

[[ -f "$REVISION_FILE" ]] ||
    fail "Missing SmolVLA revision file: $REVISION_FILE"

MODEL_SHA="$(tr -d '\r\n' < "$REVISION_FILE")"

[[ "$MODEL_SHA" =~ ^[a-f0-9]{40}$ ]] ||
    fail "Invalid SmolVLA revision SHA"

say "SmolVLA revision: $MODEL_SHA"

# ============================================================
# 5. Check GPU and disk
# ============================================================

say "GPU information"

nvidia-smi \
    --query-gpu=name,memory.total,driver_version \
    --format=csv

say "Disk space"

df -h "$BASE"

# ============================================================
# 6. Create or reuse Python environment
# ============================================================

if [[ -x "$PYENV/bin/python" ]]; then

    say "Reusing Python environment: $PYENV"
    PY="$PYENV/bin/python"

else

    if (( CHECK_ONLY )); then
        fail "Python environment does not exist: $PYENV"
    fi

    command -v python3.12 >/dev/null ||
        fail "Python 3.12 is not available"

    [[ ! -e "$PYENV" ]] ||
        fail "Environment directory exists but is incomplete: $PYENV"

    say "Creating Python 3.12 environment"

    mkdir -p "$(dirname "$PYENV")"

    python3.12 -m venv "$PYENV"

    PY="$PYENV/bin/python"

fi

"$PY" -c '
import sys
assert sys.version_info[:2] == (3, 12), sys.version
print("Python:", sys.version)
' || fail "Python 3.12 required"

# ============================================================
# 7. Configure pip mirrors and caches
# ============================================================

export PIP_INDEX_URL="$PYPI_INDEX"
export PIP_DEFAULT_TIMEOUT=120
export PIP_RETRIES=8

export PIP_CACHE_DIR="$BASE/cache/pip"
export HF_HOME="$BASE/cache/huggingface"
export HF_HUB_CACHE="$HF_HOME/hub"

export PYTHONNOUSERSITE=1

if (( ! CHECK_ONLY )); then

    mkdir -p \
        "$LOGDIR" \
        "$PIP_CACHE_DIR" \
        "$HF_HOME"

    say "Python package index: $PIP_INDEX_URL"

    "$PY" -m pip install -U \
        pip setuptools wheel

fi

# ============================================================
# 8. Install or reuse PyTorch
# ============================================================

if (( ! CHECK_ONLY )); then

    if "$PY" -c "import torch" >/dev/null 2>&1; then

        say "Existing PyTorch found. Reusing installation."

    else

        (( INSTALL_TORCH )) ||
            fail "PyTorch missing and installation disabled"

        say "Installing PyTorch CUDA 12.8"

        "$PY" -m pip install \
            "torch==$TORCH_VERSION" \
            "torchvision==$TV_VERSION" \
            --index-url "$TORCH_INDEX" \
            --timeout 180 \
            --retries 10

    fi

fi

# ============================================================
# 9. Verify PyTorch and CUDA
# ============================================================

say "Checking PyTorch and CUDA"

"$PY" - "$TORCH_VERSION" <<'PY'
import sys
import torch

expected = sys.argv[1]

print("Torch:", torch.__version__)
print("CUDA:", torch.version.cuda)
print("CUDA available:", torch.cuda.is_available())

if torch.__version__.split("+")[0] != expected:
    raise SystemExit("Unexpected PyTorch version")

if not torch.cuda.is_available():
    raise SystemExit("CUDA unavailable")

print("GPU:", torch.cuda.get_device_name(0))
print("Compute capability:", torch.cuda.get_device_capability(0))
print("BF16 supported:", torch.cuda.is_bf16_supported())

if not torch.cuda.is_bf16_supported():
    print(
        "NOTICE: GPU does not support native BF16. "
        "Verify FP16/FP32 inference compatibility."
    )
PY

# ============================================================
# 10. Install local LeRobot and Cable_robot packages
# ============================================================

if (( ! CHECK_ONLY )); then

    say "Installing LeRobot from uploaded local source"

    "$PY" -m pip install \
        -e "$LEROBOT[smolvla]"

    say "Installing Cable_robot protocol and server"

    "$PY" -m pip install \
        -e "$REPO/vla_protocol" \
        -e "$REPO/vla_server"

fi

# ============================================================
# 11. Verify dependencies
# ============================================================

say "Checking Python dependency consistency"

"$PY" -m pip check

say "Checking CUDA computation and imports"

"$PY" - "$LEROBOT" "$MODEL" "$TORCH_VERSION" <<'PY'
from pathlib import Path
import sys
import torch

lerobot_dir = Path(sys.argv[1]).resolve()
model_dir = Path(sys.argv[2]).resolve()
expected_torch = sys.argv[3]

if torch.__version__.split("+")[0] != expected_torch:
    raise SystemExit("PyTorch changed during dependency installation")

if not torch.cuda.is_available():
    raise SystemExit("CUDA became unavailable")

x = torch.tensor([1.0, 2.0], device="cuda")
result = (x * 2).cpu().tolist()

assert result == [2.0, 4.0]
print("CUDA tensor execution: OK")

import lerobot
import vla_protocol
import vla_server

from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from safetensors import safe_open

lerobot_path = Path(lerobot.__file__).resolve()

if not lerobot_path.is_relative_to(lerobot_dir):
    raise SystemExit(
        f"Wrong LeRobot import location: {lerobot_path}"
    )

print("LeRobot import:", lerobot_path)
print("SmolVLA class:", SmolVLAPolicy.__name__)

# Validate checkpoint structure.
weights = list(model_dir.rglob("*.safetensors"))

if len(weights) < 3:
    raise SystemExit("Expected model and processor safetensors files")

for path in weights:
    with safe_open(str(path), framework="pt", device="cpu") as f:
        names = list(f.keys())

        if not names:
            raise SystemExit(f"Empty safetensors metadata: {path}")

        print(
            "Readable safetensors:",
            path.name,
            "tensor count:",
            len(names),
        )

print("Checkpoint metadata: OK")
print("Cable_robot imports: OK")
print("Environment verification: PASS")
print("Genuine SmolVLA inference: NOT YET TESTED")
PY

# ============================================================
# 12. Save reproducibility records
# ============================================================

if (( ! CHECK_ONLY )); then

    say "Saving environment records"

    mkdir -p "$LOGDIR"

    "$PY" -m pip freeze \
        > "$LOGDIR/pip-freeze.txt"

    nvidia-smi \
        > "$LOGDIR/nvidia-smi.txt"

    git -C "$REPO" rev-parse HEAD \
        > "$LOGDIR/cable_robot.sha"

    printf '%s\n' "$ACTUAL_LEROBOT" \
        > "$LOGDIR/lerobot.sha"

    printf '%s\n' "$MODEL_SHA" \
        > "$LOGDIR/smolvla.sha"

    "$PY" - <<'PY' > "$LOGDIR/runtime.txt"
import sys
import torch

print("Python:", sys.version)
print("Torch:", torch.__version__)
print("CUDA:", torch.version.cuda)
print("GPU:", torch.cuda.get_device_name(0))
print("BF16:", torch.cuda.is_bf16_supported())
PY

fi

# ============================================================
# 13. Final result
# ============================================================

say "PASS: GPU environment and installed packages verified."

echo
echo "Python: $PY"
echo "LeRobot: $LEROBOT"
echo "SmolVLA: $MODEL"
echo "Records: $LOGDIR"

echo
echo "NEXT: Validate genuine SmolVLA GPU inference."
