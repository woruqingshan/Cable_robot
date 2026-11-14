[[ -f "$BASE/models/smolvla_base_revision.txt" ]] || fail 'Missing models/smolvla_base_revision.txt (model SHA recorded on Mac)'
MODEL_SHA="$(tr -d '\r\n' < "$BASE/models/smolvla_base_revision.txt")"
[[ "$MODEL_SHA" =~ ^[a-f0-9]{40}$ ]] || fail "Model revision is not a 40-character SHA: $MODEL_SHA"
[[ "$EXPECTED_LEROBOT" =~ ^[a-f0-9]{40}$ ]] || fail 'VLA_LEROBOT_REF must be an exact 40-character SHA'
ACTUAL_LEROBOT="$(git -C "$LEROBOT" rev-parse HEAD)"
[[ "$ACTUAL_LEROBOT" == "$EXPECTED_LEROBOT" ]] || fail "LeRobot HEAD $ACTUAL_LEROBOT != expected $EXPECTED_LEROBOT. Fix/approve the revision; script will not reset Git."
[[ -z "$(git -C "$LEROBOT" status --porcelain --untracked-files=no)" ]] || fail 'LeRobot tracked worktree is dirty or incomplete (git status --short). Restore intentionally before installation.'

say 'GPU and storage'
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv
df -h "$BASE" || true

if [[ -x "$PYENV/bin/python" ]]; then
  PY="$PYENV/bin/python"
else
  if (( CHECK_ONLY )); then fail "Environment not created: $PYENV"; fi
  command -v python3.12 >/dev/null || fail 'python3.12 not available; install/select Python 3.12 before running'
  [[ ! -e "$PYENV" ]] || fail "Environment directory exists but has no bin/python: $PYENV"
  say 'Creating isolated Python 3.12 environment'
  mkdir -p "$(dirname "$PYENV")"
  python3.12 -m venv "$PYENV"
  PY="$PYENV/bin/python"
fi
"$PY" -c 'import sys; assert sys.version_info[:2] == (3,12), sys.version' || fail 'GPU model environment must be Python 3.12'

if (( ! CHECK_ONLY )); then
  mkdir -p "$LOGDIR" "$BASE/cache/pip" "$BASE/cache/huggingface"
  export PIP_INDEX_URL="$PYPI_INDEX" PIP_DEFAULT_TIMEOUT=120 PIP_RETRIES=8
  export PIP_CACHE_DIR="$BASE/cache/pip" HF_HOME="$BASE/cache/huggingface" HF_HUB_CACHE="$BASE/cache/huggingface/hub"
  export PYTHONNOUSERSITE=1
  say "Using ordinary Python packages from: $PIP_INDEX_URL"
  "$PY" -m pip install -U pip setuptools wheel
  if "$PY" -c 'import torch' >/dev/null 2>&1; then
    say 'Reusing installed PyTorch'
  else
    (( INSTALL_TORCH )) || fail 'PyTorch missing and --skip-torch-install requested'
    say 'Installing CUDA-enabled Torch from dedicated official wheel index'
    "$PY" -m pip install "torch==$TORCH_VERSION" "torchvision==$TV_VERSION" --index-url "$TORCH_INDEX"
  fi
fi

say 'Checking Torch/CUDA and model GPU capabilities'
"$PY" - "$TORCH_VERSION" <<'PY'
import sys, torch
want=sys.argv[1]
print('Python:', sys.version.split()[0], 'Torch:', torch.__version__, 'CUDA:', torch.version.cuda)
if torch.__version__.split('+')[0] != want:
    raise SystemExit('Torch version mismatch; stop instead of silently reinstalling')
if not torch.cuda.is_available():
    raise SystemExit('CUDA unavailable: inspect image/driver/venv before continuing')
print('GPU:', torch.cuda.get_device_name(0))
print('Compute capability:', torch.cuda.get_device_capability(0))
print('BF16 supported:', torch.cuda.is_bf16_supported())
if not torch.cuda.is_bf16_supported():
    print('NOTICE: Do NOT force BF16 on this GPU; verify FP16/FP32 support for actual model inference')
PY

if (( ! CHECK_ONLY )); then
  say 'Installing LeRobot from the UPLOADED local Git checkout (no git download)'
  "$PY" -m pip install -e "$LEROBOT[smolvla]"
  say 'Installing local Cable_robot protocol and HTTP server'
  "$PY" -m pip install -e "$REPO/vla_protocol" -e "$REPO/vla_server"
fi

say 'Checking Python dependencies and imports'
"$PY" -m pip check
"$PY" - "$LEROBOT" "$MODEL" "$TORCH_VERSION" <<'PY'
import pathlib, sys, torch
if torch.__version__.split('+')[0] != sys.argv[3] or not torch.cuda.is_available():
    raise SystemExit('Torch/CUDA changed after installing LeRobot; resolve dependencies before proceeding')
try:
    x = torch.tensor([1., 2.], device='cuda')
    result = (x * 2).cpu().tolist()
    if result != [2., 4.]: raise RuntimeError('Wrong CUDA result')
    print('CUDA tensor execution OK')
except Exception as exc:
    raise SystemExit(f'Actual CUDA tensor execution failed: {exc}')
from safetensors import safe_open
import lerobot, vla_protocol, vla_server
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
source = pathlib.Path(sys.argv[1]).resolve()
actual = pathlib.Path(lerobot.__file__).resolve()
if not actual.is_relative_to(source):
    raise SystemExit(f'Wrong LeRobot import location: {actual} (expected under {source})')
model=pathlib.Path(sys.argv[2])
weights=list(model.rglob('*.safetensors'))
if len(weights) < 3:
    raise SystemExit('Expected model and pre/post processor safetensors files')
for p in weights:
    with safe_open(str(p), framework='pt', device='cpu') as f:
        n=len(list(f.keys()))
        print('Readable safetensors:', p.name, 'tensors:', n)
        if not n: raise SystemExit('Empty safetensors metadata: '+str(p))
print('LeRobot:', actual, 'SmolVLA class:', SmolVLAPolicy.__name__)
print('Environment and local files CHECKED; genuine model loading/inference NOT yet tested.')
PY

if (( ! CHECK_ONLY )); then
  say 'Recording local installation evidence'
  "$PY" -m pip freeze > "$LOGDIR/pip-freeze.txt"
  nvidia-smi > "$LOGDIR/nvidia-smi.txt"
  git -C "$REPO" rev-parse HEAD > "$LOGDIR/cable_robot.sha"
  printf '%s\n' "$ACTUAL_LEROBOT" > "$LOGDIR/lerobot.sha"
  printf '%s\n' "$MODEL_SHA" > "$LOGDIR/smolvla.sha"
  "$PY" - <<'PY' > "$LOGDIR/runtime.txt"
import sys, torch
print('Python',sys.version)
print('Torch',torch.__version__)
print('CUDA',torch.version.cuda)
print('GPU',torch.cuda.get_device_name(0))
print('BF16',torch.cuda.is_bf16_supported())
PY
fi
say 'PASS: installed imports, CUDA and checkpoint metadata checks; real GPU inference is the NEXT stage.'
echo "Python: $PY"
echo "Records: $LOGDIR"
