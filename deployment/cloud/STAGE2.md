# Stage 2: verified SmolVLA GPU inference and private HTTP

On 2026-11-14, AutoDL passed genuine official-processor inference and the existing
HTTP service's real GPU acceptance tests. These checks used synthetic observations;
they establish pipeline execution, not task performance or a calibrated robot policy.
The local Ubuntu mock ROS inspection also passed: three genuine chunks received,
three unmapped-action rejections, zero execution messages and unchanged mock state. Real sensors, calibration and physical actuation remain later work.

## Preserve the deployed environment

The offline-first restore order remains: clone the project at the approved commit;
upload the pinned LeRobot source; upload the checkpoint and its revision record;
upload the complete SmolVLM2 Hub cache; configure the separate cloud Python venv;
run environment checks; run genuine inference; start the existing HTTP service.
For the current AutoDL machine, source, weights, cache and dependencies are already
present. Reuse them; do not run installation or model-download commands again.
The original operator runbook `docs/stage2_gpu_runbook_v2.md` on AutoDL is preserved.

| Resource | Verified value |
|---|---|
| Cloud root | `/root/autodl-tmp/idc-vla` |
| Python | `venvs/vla-model/bin/python`, 3.12.3 |
| Torch / CUDA | `2.8.0+cu128` / `12.8` |
| LeRobot | 0.7.1, `ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2` |
| Transformers | 5.22.2 |
| HF Hub / Safetensors / Accelerate | 2.4.0 / 0.8.0 / 1.15.0 |
| FastAPI / Uvicorn | 0.115.12 / 0.34.2 |
| GPU | Tesla V100-PCIE-32GB, compute capability 7.0 |
| Checkpoint | `lerobot/smolvla_base`, `5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6` |
| Backbone | `HuggingFaceTB/SmolVLM2-500M-Video-Instruct`, `4cde9d208e19519c36917aa170fffc7e381aa0ad` |

The original mixed parameter precision was preserved: 474 BF16 parameter tensors
and 26 FP32 tensors. V100 has no native BF16 support. Actual forward passes
nevertheless completed on this precise stack, with finite outputs and exact
same-seed reset replay. This is execution evidence for this stack, not proof of
native BF16 acceleration or policy accuracy. No autocast, precision conversion,
architecture change or processor replacement was introduced.

## Reproducible source synchronization

Use Ubuntu's actual checkout `~/Desktop/code/cable_robot/cable_vla_robot`.
Inspect both repositories before changing branches. The initial local commit
`a4684a8` was behind AutoDL's `dc94075`; fetch the latter and create a feature
branch without staging unrelated files. Never reset either worktree or push main.

Transfer committed application source with a Git bundle, then create a detached
remote worktree under `runs/stage2/checkouts/<commit>`. Keep the original remote
`Cable_robot` worktree, its untracked runbook and accidental in-repo venv untouched.
For example, with an explicit verified local commit and a fresh bundle filename:

```bash
# Ubuntu: feature branch must already contain committed changes.
git bundle create /tmp/stage2-source.bundle HEAD ^dc94075d36dda6986c941c15c50d2a1463109150
scp /tmp/stage2-source.bundle autodl-vla:/root/autodl-tmp/idc-vla/runs/stage2/stage2-source.bundle

# AutoDL: replace COMMIT with the full transferred commit, and use a new worktree.
export BASE=/root/autodl-tmp/idc-vla
export COMMIT=REPLACE_WITH_TRANSFERRED_40_HEX_COMMIT
git -C "$BASE/Cable_robot" fetch "$BASE/runs/stage2/stage2-source.bundle" HEAD
git -C "$BASE/Cable_robot" worktree add --detach "$BASE/runs/stage2/checkouts/$COMMIT" "$COMMIT"
export SRC="$BASE/runs/stage2/checkouts/$COMMIT"
export PY="$BASE/venvs/vla-model/bin/python"
export HF_HOME="$BASE/cache/huggingface"
export HF_HUB_CACHE="$HF_HOME/hub"
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 TOKENIZERS_PARALLELISM=false
```

## Stage 2.1: genuine inference gate

Choose new report paths for each run; evidence is never overwritten.

```bash
"$PY" "$SRC/deployment/cloud/scripts/verify_smolvla_inference.py" \
  --report "$BASE/runs/stage2/inference-NEW.json"
```

This verifies the clean pinned framework, uploaded revision record, offline
backbone ref, actual checkpoint hashes, official observation preparation,
preprocessing, `predict_action_chunk`, postprocessing, finite `[1,50,6]` output,
repeated calls, reset replay and a single-camera subset. It records synchronized
CUDA timings, precision, installed versions and allocator memory. A failed
forward pass exits nonzero and cannot be replaced with Fake/Mock output.

The default is original checkpoint precision. An explicit `--dtype float32`
requires `--precision-reason`; it never silently retries or changes precision.
FP32 was unnecessary and was not tested on this deployment. `prepare_stage2.py`
currently requires passing original-precision evidence for the production adapter.

## Stage 2.2: existing HTTP service

Generate external receipts, config and a deployment manifest from passing evidence:

```bash
export DEPLOY="$BASE/runs/stage2/deployment-NEW"
"$PY" "$SRC/deployment/cloud/scripts/prepare_stage2.py" \
  --inference-report "$BASE/runs/stage2/inference-NEW.json" \
  --output-dir "$DEPLOY"

"$PY" "$SRC/deployment/cloud/scripts/verify_stage2_http.py" \
  --config "$DEPLOY/server.yaml" --manifest "$DEPLOY/manifest.yaml" \
  --report "$BASE/runs/stage2/http-NEW.json" --gpu-fault-probes
```

The runner owns the existing `start_server.sh` entry point, binds only
`127.0.0.1:8000`, refuses an occupied port and shuts down its own server cleanly.
It performs real HTTP health, three GPU calls, idempotent cache replay, session
and task resets, optional cameras, malformed/missing inputs and concurrent calls.
Every report entry identifies whether it ran a genuine forward, reused cached
output or performed validation only. Additional isolated ASGI probes use the
real GPU model with an explicitly injected postprocessor exception (500), recovery,
a real 50 ms timeout (504), and a deliberately missing model path (503). These
probes add no fault-injection endpoint to the production application.

The GPU runner uses the standard library for HTTP. It needs no cloud `httpx`
installation. Mock/Fake tests remain explicitly synthetic. `/health` distinguishes
loaded/readiness from genuine-forward verification. `Server-Timing` separates
server request handling from policy duration; health diagnostics separately
record preprocessing, model execution, postprocessing and GPU memory.

The generated `environment.kind: existing_venv` manifest verifies a hashed
`pip freeze --all` snapshot against the installed venv and records actual runtime
versions. It is audit evidence, not a hash-locked installation recipe. Install and
download operations reject this manifest mode. Container deployments retain the
original image-digest and hashed-requirements requirements. Checkpoint receipts
are kept outside model directories. Uploaded model revision provenance comes
from the operator's revision record plus file inventory; it is not an independent
Hub attestation.

For an interactive service after smoke acceptance:

```bash
export VLA_PYTHON="$PY"
bash "$SRC/deployment/cloud/scripts/start_server.sh" \
  --manifest "$DEPLOY/manifest.yaml" --config "$DEPLOY/server.yaml"
# Ctrl-C performs serialized model/processor cleanup.
```

## Feature and native-output contract

| Canonical input | Model feature | Requirement |
|---|---|---|
| Named joint1..joint5 and r_joint | `observation.state`, `[6]` | Synthetic inspection binding only; no padding/scaling |
| `mock_rgb` | `observation.images.camera1`, `[3,256,256]` | Required in generated inspection config |
| `camera2` | `observation.images.camera2`, `[3,256,256]` | Optional |
| `camera3` | `observation.images.camera3`, `[3,256,256]` | Optional |
| Nonblank instruction | Official task/newline/tokenizer pipeline | Required |

The image descriptor is `[3,256,256]`; official checkpoint processing resizes with
padding to 512×512. Different bounded source sizes are accepted only under
`shape_policy: official_processor`. Missing optional slots stay absent: the
upstream `prepare_images` implementation owns their handling. Here
`empty_cameras=0` means absent slots add no fabricated views. Unknown camera IDs,
missing required cameras and invalid state/task/schema return structured errors.
The checkpoint normalizer lists legacy image/image2/image3 names while its input
features use camera1/2/3; visual normalization is IDENTITY and was verified through
the unmodified official processors. No local renaming or normalization was added.

Responses preserve all 50 steps and six continuous model-native channels. Their
channel identities, units, absolute/delta meaning, robot mapping and calibrated
timing are unknown. The protocol now accepts `semantics: unknown`; this requires
an updated client, and older strict clients reject it. Native channels are named
`native_0`..`native_5`, units `model_native_unknown`, with an unmapped action-space
ID. The 20 ms offsets are an explicit synthetic inspection schedule, not measured
checkpoint or robot timing. Only synthetic source observations are enabled by the
generated deployment config. No native-to-JetRover conversion is registered.

## Ubuntu tunnel and ROS inspection

Keep the cloud Python 3.12 environment separate from Ubuntu ROS Python 3.10.
Open a local-only SSH forward in another terminal:

```bash
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18000:127.0.0.1:8000 autodl-vla
curl --fail http://127.0.0.1:18000/health
```

To repeat the HTTP client acceptance from Ubuntu, retrieve the non-secret service
config, then use the existing local test venv (set `LOCAL_PY` to its absolute path):

```bash
export REPO="$HOME/Desktop/code/cable_robot/cable_vla_robot"
export LOCAL_PY=REPLACE_WITH_EXISTING_LOCAL_PYTHON310_VENV/bin/python
export LOCAL_RUN="$HOME/Documents/Codex/stage2-NEW"
mkdir -p "$LOCAL_RUN"
scp "autodl-vla:$DEPLOY/server.yaml" "$LOCAL_RUN/server.yaml"
"$LOCAL_PY" "$REPO/deployment/cloud/scripts/verify_stage2_http.py" \
  --connect-only --check-local-client --url http://127.0.0.1:18000 \
  --config "$LOCAL_RUN/server.yaml" --report "$LOCAL_RUN/tunnel-http.json"

source /opt/ros/humble/setup.bash
"$LOCAL_PY" "$REPO/deployment/local/smolvla_cloud_inspection.py" \
  --url http://127.0.0.1:18000 --output-dir "$LOCAL_RUN/ros-inspection"
```

Set `DEPLOY` on Ubuntu to the selected absolute **remote** deployment directory
before the scp command. The ROS script uses a private localhost DDS domain, starts
only mock nodes, receives genuine `[50,6]` chunks, records them, checks
`UNSUPPORTED_ACTION_PROFILE`, and asserts zero execution messages and unchanged
mock state. It never launches vendor drivers. P2 validation and calibration
boundaries are unchanged; real native outputs never reach an executor.

Stage 2.3 still needs real camera/state collection with explicit source profiles,
known state representation, task/session lifecycle and latency/freshness tests.
The current ROS node intentionally labels its inputs synthetic and uses a single
mock camera ID. A separate read-only real-observation collector is the next change;
do not relabel synthetic values as real. Hardware mapping, training/fine-tuning,
calibration and P2-validated actuation remain gated on separate evidence.

## Measured evidence

The first standalone gate recorded a 10.527 s cold model load, 0.927 s first
model execution, 0.436/0.438 s subsequent executions and 1.010 GiB peak allocated
GPU memory. The initial passing HTTP source commit was
`2a939605248cea29ff98278301d76351fffa8726`: loopback round trips were
0.970/0.456/0.462 s and peak allocated memory was 1.008 GiB. Allocator peaks exclude
CUDA context/driver overhead; reports also preserve reserved memory and nvidia-smi.
Final acceptance reports under `runs/stage2` identify their exact tested source
commit and separate test types. Generated reports, snapshots, bundles, caches,
weights and venvs are not committed.
