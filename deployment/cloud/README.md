# Cloud deployment preparation — Stage 1.5

The Mock/Fake HTTP deployment path is implemented and tested locally. The optional
SmolVLA binding is implemented against reviewed official APIs, but **no GPU,
LeRobot installation, real checkpoint, real model loading or genuine inference
has been verified**. Production GPU pins deliberately remain `null` in
`model_manifest.example.yaml` and `config/smolvla.example.yaml`. GPU bootstrap
fails clearly until an operator supplies an actually verified pairing.

## Layout and environment boundaries

```text
/workspace/Cable_robot/                 # clean, exact approved Git revision
/workspace/third_party/lerobot/         # independent exact upstream checkout
/workspace/venvs/                      # independent cloud environments
/data/models/smolvla_base/             # pinned checkpoint and download receipt
/data/checkpoints/
/data/runs/                            # manifests, logs, smoke evidence
```

All paths are configurable. Third-party code, model files and venvs must be
outside Cable_robot. The local ROS client remains Python 3.10. Current reviewed
LeRobot requires Python 3.12+, so model packages must never be installed into the
ROS environment. Scripts do not install drivers, CUDA toolkits or ROS packages.

## Restore and test the HTTP server on a new machine

First publish the approved branch through the normal user-authorized workflow;
Stage 1.5 does not push it. Select a real 40-character Cable_robot commit. This
placeholder is intentionally not a deployable revision:

```bash
export CABLE_REV="REPLACE_WITH_APPROVED_40_HEX_COMMIT"
git clone --no-checkout https://github.com/woruqingshan/Cable_robot /workspace/Cable_robot
cd /workspace/Cable_robot
git fetch origin "$CABLE_REV"
git checkout --detach "$CABLE_REV"

# Standard-library bootstrap; omit --execute to print a non-mutating plan.
python3 deployment/cloud/scripts/install_server.py \
  --venv /workspace/venvs/vla-http --execute
export VLA_PYTHON=/workspace/venvs/vla-http/bin/python
bash deployment/cloud/scripts/check_environment.sh --mode mock
bash deployment/cloud/scripts/start_server.sh --config deployment/cloud/config/mock.yaml
```

In another terminal:

```bash
export VLA_PYTHON=/workspace/venvs/vla-http/bin/python
"$VLA_PYTHON" deployment/cloud/scripts/smoke_test.py --url http://127.0.0.1:8000
```

This performs actual HTTP health/inference/error checks. Stop the server with
Ctrl-C. `config/fake.yaml` uses the same entry point and produces an explicitly
synthetic `[50,7]` model-native chunk for inspection. It is not a SmolVLA emulator
or evidence of real inference.

The exact HTTP/protocol dependency lock is `requirements-server.lock`. A fresh
isolated Python 3.10.12 environment and these Mock entry points were verified
locally. A fresh Python 3.12/CUDA image has not been verified. The GitHub network
clone itself was not tested for the unpublished Stage 1.5 branch; a clean local
clone is used for source-completeness acceptance.

## Resolve the GPU deployment manifest in Stage 2

Create a deployment manifest and service configuration under `/data/runs`, rather
than editing the unresolved examples into a false verified configuration. Record:

* Exact clean Cable_robot and LeRobot commit SHAs, plus local LeRobot checkout.
* Exact model repository/checkpoint revision and separate checkpoint directory.
* Exact Python patch, Torch version, CUDA runtime and pinned base image digest.
* A complete GPU dependency lock with exact versions and package hashes, and its
  SHA256. `pip --require-hashes` enforces transitive closure at installation.
* Checkpoint content inventory SHA256, feature specification, processor and
  normalization identities, native channels/units/semantics and their source.
* The service configuration and explicit timeout/warmup/transport limits.

The reviewed API reference
[`ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2`](https://github.com/huggingface/lerobot/tree/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2)
is **not** a verified source/checkpoint/environment pairing. Do not substitute a
floating `main` branch or invent a compatible model revision.

Once the pins and GPU environment have genuinely been selected:

```bash
export VLA_PYTHON=/workspace/venvs/vla-model/bin/python
# Plan first; these commands do not mutate unless --execute is supplied:
bash deployment/cloud/scripts/install_lerobot.sh --manifest /data/runs/manifest.yaml
bash deployment/cloud/scripts/download_model.sh --manifest /data/runs/manifest.yaml

# Stage 2 only: network/installation/weight download, NOT executed in Stage 1.5:
bash deployment/cloud/scripts/install_lerobot.sh --manifest /data/runs/manifest.yaml --execute
bash deployment/cloud/scripts/download_model.sh --manifest /data/runs/manifest.yaml --execute
```

Use an independent exact Python 3.12 venv without a sourced ROS environment. The
framework installer preserves an existing checkout and rejects wrong/dirty
revisions instead of resetting them. The downloader writes a receipt containing
repository, exact revision and per-file hashes. A matching existing receipt is
verified rather than overwritten. Copy its inventory SHA into both manifest and
service policy configuration before model startup. Missing local tokenizer or
backbone assets also block offline loading; their pinned artifacts/cache must be
prepared and verified during Stage 2.

Then, still in the model environment:

```bash
bash deployment/cloud/scripts/check_environment.sh --mode gpu --manifest /data/runs/manifest.yaml
bash deployment/cloud/scripts/start_server.sh --manifest /data/runs/manifest.yaml --config /data/runs/server.yaml
```

The bootstrap verifies source/lock/version/checkpoint/configuration consistency.
It does not equate those checks with model loading or successful inference.
`/health` separately reports HTTP availability, policy readiness, model loading,
warmup, GPU loading and genuine inference verification. A diagnostic direct app
startup with the unresolved SmolVLA example remains HTTP-accessible but reports
`policy_ready:false`; it never falls back to Mock.

## Official model integration and native action inspection

`vla_server.policies.smolvla.SmolVLAPolicyAdapter` owns local checkpoint loading,
official `make_pre_post_processors`, official observation preparation,
`predict_action_chunk`, output postprocessing and reset/cleanup. It checks actual
checkpoint state/image/action shapes and effective chunk size. No local language
tokenizer, normalization, image resize or guessed robot action conversion is added.
Loading is offline-only; the server never downloads missing artifacts.

Canonical camera IDs map explicitly to model image feature names. Named state
channels, units, shape and processor identity are explicit. The feature spec
must match the checkpoint. Native channel names/units/semantics still require
checkpoint/dataset evidence; matching a tensor dimension is insufficient.

Use the local ROS `model_inspection.yaml` profile to receive long native chunks.
The client records native actions before attempting adaptation. Unmapped real or
Fake actions are rejected without simulated movement and later requests continue.
The only robot mapping shipped remains the synthetic five-joint identity map.

## Secure access and independent timing

The app/config only bind to `127.0.0.1` or `::1`. Use an SSH tunnel or a separately
managed private VPN/proxy; do not expose this unauthenticated service publicly:

```bash
# Replace SSH_TARGET with your configured SSH host alias; no credentials in Git.
ssh -N -L 18000:127.0.0.1:8000 SSH_TARGET
```

Set the ROS launch `server_url:=http://127.0.0.1:18000`. DDS remains localhost-only.
Remote observation clocks are metadata, while transport deadlines are client-local
monotonic time. The cloud example uses request 30 s / inference 20 s solely as
explicit unverified settings; observation freshness stays 0.5 s and candidate
freshness stays 5 s. Slow outputs do not gain execution permission automatically.

## Docker status

`Dockerfile` is an **unbuilt, GPU-unverified template**. `BASE_IMAGE` has no floating
default; supply the pinned Python 3.12/CUDA image digest from the manifest. The
HTTP service runs in `/opt/vla`, a separate venv. For GPU bootstrap, mount the
clean pinned Git checkout read-only at `/workspace/Cable_robot` so its revision
can be verified, and mount third-party/model/run directories separately. Never
COPY weights into an image. `Dockerfile.dockerignore` limits build context to
application/deployment source and excludes weights, secrets and generated files.

```bash
# Stage 2 only; select the verified digest first. Not executed in Stage 1.5.
docker build --build-arg BASE_IMAGE="$VERIFIED_IMAGE_DIGEST" \
  -f deployment/cloud/Dockerfile -t cable-vla:approved .
# Supply GPU runtime, clean source bind mount and separate data/third-party mounts
# according to the actual rented host; validate them before claiming deployment.
```

Before calling GPU integration successful: restore the pinned environment, verify
all artifacts, load the real model/processors, run genuine single-observation and
session-reset inference, inspect real `[H,D]` output semantics, pass cloud HTTP and
local ROS inspection smoke tests, verify stale/timeout handling and clean shutdown,
and record evidence. Robot action mapping/calibration, constraints, real perception
and physical manipulation remain separate later work.
