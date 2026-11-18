#!/usr/bin/env python3
"""Offline genuine GPU forward-pass gate; synthetic inputs never authorize motion."""
import argparse
from collections import Counter
from datetime import datetime, timezone
from hashlib import sha256
from importlib.metadata import version
import json
import os
from pathlib import Path
import subprocess
import sys
from time import perf_counter
import traceback

LEROBOT_REV = "ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2"
CHECKPOINT_REV = "5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6"
BACKBONE_REV = "4cde9d208e19519c36917aa170fffc7e381aa0ad"


def git(path, *args):
    return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()


def digest(path):
    h = sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def external_report(value, repo):
    path = Path(value).expanduser().resolve()
    if path.is_relative_to(repo.resolve()):
        raise ValueError("experiment reports must be outside the Git repository")
    if path.exists():
        raise ValueError("report already exists; choose a new evidence path")
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def validate_actions(actions, torch, shape=(1, 50, 6)):
    if tuple(actions.shape) != shape:
        raise ValueError(f"unexpected action shape: {tuple(actions.shape)} != {shape}")
    if not bool(torch.isfinite(actions).all().item()):
        raise ValueError("action contains NaN/Inf")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="/root/autodl-tmp/idc-vla")
    parser.add_argument("--repo", help="source repository used for provenance")
    parser.add_argument("--report", required=True)
    parser.add_argument("--dtype", choices=["checkpoint", "float32"], default="checkpoint")
    parser.add_argument("--precision-reason", help="required for explicit FP32 conversion; no automatic fallback")
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()
    if args.repeats < 2 or (args.dtype != "checkpoint" and not args.precision_reason):
        parser.error("use at least two repeats and justify any precision conversion")
    base = Path(args.base).resolve()
    repo = Path(args.repo).resolve() if args.repo else Path(__file__).resolve().parents[3]
    report_path = external_report(args.report, repo)
    report = {"schema_version": 1, "status": "FAIL", "test_kind": "real_gpu_inference",
        "synthetic_observation": True, "robot_actuation": False,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "requested_dtype": args.dtype, "precision_reason": args.precision_reason,
        "script_sha256": digest(__file__), "requests": []}
    policy = pre = post = torch = None
    try:
        checkout = base / "third_party/lerobot"
        checkpoint = base / "models/smolvla_base"
        if git(checkout, "rev-parse", "HEAD") != LEROBOT_REV or git(checkout, "status", "--porcelain", "--untracked-files=no"):
            raise ValueError("LeRobot must match the exact clean reviewed revision")
        if (base / "models/smolvla_base_revision.txt").read_text().strip() != CHECKPOINT_REV:
            raise ValueError("uploaded checkpoint revision record differs")
        os.environ.update(HF_HOME=str(base / "cache/huggingface"),
            HF_HUB_CACHE=str(base / "cache/huggingface/hub"), HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1",
            HF_HUB_DISABLE_TELEMETRY="1", TOKENIZERS_PARALLELISM="false")
        sys.path.insert(0, str(checkout / "src"))
        import numpy as np
        import torch
        import lerobot
        from huggingface_hub import snapshot_download
        from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.utils import prepare_observation_for_inference
        if not Path(lerobot.__file__).resolve().is_relative_to(checkout / "src"):
            raise ValueError("wrong imported LeRobot source")
        if not torch.cuda.is_available():
            raise ValueError("CUDA unavailable")
        backbone = Path(snapshot_download("HuggingFaceTB/SmolVLM2-500M-Video-Instruct",
            revision=BACKBONE_REV, local_files_only=True))
        # The upstream loader resolves the backbone by model ID. Check its local main ref too.
        resolved = Path(snapshot_download("HuggingFaceTB/SmolVLM2-500M-Video-Instruct", local_files_only=True))
        if backbone.resolve() != resolved.resolve():
            raise ValueError("offline backbone main ref differs from pinned snapshot")
        report.update(cable_robot_revision=git(repo, "rev-parse", "HEAD"),
            source_dirty=bool(git(repo, "status", "--porcelain", "--untracked-files=no")),
            lerobot_revision=LEROBOT_REV, checkpoint_revision=CHECKPOINT_REV,
            backbone_revision=BACKBONE_REV, backbone_snapshot=str(backbone), python=sys.version,
            packages={p: version(p) for p in ["torch", "torchvision", "lerobot", "transformers", "huggingface-hub", "safetensors", "accelerate", "fastapi", "uvicorn"]},
            cuda_runtime=torch.version.cuda, gpu=torch.cuda.get_device_name(0),
            compute_capability=list(torch.cuda.get_device_capability(0)),
            native_bf16_supported=torch.cuda.is_bf16_supported(including_emulation=False),
            checkpoint_files={p.name: digest(p) for p in sorted(checkpoint.iterdir()) if p.is_file()})
        cfg = SmolVLAConfig.from_pretrained(checkpoint, local_files_only=True)
        cfg.device = "cuda"
        report["configuration"] = cfg.to_dict() if hasattr(cfg, "to_dict") else json.loads((checkpoint / "config.json").read_text())
        if list(cfg.input_features["observation.state"].shape) != [6] or list(cfg.action_feature.shape) != [6] or cfg.chunk_size != 50 or cfg.num_steps != 10:
            raise ValueError("unexpected base checkpoint state/action/horizon/sampling configuration")
        report["camera_features"] = {name: list(feature.shape) for name, feature in cfg.image_features.items()}
        torch.manual_seed(1729)
        torch.cuda.reset_peak_memory_stats()
        begin = perf_counter()
        policy = SmolVLAPolicy.from_pretrained(checkpoint, config=cfg, local_files_only=True, strict=True).to("cuda").eval()
        torch.cuda.synchronize()
        report["cold_load_s"] = perf_counter() - begin
        report["loaded_parameter_dtypes"] = dict(Counter(str(p.dtype) for p in policy.parameters()))
        if args.dtype == "float32":
            policy.to(dtype=torch.float32)
        report["execution_parameter_dtypes"] = dict(Counter(str(p.dtype) for p in policy.parameters()))
        report["allocated_after_load_gib"] = torch.cuda.memory_allocated() / 1024**3
        pre, post = make_pre_post_processors(policy.config, pretrained_path=checkpoint,
            pretrained_revision=CHECKPOINT_REV, preprocessor_overrides={"device_processor": {"device": "cuda"}})

        def reset():
            policy.reset()
            for processor in (pre, post):
                if callable(getattr(processor, "reset", None)):
                    processor.reset()

        def infer(label, cameras, seed=None, reset_first=False):
            if reset_first:
                reset()
            if seed is not None:
                torch.manual_seed(seed)
            frame = {"observation.state": np.zeros(6, dtype=np.float32)}
            for name in cameras:
                c, h, w = cfg.image_features[name].shape
                frame[name] = np.full((h, w, c), 64, dtype=np.uint8)
            torch.cuda.synchronize()
            begin = perf_counter()
            prepared = prepare_observation_for_inference(frame, torch.device("cuda"), task="Inspect the synthetic scene; do not execute robot actions", robot_type="synthetic_uncalibrated")
            batch = pre(prepared)
            torch.cuda.synchronize()
            pre_s = perf_counter() - begin
            begin = perf_counter()
            with torch.inference_mode():
                actions = policy.predict_action_chunk(batch)
            torch.cuda.synchronize()
            model_s = perf_counter() - begin
            validate_actions(actions, torch)
            begin = perf_counter()
            with torch.inference_mode():
                actions = post(actions)
            torch.cuda.synchronize()
            post_s = perf_counter() - begin
            validate_actions(actions, torch)
            values = actions.detach().float().cpu()
            report["requests"].append({"label": label, "camera_features": list(cameras),
                "reset_first": reset_first, "seed": seed, "shape": list(values.shape), "finite": True,
                "preprocess_s": pre_s, "model_s": model_s, "postprocess_s": post_s,
                "total_s": pre_s + model_s + post_s, "output_dtype": str(actions.dtype),
                "min": values.min().item(), "max": values.max().item(),
                "values": values[0].tolist(), "peak_allocated_gib": torch.cuda.max_memory_allocated() / 1024**3})
            print(label, "PASS", list(values.shape), f"model={model_s:.3f}s", flush=True)
            return values

        cameras = list(cfg.image_features)
        first = infer("first_full_cameras", cameras, seed=1729, reset_first=True)
        for index in range(args.repeats - 1):
            infer(f"repeat_{index + 1}", cameras)
        repeated = infer("reset_same_seed", cameras, seed=1729, reset_first=True)
        report["reset_repeatability"] = {"atol": 1e-5, "max_abs_difference": (first - repeated).abs().max().item(),
            "pass": bool(torch.allclose(first, repeated, atol=1e-5, rtol=1e-5))}
        if not report["reset_repeatability"]["pass"]:
            raise ValueError("same-seed reset replay differs")
        infer("single_camera_official_subset", cameras[:1], reset_first=True)
        report["missing_camera_semantics"] = {"empty_cameras": cfg.empty_cameras,
            "behavior": "absent camera slots left absent; official prepare_images owns omission/masking"}
        report["peak_allocated_gib"] = torch.cuda.max_memory_allocated() / 1024**3
        report["peak_reserved_gib"] = torch.cuda.max_memory_reserved() / 1024**3
        report["nvidia_smi"] = subprocess.check_output(["nvidia-smi", "--query-gpu=name,driver_version,memory.used,memory.total", "--format=csv"], text=True).strip()
        report["status"] = "PASS"
        report["genuine_forward_pass_verified"] = True
    except Exception as exc:
        report["error"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
        if torch is not None and torch.cuda.is_available():
            report["peak_allocated_gib"] = torch.cuda.max_memory_allocated() / 1024**3
        print(report["error"]["traceback"], file=sys.stderr)
    finally:
        if policy is not None:
            policy.reset()
        policy = pre = post = None
        if torch is not None and torch.cuda.is_available():
            torch.cuda.empty_cache()
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(json.dumps({"status": report["status"], "report": str(report_path)}), flush=True)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
