"""Non-destructive bootstrap. GPU installers require resolved pins and --execute."""
import argparse
from importlib.metadata import version, PackageNotFoundError
import json
import os
from pathlib import Path
import subprocess
import sys
from vla_protocol.errors import VLAError
from vla_server.config import load_config
from vla_server.deployment import DeploymentManifest
from vla_server.artifacts import inventory, inventory_hash, RECEIPT

ROOT = Path(__file__).resolve().parents[3]


def load_manifest(path):
    # Same duplicate/alias-safe YAML reader as service configuration.
    from vla_server.config import read_yaml
    try:
        return DeploymentManifest.model_validate(read_yaml(path))
    except VLAError:
        raise
    except Exception as exc:
        raise VLAError("DEPLOYMENT_INVALID", str(exc)) from exc


def outside_repo(value):
    path = Path(value).expanduser().resolve()
    if path == Path("/") or path == Path.home() or path.is_relative_to(ROOT) or ROOT.is_relative_to(path):
        raise VLAError("DEPLOYMENT_PATH_INVALID", "third-party/model/environment output must be separate from Cable_robot")
    return path


def check_python(manifest):
    actual = ".".join(str(x) for x in sys.version_info[:3])
    if actual != manifest.environment.python_version or sys.prefix == sys.base_prefix or os.environ.get("ROS_DISTRO"):
        raise VLAError("CLOUD_ENVIRONMENT_REQUIRED", "activate the exact independent Python 3.12 cloud venv without a sourced ROS environment")


def check_source(manifest):
    env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    actual = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, env=env, text=True).strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT, env=env, text=True).strip()
    if actual != manifest.cable_robot_revision or dirty:
        raise VLAError("SOURCE_REVISION_MISMATCH", "Cable_robot must match the exact clean deployment revision")


def command(argv, execute):
    if execute:
        subprocess.run([str(x) for x in argv], check=True)
    return [str(x) for x in argv]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=["check", "install-lerobot", "download-model", "start"])
    parser.add_argument("--mode", choices=["mock", "gpu"], default="mock")
    parser.add_argument("--manifest")
    parser.add_argument("--config")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--port", type=int)
    args = parser.parse_args()
    try:
        result = {"operation": args.operation, "executed": False, "gpu_inference_verified": False}
        if args.operation == "check" and args.mode == "mock":
            result.update(status="MOCK_ENVIRONMENT_READY", python=sys.version.split()[0],
                packages={name: version(name) for name in ("pydantic", "fastapi", "uvicorn", "httpx", "PyYAML", "Pillow")},
                gpu_environment_verified=False, configuration_reproducible=False)
        elif args.operation == "start" and args.config and load_config(args.config).policy.type != "smolvla":
            os.execv(sys.executable, [sys.executable, "-B", "-m", "vla_server.app", "--config", str(Path(args.config).resolve()),
                *(["--port", str(args.port)] if args.port else [])])
        else:
            if not args.manifest:
                raise VLAError("MANIFEST_REQUIRED", "GPU operations require an exact deployment manifest")
            manifest = load_manifest(args.manifest)
            manifest.require_resolved(download=args.operation in ("download-model", "install-lerobot"))
            lock = manifest.verify_lock(args.manifest)
            check_source(manifest)
            checkout, checkpoint = outside_repo(manifest.lerobot.checkout), outside_repo(manifest.checkpoint.directory)
            if args.operation == "install-lerobot":
                commands = []
                if checkout.exists():
                    actual = subprocess.check_output(["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
                    dirty = subprocess.check_output(["git", "-C", str(checkout), "status", "--porcelain", "--untracked-files=no"], text=True).strip()
                    origin = subprocess.check_output(["git", "-C", str(checkout), "remote", "get-url", "origin"], text=True).strip().removesuffix(".git")
                    if actual != manifest.lerobot.revision or dirty or origin != manifest.lerobot.repository:
                        raise VLAError("LEROBOT_REVISION_MISMATCH", "existing checkout is preserved; supply a matching clean revision")
                else:
                    commands += [["git", "clone", "--no-checkout", manifest.lerobot.repository, str(checkout)],
                        ["git", "-C", str(checkout), "fetch", "--depth=1", "origin", manifest.lerobot.revision],
                        ["git", "-C", str(checkout), "checkout", "--detach", manifest.lerobot.revision]]
                commands += [[sys.executable, "-m", "pip", "install", "--require-hashes", "-r", str(lock)],
                             [sys.executable, "-m", "pip", "install", "--no-deps", "--no-build-isolation", "-e", str(checkout)]]
                if args.execute: check_python(manifest)
                result["commands"] = [command(c, args.execute) for c in commands]
                result["executed"] = args.execute
            elif args.operation == "download-model":
                result.update(repository=manifest.checkpoint.repository, revision=manifest.checkpoint.revision, directory=str(checkpoint))
                if args.execute:
                    check_python(manifest)
                    if checkpoint.exists():
                        from vla_server.artifacts import verify_checkpoint
                        if not (checkpoint / RECEIPT).is_file():
                            raise VLAError("CHECKPOINT_DIRECTORY_EXISTS", "existing directory has no receipt and will not be overwritten")
                        receipt = json.loads((checkpoint / RECEIPT).read_text())
                        expected = manifest.checkpoint.inventory_sha256 or inventory_hash(receipt["files"])
                        files = verify_checkpoint(checkpoint, manifest.checkpoint.repository, manifest.checkpoint.revision, expected)
                        result["already_present"] = True
                    else:
                        from huggingface_hub import snapshot_download
                        snapshot_download(repo_id=manifest.checkpoint.repository, revision=manifest.checkpoint.revision, local_dir=checkpoint)
                        files = inventory(checkpoint)
                        receipt = {"repository": manifest.checkpoint.repository, "revision": manifest.checkpoint.revision, "files": files}
                        (checkpoint / RECEIPT).write_text(json.dumps(receipt, indent=2) + "\n")
                    result.update(executed=True, inventory_sha256=inventory_hash(files))
            elif args.operation in ("check", "start"):
                check_python(manifest)
                import torch
                if (torch.__version__ != manifest.environment.torch_version or torch.version.cuda != manifest.environment.cuda_runtime or not torch.cuda.is_available()):
                    raise VLAError("GPU_ENVIRONMENT_MISMATCH", "Torch/CUDA availability or versions differ from manifest")
                from vla_server.artifacts import verify_checkpoint
                verify_checkpoint(checkpoint, manifest.checkpoint.repository, manifest.checkpoint.revision, manifest.checkpoint.inventory_sha256)
                config_path = Path(args.config) if args.config else Path(args.manifest).resolve().parent / manifest.server_config
                config = load_config(config_path)
                config.policy.require_model_pins()
                if (config.source_revision != manifest.cable_robot_revision or config.policy.lerobot_revision != manifest.lerobot.revision
                    or config.policy.checkpoint_revision != manifest.checkpoint.revision or config.policy.checkpoint_inventory_sha256 != manifest.checkpoint.inventory_sha256
                    or Path(config.policy.lerobot_checkout).resolve() != checkout or Path(config.policy.checkpoint_path).resolve() != checkpoint):
                    raise VLAError("DEPLOYMENT_CONFIG_MISMATCH", "server configuration differs from the pinned manifest")
                if args.operation == "start":
                    os.execv(sys.executable, [sys.executable, "-B", "-m", "vla_server.app", "--config", str(config_path.resolve()),
                        *(["--port", str(args.port)] if args.port else [])])
                result.update(status="GPU_ENVIRONMENT_CONFIGURED", model_loading_verified=False)
        print(json.dumps(result, indent=2))
    except (VLAError, PackageNotFoundError, ImportError, OSError, ValueError, subprocess.CalledProcessError) as exc:
        error = exc if isinstance(exc, VLAError) else VLAError("DEPLOYMENT_CHECK_FAILED", str(exc))
        print(json.dumps(error.report(), indent=2), file=sys.stderr)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
