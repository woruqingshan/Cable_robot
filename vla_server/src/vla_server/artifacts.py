"""Pinned checkpoint inventory, shared by deployment and optional model loading."""
from hashlib import sha256
import json
from pathlib import Path
from vla_protocol.errors import VLAError

RECEIPT = "checkpoint_receipt.json"


def file_hash(path):
    digest = sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def inventory(root):
    root = Path(root).resolve()
    files = {}
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root)
        if not path.is_file() or ".cache" in relative.parts or relative.as_posix() == RECEIPT:
            continue
        if not path.resolve().is_relative_to(root):
            raise VLAError("CHECKPOINT_INVALID", "checkpoint symlink escapes its directory")
        files[relative.as_posix()] = file_hash(path)
    if len(files) > 100000:
        raise VLAError("CHECKPOINT_INVALID", "checkpoint contains too many files")
    return files


def inventory_hash(files):
    return sha256(json.dumps(files, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def verify_checkpoint(root, repository, revision, expected_inventory):
    root = Path(root)
    if not (root / "config.json").is_file() or not (root / RECEIPT).is_file():
        raise VLAError("CHECKPOINT_MISSING", "local config.json and pinned download receipt are required")
    try:
        receipt = json.loads((root / RECEIPT).read_text())
        if (receipt["repository"], receipt["revision"]) != (repository, revision):
            raise ValueError("checkpoint repository/revision differs")
        files = inventory(root)
        if files != receipt["files"] or inventory_hash(files) != expected_inventory:
            raise ValueError("checkpoint content inventory differs from pin")
        if not any(name.endswith(".safetensors") for name in files):
            raise ValueError("no local safetensors checkpoint")
    except (ValueError, KeyError, OSError) as exc:
        raise VLAError("CHECKPOINT_INVALID", str(exc)) from exc
    return files
