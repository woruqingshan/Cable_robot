"""Exact deployment pins are configuration evidence, not proof of GPU execution."""
from pathlib import Path
import re
from urllib.parse import urlparse
from pydantic import Field
from vla_protocol.profiles import StrictModel
from vla_protocol.errors import VLAError
from .artifacts import file_hash


class FrameworkPin(StrictModel):
    repository: str = "https://github.com/huggingface/lerobot"
    revision: str | None = Field(default=None, pattern=r"^[0-9a-f]{40}$")
    checkout: str = "/workspace/third_party/lerobot"


class CheckpointPin(StrictModel):
    repository: str = "lerobot/smolvla_base"
    revision: str | None = Field(default=None, pattern=r"^[0-9a-f]{40}$")
    directory: str = "/data/models/smolvla_base"
    inventory_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")


class EnvironmentPin(StrictModel):
    python_version: str | None = Field(default=None, pattern=r"^3\.12\.\d+$")
    torch_version: str | None = Field(default=None, min_length=1)
    cuda_runtime: str | None = Field(default=None, min_length=1)
    requirements_lock: str | None = None
    requirements_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    base_image_digest: str | None = Field(default=None, pattern=r"^.+@sha256:[0-9a-f]{64}$")


class DeploymentManifest(StrictModel):
    schema_version: int = Field(default=1, ge=1, le=1)
    cable_robot_revision: str | None = Field(default=None, pattern=r"^[0-9a-f]{40}$")
    lerobot: FrameworkPin = Field(default_factory=FrameworkPin)
    checkpoint: CheckpointPin = Field(default_factory=CheckpointPin)
    environment: EnvironmentPin = Field(default_factory=EnvironmentPin)
    server_config: str

    def require_resolved(self, *, download=False):
        missing = []
        for name, value in (("cable_robot_revision", self.cable_robot_revision),
            ("lerobot.revision", self.lerobot.revision), ("checkpoint.revision", self.checkpoint.revision),
            ("python_version", self.environment.python_version), ("torch_version", self.environment.torch_version),
            ("cuda_runtime", self.environment.cuda_runtime), ("requirements_lock", self.environment.requirements_lock),
            ("requirements_sha256", self.environment.requirements_sha256), ("base_image_digest", self.environment.base_image_digest)):
            if value is None:
                missing.append(name)
        if not download and self.checkpoint.inventory_sha256 is None:
            missing.append("checkpoint.inventory_sha256")
        if missing:
            raise VLAError("DEPLOYMENT_UNRESOLVED", "unverified pins remain: " + ", ".join(missing))
        if self.lerobot.repository != "https://github.com/huggingface/lerobot":
            raise VLAError("DEPLOYMENT_INVALID", "this bootstrap supports the official LeRobot repository only")

    def verify_lock(self, manifest_path):
        lock = Path(self.environment.requirements_lock)
        if not lock.is_absolute():
            lock = Path(manifest_path).resolve().parent / lock
        if not lock.is_file() or file_hash(lock) != self.environment.requirements_sha256:
            raise VLAError("ENVIRONMENT_LOCK_INVALID", "GPU dependency lock missing or hash differs")
        text = lock.read_text()
        logical = text.replace("\\\n", " ")
        lines = [line.strip() for line in logical.splitlines() if line.strip() and not line.lstrip().startswith("#")]
        requirements = []
        for line in lines:
            if line.startswith(("--index-url ", "--extra-index-url ")):
                parsed = urlparse(line.split(None, 1)[1])
                if parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
                    raise VLAError("ENVIRONMENT_LOCK_INVALID", "dependency index must be HTTPS without embedded credentials")
            elif line == "--only-binary=:all:":
                continue
            else:
                requirements.append(line)
        if not requirements or any(not re.match(r"^[A-Za-z0-9_.-]+==[^\s]+\s+--hash=sha256:[0-9a-f]{64}", line) for line in requirements):
            raise VLAError("ENVIRONMENT_LOCK_INVALID", "GPU lock must pin all packages and hashes; pip --require-hashes will enforce closure")
        return lock
