import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import httpx
import pytest
from vla_protocol.errors import VLAError
from vla_server.config import load_config
from vla_server.deployment import DeploymentManifest

ROOT = Path(__file__).resolve().parents[3]
CLOUD = ROOT / "deployment/cloud"


def test_configuration_pins_remain_honestly_unresolved():
    import yaml
    manifest = DeploymentManifest.model_validate(yaml.safe_load((CLOUD / "model_manifest.example.yaml").read_text()))
    with pytest.raises(VLAError) as err: manifest.require_resolved()
    assert err.value.code == "DEPLOYMENT_UNRESOLVED"
    model = load_config(CLOUD / "config/smolvla.example.yaml")
    with pytest.raises(VLAError): model.policy.require_model_pins()
    for field in ("cable_robot_revision",):
        data = manifest.model_dump()
        data[field] = "main"
        with pytest.raises(ValueError): DeploymentManifest.model_validate(data)


def test_gpu_lock_requires_hashes_and_accepts_explicit_secure_pytorch_index(tmp_path):
    from hashlib import sha256
    lock = tmp_path / "gpu.lock"
    def validate(text):
        lock.write_text(text)
        manifest = DeploymentManifest(server_config="server.yaml", environment={"requirements_lock": str(lock),
            "requirements_sha256": sha256(lock.read_bytes()).hexdigest()})
        return manifest.verify_lock(tmp_path / "manifest.yaml")
    # Syntax-only dummy hash: no package, CUDA dependency or model is installed.
    pinned = "torch==2.7.1 --hash=sha256:" + "a" * 64 + "\n"
    assert validate("--extra-index-url https://download.pytorch.org/whl/cu126\n" + pinned) == lock
    for text in ("torch>=2.7\n", "--index-url http://index.example/simple\n" + pinned,
                 "--index-url https://user:secret@index.example/simple\n" + pinned,
                 "--trusted-host index.example\n" + pinned):
        with pytest.raises(VLAError): validate(text)


@pytest.mark.parametrize("script", ["install_lerobot.sh", "download_model.sh", "check_environment.sh"])
def test_gpu_scripts_fail_before_network_or_install_when_pins_unresolved(script):
    env = dict(os.environ, VLA_PYTHON=sys.executable, PYTHONDONTWRITEBYTECODE="1")
    args = ["bash", str(CLOUD / "scripts" / script), "--manifest", str(CLOUD / "model_manifest.example.yaml")]
    if script == "check_environment.sh": args += ["--mode", "gpu"]
    result = subprocess.run(args, env=env, capture_output=True, text=True, timeout=10)
    assert result.returncode == 2 and json.loads(result.stderr)["code"] == "DEPLOYMENT_UNRESOLVED"


def test_shell_syntax_and_server_install_plan_are_safe(tmp_path):
    for path in (CLOUD / "scripts").glob("*.sh"):
        subprocess.run(["bash", "-n", str(path)], check=True)
    env = dict(os.environ, VLA_PYTHON=sys.executable)
    target = tmp_path / "venv"
    result = subprocess.run(["bash", str(CLOUD / "scripts/install_server.sh"), "--venv", str(target)],
        env=env, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report["executed"] is False and len(report["commands"]) == 3
    assert not target.exists()
    assert not any("torch" in str(command) or "lerobot" in str(command) for command in report["commands"])


@pytest.mark.parametrize("policy", ["mock", "fake"])
def test_cloud_entrypoint_actual_health_inference_errors_and_shutdown(tmp_path, policy):
    env = dict(os.environ, VLA_PYTHON=sys.executable, PYTHONDONTWRITEBYTECODE="1")
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    log = (tmp_path / "cloud.log").open("w")
    process = subprocess.Popen(["bash", str(CLOUD / "scripts/start_server.sh"), "--config", str(CLOUD / f"config/{policy}.yaml"), "--port", str(port)],
        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        end = time.monotonic() + 10
        while time.monotonic() < end:
            try:
                if httpx.get(f"http://127.0.0.1:{port}/health", timeout=0.2, trust_env=False).status_code == 200: break
            except httpx.HTTPError: time.sleep(0.05)
        else: pytest.fail((tmp_path / "cloud.log").read_text())
        result = subprocess.run([sys.executable, "-B", str(CLOUD / "scripts/smoke_test.py"), "--url", f"http://127.0.0.1:{port}"],
            env=env, capture_output=True, text=True, timeout=10)
        assert result.returncode == 0, result.stderr + (tmp_path / "cloud.log").read_text()
        report = json.loads(result.stdout)
        assert report["status"] == "PASS" and report["gpu_inference_verified"] is False
        assert report["shape"] == ([1, 5] if policy == "mock" else [50, 7])
    finally:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
        log.close()
    assert process.returncode == 0
