#!/usr/bin/env python3
"""Synthetic communication acceptance using the existing one-flight PolicyClient."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import time
from uuid import uuid4

from deployment.local.test_support import observation
from cable_robot_vla.policy_client import PolicyClient
from cable_robot_vla.action_adapter import ActionAdapterRegistry
from vla_protocol import codec
from vla_protocol.errors import VLAError
from vla_protocol.observation import Observation
from vla_protocol.profiles import RuntimeLimits


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url",default="http://127.0.0.1:18000")
    parser.add_argument("--report",required=True)
    parser.add_argument("--server-sha",required=True)
    parser.add_argument("--camera-id",default="mock_rgb")
    parser.add_argument("--count",type=int,default=3)
    parser.add_argument("--interval",type=float,default=2.)
    args=parser.parse_args()
    if not 1 <= args.count <= 20 or args.interval < 0:
        parser.error("bounded count 1..20 and nonnegative interval required")
    path=Path(args.report).expanduser()
    if path.exists():
        parser.error("report already exists")
    sha=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip()
    if sha != args.server_sha:
        parser.error("JetRover and AutoDL must use identical source SHA")
    path.parent.mkdir(parents=True,exist_ok=True)
    limits=RuntimeLimits(max_horizon=64,request_timeout_s=30.)
    client=PolicyClient(args.url,limits=limits)
    session=uuid4().hex
    report={"schema_version":1,"test":"synthetic_shadow_link","source_sha":sha,
        "server_sha":args.server_sha,"input_provenance":"synthetic_fixture",
        "hardware_validated":False,"execution_enabled":False,"requests":[],"status":"NOT_RUN"}
    try:
        health=client.http.get(args.url.rstrip("/")+"/health").json()
        if not health.get("gpu_model_ready") or health.get("model_type") != "smolvla" or health.get("source_revision") != sha:
            raise RuntimeError("server GPU readiness or source SHA mismatch")
        report["health_before"]=health
        for seq in range(args.count):
            data=observation().model_dump()
            data.update(session_id=session,observation_seq=seq,request_id=uuid4().hex,
                source_profile={"profile_id":"synthetic_uncalibrated_v1"},
                capabilities=["native_actions_v1","source_profiles_v1","session_state_v1"])
            data["images"][0]["camera_id"]=args.camera_id
            obs=codec.validate(data,Observation,limits)
            started=datetime.now(timezone.utc).isoformat()
            client.submit(obs)
            deadline=time.monotonic()+35
            result=None
            while time.monotonic()<deadline and result is None:
                result=client.poll()
                if result is None:time.sleep(.01)
            if result is None:raise TimeoutError("bounded client did not complete")
            candidate=result.candidate
            if candidate.synthetic or candidate.policy.policy_type!="smolvla" or candidate.shape != [50,6]:
                raise RuntimeError("genuine SmolVLA native 50x6 response required")
            try:
                ActionAdapterRegistry().adapt(candidate)
            except VLAError as error:
                mapping_status=error.code
            else:raise RuntimeError("unknown model output unexpectedly acquired a robot mapping")
            report["requests"].append({"request_id":obs.request_id,"observation_seq":seq,
                "source_timestamps":{"joint":obs.joints.stamp.model_dump(),"camera":obs.images[0].stamp.model_dump()},
                "request_started_utc":started,"completed_utc":datetime.now(timezone.utc).isoformat(),
                "round_trip_s":result.round_trip_s,"encoding_s":result.encoding_s,
                "finite":all(math.isfinite(v) for row in candidate.values for v in row),
                "mapping_rejection":mapping_status,"candidate":candidate.model_dump(mode="json")})
            time.sleep(args.interval)
        report["health_after"]=client.http.get(args.url.rstrip("/")+"/health").json()
        if not report["health_after"].get("genuine_inference_verified"):
            raise RuntimeError("service did not attest genuine model inference")
        report["status"]="PASS"
    except Exception as error:
        report["status"]="FAIL"
        report["error"]={"code":getattr(error,"code",type(error).__name__),"message":str(error)}
        raise
    finally:
        client.close()
        with path.open("x") as stream:json.dump(report,stream,ensure_ascii=False,allow_nan=False,indent=2)
        print(json.dumps({"report":str(path),"status":report["status"],"completed_requests":len(report["requests"])}))


if __name__=="__main__":
    main()
