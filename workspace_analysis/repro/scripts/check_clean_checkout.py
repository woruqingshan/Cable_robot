#!/usr/bin/env python3
"""Isolate current source/tests, omit results/venv, guard original artifact reads."""
import sys
sys.dont_write_bytecode=True
import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[2]


def run():
    repo=Path(subprocess.check_output(['git','rev-parse','--show-toplevel'],cwd=ROOT,text=True).strip())
    tracked=subprocess.check_output(['git','ls-files','-z'],cwd=repo).decode().split('\0')
    paths={repo/p for p in tracked if p and (repo/p).is_relative_to(ROOT)}
    # Include pending implementation/test files without staging or committing.
    # Restrict discovery to source/config/infrastructure, never arbitrary untracked data.
    for name in ['repro','tests','src','scripts','config']:
        paths.update(p for p in (ROOT/name).rglob('*') if p.is_file())
    def allowed(p):
        relative=p.relative_to(ROOT)
        return p.is_file() and not any(x in relative.parts for x in ['results','.venv','__pycache__','.pytest_cache','validation'])
    paths={p for p in paths if allowed(p)};hashes={str(p.relative_to(ROOT)):sha256(p.read_bytes()).hexdigest() for p in paths}
    with tempfile.TemporaryDirectory(prefix='jetrover-r2-clean-') as temporary:
        isolated=Path(temporary)/'cable_vla_robot/workspace_analysis';isolated.mkdir(parents=True)
        for source in paths:
            destination=isolated/source.relative_to(ROOT);destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,destination)
        policy=Path(temporary)/'policy';policy.mkdir()
        # Also guard ordinary Python subprocesses. -I subprocesses intentionally
        # ignore PYTHONPATH, but only see copied source and no results directory.
        (policy/'sitecustomize.py').write_text('''import os,sys\nfrom pathlib import Path\nroots=[Path(p).resolve() for p in os.environ.get("R2_FORBIDDEN_RESULTS","").split(os.pathsep) if p]\ndef guard(event,args):\n if event=="open" and args and isinstance(args[0],(str,bytes)):\n  p=Path(os.fsdecode(args[0])).resolve()\n  if any(p==r or r in p.parents for r in roots):raise RuntimeError("BLOCKED: original historical artifact access: "+str(p))\nsys.addaudithook(guard)\n''')
        env=dict(os.environ)
        for name in ['C1_BACKEND','G3_TEST_INPUT_PREFIX','G4_TEST_SOURCE_ROOT','JETROVER_SYSTEM_SOURCE','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','CMAKE_PREFIX_PATH','ROS_DISTRO']:
            env.pop(name,None)
        python_paths=[p for p in env.get('PYTHONPATH','').split(os.pathsep) if p and not Path(p).resolve().is_relative_to(repo)]
        env.update(PYTHONDONTWRITEBYTECODE='1',WORKSPACE_ARTIFACT_ROOT=str(isolated/'results'),R2_FORBIDDEN_RESULTS=str(ROOT/'results'),
                   PYTHONPATH=os.pathsep.join([str(policy),*python_paths]))
        command=[sys.executable,'-B','-m','pytest','-q','-rs','-p','no:cacheprovider','-m','core','tests']
        result=subprocess.run(command,cwd=isolated,env=env,capture_output=True,text=True,timeout=300)
        generated_results=(isolated/'results').exists()
        report={'status':'PASS' if result.returncode==0 and not generated_results else 'FAIL','exit_code':result.returncode or int(generated_results),
            'method':'isolated copy of tracked workspace and pending source/config/test infrastructure; original dependencies reused',
            'copied_source_sha256':hashes,'historical_results_copied':False,'original_history_read_guard':True,
            'official_ament_overlay_removed':True,'native_backend_removed':True,'results_created':generated_results,
            'python':sys.executable,'stdout':result.stdout,'stderr':result.stderr,'temporary_copy_removed':True}
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--json-output',type=Path);args=parser.parse_args()
    report=run()
    if args.json_output:
        with args.json_output.open('x') as f:json.dump(report,f,indent=2)
    print(report['stdout']);print(report['stderr'],file=sys.stderr);print('Clean checkout: '+report['status'])
    raise SystemExit(report['exit_code'])
