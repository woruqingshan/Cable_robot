"""Strict read-only adaptation of saved T5.1 results. No planner/backend is run."""
import argparse
from io import BytesIO
import json
from pathlib import Path
import sys
import zipfile
import numpy as np
from .data import Catalog, Goal, Port, Trajectory
from .errors import ReplayBlocked, ReplayError
from .research import use_workspace
from .storage import ReadOnlyStore, json_object, relative_name, sha


def bind_inputs(workspace: ReadOnlyStore, profile: dict, manifest: dict) -> dict[str, Path]:
    """Match explicit current files by ALL recorded input digests, never by old paths.

    The backend identity file is only hashed as provenance bytes; not executed or
    dynamically loaded. Missing provenance remains BLOCKED rather than skipped.
    """
    if set(profile) != {'schema_version','workspace_relative_files'} or profile['schema_version'] != 1:
        raise ReplayError('INPUT_PROFILE', 'unsupported input binding profile')
    names = profile['workspace_relative_files']
    if not isinstance(names, list) or not names or len(names) != len(set(names)):
        raise ReplayError('INPUT_PROFILE', 'unique declared input files required')
    by_hash = {}
    for name in names:
        raw = workspace.read(name); h = sha(raw)
        if h in by_hash: raise ReplayError('INPUT_PROFILE', 'ambiguous identical input files')
        by_hash[h] = workspace.path(name)
    bindings = {}
    for old_name, expected in manifest['input_hashes'].items():
        if expected not in by_hash:
            raise ReplayBlocked('MISSING_PROVENANCE_INPUT', 'No declared file matches '+old_name+' SHA256 '+str(expected))
        bindings[old_name] = by_hash[expected]
    return bindings


def _trajectory(store: ReadOnlyStore, files: dict, name: str, phase: str) -> Trajectory:
    if name not in files: raise ReplayError('UNLISTED_ARTIFACT', name)
    raw = store.read(name, digest=files[name])
    try:
        with zipfile.ZipFile(BytesIO(raw)) as archive:
            entries = archive.infolist()
            if len({e.filename for e in entries}) != len(entries) or sum(e.file_size for e in entries) > 64*1024*1024:
                raise ReplayError('INVALID_NPZ', 'duplicate members or oversized expansion')
        with np.load(BytesIO(raw), allow_pickle=False) as data:
            expected = {'q','T_base_tcp','valid','query_id','self_clearance_m','environment_clearance_m'}
            if set(data.files) != expected: raise ReplayError('INVALID_NPZ', 'unexpected arrays')
            return Trajectory(data['q'], data['T_base_tcp'], data['valid'], data['query_id'],
                data['self_clearance_m'], data['environment_clearance_m'],
                tuple((name,i) for i in range(len(data['q']))), (phase,)*len(data['q']))
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        if isinstance(exc, ReplayError): raise
        raise ReplayError('INVALID_NPZ', name+': '+str(exc)) from exc


def read_catalog(workspace_root: str | Path, benchmark_dir: str | Path,
                 input_profile: str | Path, *, expected_manifest_sha256: str | None = None) -> Catalog:
    """Full existing benchmark verification, then immutable exact-sample replay data.

    Only the declared run, explicit provenance profile and trusted model resources
    are read. Legacy absolute input names are metadata, never fallback read paths.
    """
    root = use_workspace(workspace_root); store = ReadOnlyStore(benchmark_dir); workspace = ReadOnlyStore(root)
    original_manifest = store.read('manifest.json', digest=expected_manifest_sha256)
    manifest = json_object(original_manifest)
    if manifest.get('stage') != 'T5.1' or manifest.get('schema_version') != 1 or manifest.get('status') != 'COMPLETED':
        raise ReplayBlocked('INCOMPLETE_EXPERIMENT', 'completed T5.1 run required')
    files = manifest.get('files')
    if not isinstance(files, dict) or not files: raise ReplayError('INVALID_MANIFEST', 'file hashes required')
    for name, h in files.items():
        relative_name(name); store.read(name, digest=h)
    receipt = store.object('validation_manifest.json')
    report_raw = store.read('independent_validation.json', digest=receipt.get('report_sha256'))
    if receipt.get('manifest_sha256') != sha(original_manifest) or json_object(report_raw).get('status') != 'PASS':
        raise ReplayError('VALIDATION_RECEIPT', 'saved full-verification receipt mismatch')
    def obj(name):
        if name not in files: raise ReplayError('UNLISTED_ARTIFACT', name)
        return store.object(name, digest=files[name])
    settings = obj('settings.json')
    ports = settings['ports']
    if not isinstance(ports,list) or not ports or any(not isinstance(p,str) or not p.isalnum() for p in ports) or len(set(ports)) != len(ports):
        raise ReplayError('PORTS', 'unique simple port IDs required')
    # Preflight every dynamic path before calling the established full verifier.
    for port in ports:
        record = obj('ports/'+port+'/result.json')
        for goal in record['goals']:
            references = [t['trajectory_file'] for t in goal['paths'] if t.get('validation') is not None]
            if goal['approach']['status'] != 'NOT_RUN': references.append(goal['approach']['trajectory_file'])
            for name in references:
                relative_name(name)
                if '/' in name or 'ports/'+port+'/'+name not in files:
                    raise ReplayError('UNTRUSTED_PATH', 'unlisted trajectory: '+str(name))
                store.path('ports/'+port+'/'+name)
    # The explicitly selected package config may be a colcon --symlink-install
    # link. Resolve only this trusted profile, never artifact or provenance files.
    profile_path = Path(input_profile).expanduser().resolve(strict=True)
    profile_store = ReadOnlyStore(profile_path.parent)
    bindings = bind_inputs(workspace, profile_store.object(profile_path.name), manifest)
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.scene.schema import parse_scene
    from workspace_analysis.task.spec import parse_task
    from workspace_analysis.task.targets import build_targets
    from workspace_analysis.experiments.task_benchmark_validation import verify_benchmark
    from repro.doctor import portable_robot_identity, compare_robot_identity
    model = load_model(root/'config/workspace.yaml')
    actual = portable_robot_identity(model); actual['model_config_sha256'] = sha(workspace.read('config/workspace.yaml'))
    assets = workspace.object('repro/environment/model_assets.json')
    asset_check = compare_robot_identity(actual, assets)
    if asset_check['status'] != 'PASS': raise ReplayError('MODEL_ASSETS', str(asset_check))
    full = verify_benchmark(store.root, model, input_bindings=bindings)
    scene = parse_scene(store.read('scene_snapshot.json', digest=files['scene_snapshot.json']).decode())
    start = obj('common_start.json')
    if start['q'] is None: raise ReplayBlocked('INVALID_START', 'no verified common simulation start')
    from workspace_analysis.robot.fk import ForwardKinematics
    fk = ForwardKinematics(model)
    targets = {}; loaded = {}
    for port in ports:
        record = obj('ports/'+port+'/result.json'); spec = parse_task(obj('ports/'+port+'/task_spec.json'))
        if spec.data['collision']['gripper_rad'] != manifest['identity']['gripper_state_rad']:
            raise ReplayError('GRIPPER_IDENTITY','task and saved evaluation state differ')
        if not np.allclose(fk(start['q'])@spec.T_ee_tcp,start['T_base_tcp'],atol=1e-10,rtol=0):
            raise ReplayError('START_TCP_IDENTITY','common start TCP differs from authoritative FK')
        targets[port] = build_targets(spec, scene); goals = []
        for goal in record['goals']:
            if not goal['candidate']: continue
            paths = {}
            for trial in goal['paths']:
                if not trial['success']: continue
                t = _trajectory(store, files, 'ports/'+port+'/'+trial['trajectory_file'], 'Path')
                if not np.array_equal(t.q[0], start['q']): raise ReplayError('INVALID_START', 'path common start differs')
                paths[trial['method']] = t
            approach = None
            if goal['approach'].get('success') is True:
                approach = _trajectory(store, files, 'ports/'+port+'/'+goal['approach']['trajectory_file'], 'Approach')
            if paths: goals.append(Goal(goal['solution_id'], tuple(goal['q']), goal['status'], paths, approach))
        best = record['reachability'].get('ik',{}).get('best_attempt')
        detail = record['status']
        if best is not None:
            e = best['errors']
            detail += f" | best TCP {e['position_mm']:.4f} mm; axis {e['insertion_axis_deg']:.4f} deg; key {e['key_axis_deg']:.4f} deg"
        loaded[port] = Port(port, record['status'], detail, tuple(goals))
    # Rehash after verification/readback; no changed input can produce a catalog.
    if store.read('manifest.json') != original_manifest: raise ReplayError('ARTIFACT_CHANGED', 'manifest')
    for name, h in files.items(): store.read(name, digest=h)
    for name, path in bindings.items():
        rel = path.relative_to(root).as_posix(); workspace.read(rel, digest=manifest['input_hashes'][name])
    return Catalog(loaded, model, scene, targets, tuple(start['q']), np.asarray(start['T_base_tcp']),
        manifest['identity']['gripper_state_rad'], sha(original_manifest),
        {'status':'PASS','scope':'FULL_T5.1_VERIFIER_WITH_EXPLICIT_HASH_BOUND_INPUT_RELOCATION',
         'full_verification':full,'model_assets':asset_check,'manifest_sha256':sha(original_manifest),
         'input_bindings':{k:str(v) for k,v in bindings.items()},'native_backend_started':False,
         'IK_or_path_search_executed':False,'original_data_modified':False,'source_git_head':manifest['git_head']})


def main() -> int:
    sys.dont_write_bytecode = True
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace-root',required=True); p.add_argument('--benchmark-dir',required=True)
    p.add_argument('--input-profile',required=True); p.add_argument('--expected-manifest-sha256')
    p.add_argument('--json-output',type=Path)
    a = p.parse_args()
    try:
        catalog = read_catalog(a.workspace_root,a.benchmark_dir,a.input_profile,expected_manifest_sha256=a.expected_manifest_sha256)
        report = {**catalog.verification,'catalog':catalog.describe()}
    except FileNotFoundError as exc: report = ReplayBlocked('MISSING_INPUT',str(exc)).report()
    except ReplayError as exc: report = exc.report()
    except Exception as exc: report = ReplayError('VERIFICATION_FAILED',str(exc)).report()
    if a.json_output:
        with a.json_output.open('x') as f: json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(report));return 0 if report['status']=='PASS' else 3 if report['status']=='BLOCKED' else 2
