#!/usr/bin/env python3
"""Replay actual D2 sample indices into complete task-scoped C2.5 robot viewers."""
import argparse,json,sys,uuid
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--backend',type=Path,required=True);p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path);a=p.parse_args()
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.collision.client import CollisionClient
    from workspace_analysis.collision.policy import load_policy
    from workspace_analysis.collision.scope import load_scope
    from workspace_analysis.collision.geometry_profile import load_geometry_profile
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.viz.collision import build_collision_figure,export_collision_viewer
    m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');policy=load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m);scope=load_scope(ROOT/'config/collision/task_scope_v0.yaml',m,policy);manifest=json.loads((a.run_dir/'manifest.json').read_text());rows=json.loads((a.run_dir/'evaluations.json').read_text())['B_task_scope']
    assert scope[1]==manifest['identities']['B_task_scope']['task_scope_sha256']
    out=a.output_dir or a.run_dir/('examples_'+uuid.uuid4().hex);out.mkdir(parents=True,exist_ok=False)
    selected={i for i in [8,19,73] if i<len(rows)};members=set(manifest['scope']['assembly_members'])
    rules={'gripper_environment':lambda p:p['kind']=='environment' and bool(set(p['links'])&members),'gripper_base':lambda p:p['kind']=='self' and bool(set(p['links'])&members) and bool(set(p['links'])&{'base_link','back_shell_green_link','back_shell_black_link'}),'gripper_arm':lambda p:p['kind']=='self' and bool(set(p['links'])&members) and bool(set(p['links'])&{'link1','link2','link3','link4','link5','servo_link1','servo_link2'})}
    found={}
    for name,rule in rules.items():
        values=[r['sample_index'] for r in rows if any(rule(p) for p in r['remaining_collision_pairs'])];found[name]={'count':len(values),'first_index':values[0] if values else None}
        if values:selected.add(values[0])
    candidates=[r['sample_index'] for r in rows if r['task_scope_candidate'] is True]
    if candidates:selected.add(candidates[0])
    profile=load_geometry_profile(ROOT/'config/collision/geometry_profiles/official_full.yaml',m);records=[]
    with CollisionClient(a.backend,timeout=60) as c:
        ready=c.initialize(m,s,geometry_profile=profile);c.set_policy(*policy)
        for i in sorted(selected):
            r=rows[i];native=c.query(m,r['q'],auxiliary_positions={'r_joint':manifest['gripper_rad']},task_scope=scope);native.update(q=r['q'],sample_index=i)
            assert native['task_scope']['task_scope_blocking_pairs']==r['task_scope_blocking_pairs'];assert native['environment_collision']==r['environment_collision']
            np.testing.assert_allclose(np.array(native['T_base_to_ee'])[:3,3],r['ee_position'],atol=1e-12,rtol=0)
            export_collision_viewer(build_collision_figure(s,robot_surfaces(m,ready,native),native),native,out/f'sample_{i}.html');records.append(native)
    with (out/'examples.json').open('x') as f:json.dump({'source_run':str(a.run_dir),'source_scope_sha256':scope[1],'external_collision_examples':found,'candidate_indices':candidates,'queries':records},f,indent=2)
    print(out)
if __name__=='__main__':main()
