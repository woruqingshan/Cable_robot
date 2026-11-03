#!/usr/bin/env python3
"""Paired complete C2.5 viewers using one D1 run's exact recorded geometry profiles."""
import argparse,json,sys,uuid
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--backend',type=Path,required=True);p.add_argument('--output-dir',type=Path);a=p.parse_args()
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.collision.client import CollisionClient
    from workspace_analysis.collision.policy import load_policy
    from workspace_analysis.collision.geometry_profile import materialized_profile
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.viz.collision import build_collision_figure,export_collision_viewer
    from workspace_analysis.workspace.results import load_results
    m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');manifest=json.loads((a.run_dir/'manifest.json').read_text());arrays,_=load_results(manifest['input_prefix'],model=m)
    out=a.output_dir or a.run_dir/('robot_viewers_'+uuid.uuid4().hex);out.mkdir(parents=True,exist_ok=False);records=[]
    for variant in ['official','refined']:
        profile=materialized_profile(manifest['profiles'][variant],m)
        with CollisionClient(a.backend,timeout=60) as client:
            ready=client.initialize(m,s,geometry_profile=profile);client.set_policy(*load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m))
            old=json.loads((a.run_dir/'idc'/variant/'evaluations.json').read_text())['assembly_policy_v1']
            for index in [None,8,73,19]:
                q=np.zeros(5) if index is None else arrays['q'][index]
                row=client.query(m,q,auxiliary_positions={'r_joint':manifest['gripper_rad']});row.update(q=q.tolist(),sample_index='reference_zero' if index is None else index)
                if index is not None:
                    for key in ['raw_self_collision','effective_self_collision','environment_collision']:assert row[key]==old[index][key]
                    np.testing.assert_allclose(np.array(row['T_base_to_ee'])[:3,3],old[index]['ee_position'],atol=1e-12,rtol=0)
                fig=build_collision_figure(s,robot_surfaces(m,ready,row),row)
                fig.update_layout(title=f'D1 {variant} / ACM v1 / {row["sample_index"]}: full official external surfaces; local derived self targets explicit in identity')
                export_collision_viewer(fig,row,out/f'{variant}_{row["sample_index"]}.html');records.append({'variant':variant,'query':row})
    with (out/'viewer_queries.json').open('x') as f:json.dump(records,f,indent=2)
    print(out)
if __name__=='__main__':main()
