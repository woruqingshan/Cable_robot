#!/usr/bin/env python3
"""Generate full robot collision viewers from verified original G2 configurations."""
import argparse
from datetime import datetime,timezone
from hashlib import sha256
import json
from pathlib import Path
import sys
import uuid
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',type=Path,required=True)
    p.add_argument('--reference-run',type=Path,default=ROOT/'results/c2/20261103T110913Z_diagnostic_dcddb9f7')
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--samples',type=int,nargs='+',default=[8,73,0])
    args=p.parse_args()
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.workspace.results import load_results
    from workspace_analysis.collision.client import CollisionClient
    from workspace_analysis.collision.policy import load_policy
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.viz.collision import build_collision_figure,export_collision_viewer
    model=load_model(ROOT/'config/workspace.yaml');scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    arrays,metadata=load_results(ROOT/'results/workspace_n100_seed42',model=model)
    reference=json.loads((args.reference_run/'queries.json').read_text())
    if len(arrays['q'])!=100 or len(set(args.samples))!=len(args.samples) or any(not 0<=i<100 for i in args.samples):raise ValueError('unique original sample indices in 0..99 required')
    out=args.output_dir or ROOT/'results/c2_5'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
    out.parent.mkdir(parents=True,exist_ok=True);out.mkdir()
    (out/'.incomplete').write_text('C2.5 verification pending')
    selected={};checks=[];regions=[]
    def pairset(row):return sorted((p['kind'],*sorted([p['first'],p['second']])) for p in row['colliding_pairs'])
    with CollisionClient(args.backend) as client:
        ready=client.initialize(model,scene)
        for label in ['baseline','assembly_policy_v0']:
            client.set_policy(*load_policy(ROOT/f'config/collision/{label}.yaml',model))
            for i,q in enumerate(arrays['q']):
                old=reference[label][i]
                if old['sample_index']!=i or old['q']!=q.tolist():raise ValueError('reference row mismatch')
                reply=client.query(model,q)
                for key in ['self_collision','environment_collision','effective_self_collision','effective_decision','filtered_pairs','effective_self_pairs']:
                    if reply[key]!=old[key]:raise ValueError('C2 numeric regression: '+key)
                if pairset(reply)!=pairset(old):raise ValueError('contact pairs changed')
                checks.append({'policy':label,'sample_index':i,'unchanged':True})
                if label=='assembly_policy_v0' and i in args.samples:
                    row={**reply,'q':q.tolist(),'sample_index':i};selected[str(i)]=row
        # Extra gripper and joint3 poses expose contact-region dependence without changing policy.
        for name,q,gripper in [('gripper_0',[0]*5,0),('gripper_03',[0]*5,.3),('joint3_min',[0,0,-2.09,0,0],0),('joint3_max',[0,0,2.09,0,0],0)]:
            row=client.query(model,q,auxiliary_positions={'r_joint':gripper})
            regions.append({'name':name,'q':q,'gripper':gripper,'contacts':row.get('contacts_base',[]),'link_transforms_base':row['link_transforms_base']})
    meshes={}
    for i,row in selected.items():
        surfaces=robot_surfaces(model,ready,row)
        fig=build_collision_figure(scene,surfaces,row)
        export_collision_viewer(fig,row,out/f'sample_{i}.html')
        meshes[i]=[{'link':s['id'],'sha256':s['sha256'],'triangles':len(s['faces']),
                    'min_base':s['vertices'].min(axis=0).tolist(),'max_base':s['vertices'].max(axis=0).tolist(),
                    'T_base_collision':s['T_base_collision']} for s in surfaces]
    for name,value in [('selected_queries.json',selected),('mesh_audit.json',meshes),('contact_region_probes.json',regions),
                       ('verification.json',{'unchanged_queries':checks,'cleanup':client.cleanup,'source_npz_sha256':metadata['dataset']['sha256'],'robot_geometry_count':len(meshes[str(args.samples[0])]),'missing_geometry':['lidar_link'],'physical_validation':'PARTIAL'})]:
        with (out/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
    (out/'.incomplete').unlink()
    print(json.dumps({'output_dir':str(out),'samples':args.samples,'unchanged_native_queries':len(checks)}))
    return 0


if __name__=='__main__':raise SystemExit(main())
