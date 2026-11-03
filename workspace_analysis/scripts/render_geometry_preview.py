#!/usr/bin/env python3
"""Static scientific rendering of actual native Mesh/contact evidence (no image editing)."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--backend',type=Path,required=True);parser.add_argument('--audit-dir',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
    if a.output.exists():raise ValueError('refusing existing PNG')
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.collision.geometry_profile import materialized_profile,effective_render_data
    from workspace_analysis.collision.geometry_audit import joint_frame
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.collision.client import CollisionClient
    from workspace_analysis.collision.policy import load_policy
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    manifest=json.loads((a.audit_dir/'manifest.json').read_text());m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');profile=materialized_profile(manifest['profile'],m)
    with CollisionClient(a.backend) as c:
        ready=c.initialize(m,s,geometry_profile=profile);c.set_policy(*load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m));row=c.query(m,[0]*5,auxiliary_positions={'r_joint':.3},geometry_audit={'max_contacts_per_pair':32,'max_contacts':20000})
        source=robot_surfaces(m,ready,row);derived=effective_render_data(m,ready,row)
    fig=plt.figure(figsize=(15,9))
    pairs=[('back_shell_green_link','link1','joint1'),('link2','servo_link1','joint2'),('l_link','r_link','l_joint')]
    for column,(first,second,joint) in enumerate(pairs):
        inverse=np.linalg.inv(joint_frame(m,row,joint))
        for layer,surfaces in enumerate([source,derived]):
            ax=fig.add_subplot(2,3,layer*3+column+1,projection='3d');points=[]
            for name,color in [(first,'#ef4444'),(second,'#3b82f6')]:
                surface=next(s for s in surfaces if s['id']==name);v=surface['vertices']@inverse[:3,:3].T+inverse[:3,3];points.append(v)
                ax.add_collection3d(Poly3DCollection(v[surface['faces']],facecolor=color,edgecolor='none',alpha=.55))
            pts=np.vstack(points);center=(pts.max(0)+pts.min(0))/2;span=(pts.max(0)-pts.min(0)).max()*.56
            for setter,c0 in zip([ax.set_xlim,ax.set_ylim,ax.set_zlim],center):setter(c0-span,c0+span)
            ax.set_box_aspect([1,1,1]);ax.set_xlabel('X [m]');ax.set_ylabel('Y [m]');ax.set_zlabel('Z [m]');ax.view_init(25,-55)
            ax.set_title(('Official' if layer==0 else 'Pair-local diagnostic')+'\n'+first+' / '+second,fontsize=9)
    fig.suptitle('q=0, gripper=0.3 rad; unchanged official environment/non-target surfaces; no physical safety claim')
    fig.tight_layout();fig.savefig(a.output,dpi=160);plt.close(fig);print(a.output)
if __name__=='__main__':main()
