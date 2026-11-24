"""Hash-frozen continuous overlay; prepares files only, never opens a device."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from .prepare_sdk_stage import prepare


def stage(output,vendor,revision,recordings,hz=20.):
    repo=Path(__file__).resolve().parents[2];output=Path(output)
    prepare(output,vendor,revision)
    paths=[]
    for directory in ('patches/jetrover_readback','cable_robot_control/cable_robot_control'):
        for p in (repo/directory).glob('*.py'):
            relative=p.relative_to(repo) if directory.startswith('patches') else Path('cable_robot_control')/p.name
            paths.append((p,relative))
    for name in ('__init__.py','hardware_feedback.py','robot_state_provider.py','readback_metrics.py'):
        paths.append((repo/'cable_robot_vla/cable_robot_vla'/name,Path('cable_robot_vla')/name))
    for source,relative in paths:
        dest=output/relative;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(source.read_bytes())
    cfg=Path('/home/ubuntu/ros2_ws/src/driver/servo_controller/config/servo_controller.yaml')
    (output/'mapping.yaml').write_bytes(cfg.read_bytes())
    params={'ros_robot_controller':{'ros__parameters':{
        'readback_maintenance':True,'feedback_hz':hz,'feedback_source_sha':revision,
        'feedback_output_directory':str(Path(recordings).resolve()),
        'feedback_configuration_path':str(output.resolve()/'mapping.yaml')}}}
    (output/'feedback.json').write_text(json.dumps(params,indent=2)+'\n')
    launcher='''from pathlib import Path
import hashlib,json,sys
root=Path(__file__).resolve().parent
m=json.loads((root/'feedback_manifest.json').read_text())
for name,h in m['files'].items():
 p=root/name
 if p.is_symlink() or hashlib.sha256(p.read_bytes()).hexdigest()!=h:raise RuntimeError('PACKAGE_HASH_MISMATCH:'+name)
sys.path.insert(0,str(root))
from patches.jetrover_readback import guarded_readback
sys.modules['ros_robot_controller.guarded_readback']=guarded_readback
sys.argv=[__file__,'--ros-args','--params-file',str(root/'feedback.json')]
from cable_robot_control.unified_feedback_node import main
main()
'''
    (output/'run_feedback.py').write_text(launcher)
    manifest=dict(source_sha=revision,mode='experimental_continuous',trusted=False,
        files={str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
               for p in sorted(output.rglob('*')) if p.is_file() and p.name!='feedback_manifest.json'})
    (output/'feedback_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--vendor-directory',type=Path,required=True)
    parser.add_argument('--recordings',type=Path,required=True)
    parser.add_argument('--hz',type=float,default=20.)
    a=parser.parse_args()
    if subprocess.check_output(['git','status','--porcelain'],text=True).strip():raise RuntimeError('COMMIT_FIRST')
    revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    m=stage(a.output,a.vendor_directory,revision,a.recordings,a.hz)
    print(json.dumps(dict(prepared_only=True,source_sha=revision,files=len(m['files']))))

if __name__=='__main__':main()
