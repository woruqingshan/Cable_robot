"""Validate offline JetRover episode records and compute TRAIN-only independent stats.

This staging format is not a LeRobotDataset and grants no execution permission.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

CHANNELS = ['joint1','joint2','joint3','joint4','joint5','r_joint']


def vector(value):
    if not isinstance(value,list) or len(value)!=6 or any(type(x) not in (int,float) or not math.isfinite(x) for x in value):
        raise ValueError('expected six finite numbers')
    return value


class Moments:
    def __init__(self):
        self.n=0; self.mean=[0.0]*6; self.m2=[0.0]*6
    def add(self, xs):
        self.n+=1
        for i,x in enumerate(xs):
            d=x-self.mean[i]; self.mean[i]+=d/self.n; self.m2[i]+=d*(x-self.mean[i])
    def result(self):
        if not self.n: raise ValueError('no training frames')
        std=[math.sqrt(max(0,x/self.n)) for x in self.m2]
        return {'count':self.n,'mean':self.mean,'std':std,'zero_variance_channels':[CHANNELS[i] for i,x in enumerate(std) if x==0]}


def audit(records, manifest, base):
    if manifest['schema_version']!=1 or manifest['channels']!=CHANNELS or manifest['units']!=['rad']*6:
        raise ValueError('unsupported contract')
    if manifest['state_semantics']!='nominal_joint_position' or manifest['action_semantics']!='absolute_commanded_target':
        raise ValueError('unsupported semantics')
    if manifest['data_kind'] not in ('synthetic_test','real_demonstration'):
        raise ValueError('unknown provenance')
    tolerance=manifest['max_alignment_ns']; period=manifest['sample_period_ns']
    if type(tolerance)!=int or tolerance<0 or type(period)!=int or period<=0: raise ValueError('invalid timing policy')
    limits=manifest['joint_bounds_rad']
    if len(limits)!=6 or any(len(b)!=2 or not all(math.isfinite(x) for x in b) or b[0]>=b[1] for b in limits): raise ValueError('invalid bounds')
    splits={}; last={}; counts={}; moments={k:Moments() for k in ['observation.state','action']}
    for r in records:
        eid=r['episode_id']; split=r['split']; group=r['scene_group']
        if split not in ('train','validation','test'): raise ValueError('invalid split')
        if splits.setdefault(group,split)!=split: raise ValueError('scene leakage across splits')
        if eid in last and (last[eid]['split']!=split or last[eid]['scene_group']!=group): raise ValueError('episode split changed')
        if r['data_kind']!=manifest['data_kind'] or r['calibration_id']!=manifest['calibration_id']: raise ValueError('provenance mismatch')
        if manifest['data_kind']=='real_demonstration' and not (r['feedback_valid'] and r['physical_calibrated'] and r['timing_verified']):
            raise ValueError('real demonstration not eligible for training')
        if r['state_source']!='bus_feedback' or r['action_source']!='controller_sent_target': raise ValueError('state/action source confusion')
        if not r['instruction'].strip() or r['subtask'] not in ('insert','unlock','pullout'): raise ValueError('invalid task')
        if r['clock_domain']!=manifest['clock_domain']: raise ValueError('clock mismatch')
        for key in ['timestamp_ns','state_timestamp_ns','image_timestamp_ns','command_timestamp_ns','target_timestamp_ns','frame_index']:
            if type(r[key])!=int or r[key]<0: raise ValueError('invalid integer timestamp/index')
        if r['target_timestamp_ns']<r['command_timestamp_ns']: raise ValueError('target precedes command')
        for key in ['state_timestamp_ns','image_timestamp_ns','command_timestamp_ns']:
            if abs(r[key]-r['timestamp_ns'])>tolerance: raise ValueError('alignment outside declared tolerance')
        expected_index=last[eid]['frame_index']+1 if eid in last else 0
        if r['frame_index']!=expected_index: raise ValueError('frame discontinuity')
        if eid in last and abs(r['timestamp_ns']-last[eid]['timestamp_ns']-period)>tolerance: raise ValueError('cadence gap')
        for key in ('state','action'):
            values=vector(r[key])
            if any(not lo<=x<=hi for x,(lo,hi) in zip(values,limits)): raise ValueError('out of configured bounds; never clip')
        image=(base/r['image_path']).resolve()
        if not image.is_relative_to(base.resolve()) or not image.is_file(): raise ValueError('missing/escaping image')
        if hashlib.sha256(image.read_bytes()).hexdigest()!=r['image_sha256']: raise ValueError('image hash mismatch')
        if r['camera_id']!=manifest['camera_id']: raise ValueError('camera mismatch')
        last[eid]=r; counts[split]=counts.get(split,0)+1
        if split=='train':
            moments['observation.state'].add(r['state']); moments['action'].add(r['action'])
    return {'schema_version':1,'data_kind':manifest['data_kind'],'counts':counts,'episodes':len(last),
            'stats_source':'train frames only, population std, no clipping',
            'stats':{k:v.result() for k,v in moments.items()},'execution_allowed':False}


def main():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--records',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('output exists')
    with a.records.open() as f:
        result=audit((json.loads(line) for line in f if line.strip()),json.loads(a.manifest.read_text()),a.records.parent)
    result['input_sha256']={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in (a.manifest,a.records)}
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
