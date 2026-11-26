"""Offline audit: never loads a policy or model weights. Run with pinned model venv."""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import subprocess
import sys


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--lerobot', type=Path, required=True)
    p.add_argument('--observation', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('output already exists; preserve prior evidence')
    import os
    os.environ.update(HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1', TOKENIZERS_PARALLELISM='false')
    sys.path.insert(0, str(a.lerobot.resolve() / 'src'))
    import numpy as np
    import torch
    from safetensors.torch import load_file
    from lerobot.processor.normalize_processor import NormalizerProcessorStep, UnnormalizerProcessorStep
    from lerobot.lerobot_types import TransitionKey as K
    torch.set_num_threads(1)
    root = a.checkpoint
    result = {'evidence_level': 'official_normalization_steps', 'model_loaded': False,
              'lerobot_revision': subprocess.check_output(['git','-C',str(a.lerobot),'rev-parse','HEAD'], text=True).strip(),
              'files': {f.name: sha(f) for f in root.iterdir() if f.is_file()}, 'tests': []}
    steps = []
    for name, cls in [('policy_preprocessor', NormalizerProcessorStep), ('policy_postprocessor', UnnormalizerProcessorStep)]:
        spec = json.loads((root / (name + '.json')).read_text())
        entry = next(x for x in spec['steps'] if 'normalizer' in x['registry_name'])
        step = cls(**entry['config'], device='cpu')
        step.load_state_dict(load_file(str(root / entry['state_file']), device='cpu'))
        steps.append(step)
        result[name] = {'type': type(step).__name__, 'config': step.get_config(),
                        'stats_keys': sorted(step._tensor_stats), 'source': entry['state_file']}
    raw = json.loads(a.observation.read_text())
    state = raw['joints']['positions'] + [raw['gripper']['position']]
    def record(name, x, y):
        result['tests'].append({'name':name, 'shape':list(x.shape), 'dtype':str(x.dtype),
                                'before':x.tolist(), 'after':y.tolist(), 'unchanged':torch.equal(x,y)})
    for name, values in [('zero',[0]*6), ('nonzero',[1,-2,3,-4,5,-6]), ('recorded_state',state)]:
        x = torch.tensor([values], dtype=torch.float32)
        y = steps[0]({K.OBSERVATION:{'observation.state':x}})[K.OBSERVATION]['observation.state']
        record('state_'+name,x,y)
    for value in [0, 0.5]:
        x = torch.full((1,50,6), value, dtype=torch.float32)
        record('action_'+str(value),x,steps[1]({K.ACTION:x})[K.ACTION])
    # Synthetic controls are isolated new objects; never alter checkpoint steps/stat files.
    for inverse, cls in [(False,NormalizerProcessorStep),(True,UnnormalizerProcessorStep)]:
        key = 'action' if inverse else 'observation.state'
        control = cls(features={key:{'type':'ACTION' if inverse else 'STATE','shape':[6]}},
                      norm_map={'STATE':'MEAN_STD','ACTION':'MEAN_STD'},
                      stats={key:{'mean':np.arange(1,7,dtype=np.float32),'std':np.full(6,2,dtype=np.float32)}}, device='cpu')
        x = torch.zeros((1,50,6) if inverse else (1,6))
        y = control({K.ACTION:x})[K.ACTION] if inverse else control({K.OBSERVATION:{key:x}})[K.OBSERVATION][key]
        expected = torch.arange(1,7,dtype=torch.float32) * (1 if inverse else -0.5)
        assert torch.allclose(y,expected.expand_as(y))
        record('positive_control_'+key,x,y)
    # Full official processor factory, config only. No SmolVLAPolicy instantiation.
    try:
        from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
        from lerobot.policies.factory import make_pre_post_processors
        cfg = SmolVLAConfig.from_pretrained(str(root), local_files_only=True)
        cfg.device = 'cpu'
        pre, post = make_pre_post_processors(cfg, pretrained_path=str(root),
                         preprocessor_overrides={'device_processor':{'device':'cpu'}})
        frame = {'observation.state':torch.tensor([state],dtype=torch.float32),
                 'observation.images.camera1':torch.zeros((1,3,256,256)), 'task':['Inspect cable.'], 'robot_type':['jetrover']}
        before = frame['observation.state'].clone()
        batch = pre(frame)
        record('full_pre_state',before,batch['observation.state'])
        for v in [0,0.5]:
            x = torch.full((1,50,6),v,dtype=torch.float32)
            record('full_post_'+str(v),x,post(x))
        result['full_pipeline'] = {'status':'PASS', 'pre_steps':[type(s).__name__ for s in pre.steps],
                                   'post_steps':[type(s).__name__ for s in post.steps], 'device_override':'cpu',
                                   'scope':'processor only, synthetic RGB/task plus recorded state; no model forward'}
    except Exception as exc:
        result['full_pipeline'] = {'status':'NOT_RUN','reason':type(exc).__name__+': '+str(exc)}
    result['max_rss_kib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'full_pipeline':result['full_pipeline'],'max_rss_kib':result['max_rss_kib'],
                      'unchanged':{x['name']:x['unchanged'] for x in result['tests']}}))

if __name__ == '__main__':
    main()
