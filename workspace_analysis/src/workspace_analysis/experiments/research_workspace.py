"""RESEARCH_ONLY strict/candidate workspace comparison over unchanged G2/G4 rows.

The existing Evaluator owns all collision queries. This module preserves indices,
compares explicit semantic fields, audits removed pairs and reuses existing views.
"""
from collections import Counter
from datetime import datetime,timezone
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import numpy as np
from .assembly_semantics import _hash,_write
from .assembly_validation import validate_assembly
from ..collision.evaluator import Evaluator
from ..collision.semantic import SemanticConfig
from ..collision.assets import robot_surfaces
from ..robot.model import load_model
from ..scene.schema import load_scene
from ..workspace.results import load_results
from ..viz.workspace import prepare_view,build_3d_figure,standalone_html
from ..viz.collision import build_collision_figure,export_collision_viewer


def semantic_masks(rows):
    """Exclusive decision partition plus overlapping self/environment masks."""
    error=np.array([r.get('configuration_decision',{}).get('status')=='ERROR' or 'error' in r for r in rows],bool)
    passed=np.array([r.get('model_configuration_valid') is True for r in rows],bool)&~error
    rejected=np.array([r.get('model_configuration_valid') is False for r in rows],bool)&~error
    unknown=~(error|passed|rejected)
    if not np.all(passed.astype(int)+rejected+unknown+error==1):raise ValueError('semantic partition failure')
    return {'model_pass':passed,'rejected':rejected,'unknown':unknown,'error':error,
            'self_collision':np.array([r.get('semantic_self_collision') is True for r in rows],bool)&~error,
            'environment_collision':np.array([r.get('environment_collision') is True for r in rows],bool)&~error}


def summarize_semantic(rows):
    masks=semantic_masks(rows);n=len(rows)
    pairs=Counter(tuple(p) for r in rows if 'error' not in r for p in r.get('semantic_blocking_pairs',[]))
    env_pairs=Counter(tuple(sorted([p['first'],p['second']])) for r in rows if 'error' not in r for p in r.get('raw_collision_pairs',[]) if p['kind']=='environment')
    distances={}
    for kind in ['self','environment']:
        values=[r['semantic_clearance']['native'][kind]['separation_m'] for r in rows if 'semantic_clearance' in r and r['semantic_clearance']['native'][kind]['available']]
        values=[v for v in values if v is not None]
        distances[kind]={'available':len(values),'min_m':min(values) if values else None,'median_m':float(np.median(values)) if values else None,'p95_m':float(np.percentile(values,95)) if values else None,'max_m':max(values) if values else None}
    times=[r['query_time'] for r in rows]
    self_m=masks['self_collision'];env=masks['environment_collision']
    return {'usage':'RESEARCH_ONLY','total':n,'raw_self_collision':sum(r.get('raw_self_collision') is True for r in rows),
            'semantic_self_collision':int(self_m.sum()),'environment_collision':int(env.sum()),'both':int((self_m&env).sum()),
            'self_only':int((self_m&~env).sum()),'environment_only':int((env&~self_m).sum()),
            'clearance_only_rejected':int((masks['rejected']&~self_m&~env).sum()),
            'model_pass':int(masks['model_pass'].sum()),'research_candidates':sum(r.get('semantic_decision')=='RESEARCH_CANDIDATE' for r in rows),
            'rejected':int(masks['rejected'].sum()),'unknown':int(masks['unknown'].sum()),'error':int(masks['error'].sum()),
            'model_pass_ratio':float(masks['model_pass'].sum()/n) if n else None,
            'research_candidate_ratio':sum(r.get('semantic_decision')=='RESEARCH_CANDIDATE' for r in rows)/n if n else None,
            'blocking_pairs':[{'links':list(p),'count':v} for p,v in pairs.most_common()],
            'environment_pairs':[{'links':list(p),'count':v} for p,v in env_pairs.most_common()],
            'clearance_distribution':distances,'latency_s':{'mean':float(np.mean(times)) if times else None,'p95':float(np.percentile(times,95)) if times else None,'total':sum(times)},
            'normal_reference_status':'PENDING','hardware_validated':False,'planning_approved':False}


def compare_rows(strict,candidate,strict_pairs,candidate_pairs):
    """Account for every paired row; pair differences must follow the explicit ACM."""
    if len(strict)!=len(candidate):raise ValueError('paired row count differs')
    additional={tuple(p) for p in candidate_pairs}-{tuple(p) for p in strict_pairs}
    if not set(map(tuple,strict_pairs))<=set(map(tuple,candidate_pairs)):raise ValueError('candidate is not a superset of strict matrix')
    result=[]
    for a,b in zip(strict,candidate):
        if a['sample_index']!=b['sample_index'] or a['q']!=b['q']:raise ValueError('paired original index/q differs')
        if any('error' in r or r.get('model_configuration_valid') is None for r in [a,b]):raise ValueError('cannot compare failed/unknown row')
        if any(r.get('pairs_may_be_truncated') or r.get('semantic_pairs_may_be_truncated') for r in [a,b]):raise ValueError('pair list truncated; no absence inference allowed')
        for key in ['raw_self_collision','environment_collision','raw_collision_pairs','ee_position','ee_orientation']:
            if a[key]!=b[key]:raise ValueError('source geometry/state/environment changed: '+key)
        before=set(map(tuple,a['semantic_blocking_pairs']));after=set(map(tuple,b['semantic_blocking_pairs']))
        removed=before-after;added=after-before
        if added or not removed<=additional:raise ValueError('unexplained pair change')
        result.append({'sample_index':a['sample_index'],'q':a['q'],'ee_position':a['ee_position'],'ee_orientation':a['ee_orientation'],
            'strict_model_pass':a['model_configuration_valid'],'candidate_model_pass':b['model_configuration_valid'],
            'recovered_research_candidate':not a['model_configuration_valid'] and b['model_configuration_valid'],
            'newly_rejected':a['model_configuration_valid'] and not b['model_configuration_valid'],
            'strict_self_collision':a['semantic_self_collision'],'candidate_self_collision':b['semantic_self_collision'],
            'environment_collision':b['environment_collision'],'removed_pairs':[list(p) for p in sorted(removed)],
            'remaining_pairs':[list(p) for p in sorted(after)],'usage':'RESEARCH_ONLY'})
    return result


def check_input_rows(arrays,rows):
    if len(rows)!=len(arrays['q']):raise ValueError('input/output row count mismatch')
    for i,row in enumerate(rows):
        if row['sample_index']!=i or row['q']!=arrays['q'][i].tolist():raise ValueError('original sample index/q mutated')
        if 'error' in row:raise ValueError('native evaluation error: '+row['error'])
        if not np.allclose(row['ee_position'],arrays['position'][i],atol=1e-10,rtol=0) or not np.allclose(row['ee_orientation'],arrays['rotation'][i],atol=1e-10,rtol=0):raise ValueError('G2/FK correspondence changed')
        if row['configuration_decision']['planning_approved'] or row['configuration_decision']['hardware_validated']:raise ValueError('research result promoted to physical/planning approval')


def representative_indices(arrays,rows):
    """Deterministic representatives; never change counts or select a new q."""
    selected=[];m=semantic_masks(rows);candidates=np.flatnonzero(m['model_pass'])
    def add(role,index):
        if index is not None and int(index) not in [i for _,i in selected]:selected.append((role,int(index)))
    if len(candidates):
        add('candidate_first',candidates[0])
        distances=[rows[i]['semantic_clearance']['native']['self']['separation_m'] for i in candidates]
        available=[j for j,d in enumerate(distances) if d is not None]
        if available:add('candidate_largest_self_separation',candidates[max(available,key=lambda j:distances[j])])
        add('candidate_different_ee',candidates[int(np.argmax(np.linalg.norm(arrays['position'][candidates]-arrays['position'][candidates[0]],axis=1)))])
    env=np.flatnonzero(m['environment_collision']);add('environment_rejected',env[0] if len(env) else None)
    self_rej=np.flatnonzero(m['self_collision']);add('self_rejected',self_rej[0] if len(self_rej) else None)
    return selected


def _workspace_figure(arrays,metadata,rows,identity,label,prefix):
    view=prepare_view(arrays,metadata,source_prefix=str(prefix));fig=build_3d_figure(view)
    fig.data[0].update(visible='legendonly',name='Original geometric workspace')
    import plotly.graph_objects as go
    m=semantic_masks(rows)
    groups=[('Model pass — RESEARCH ONLY',m['model_pass'],'#16a34a'),('Semantic self collision',m['self_collision'],'#dc2626'),
            ('Environment collision',m['environment_collision'],'#f97316'),('Unknown/error',m['unknown']|m['error'],'#64748b')]
    for name,mask,color in groups:
        points=arrays['position'][mask]
        fig.add_trace(go.Scatter3d(x=points[:,0],y=points[:,1],z=points[:,2],mode='markers',name=f'{name}: {int(mask.sum())}',
            customdata=view.customdata[mask],hovertemplate=fig.data[0].hovertemplate,marker={'size':3,'color':color}))
    fig.update_layout(title=f'{label}, N={len(rows)} — RESEARCH_ONLY, normal reference PENDING',meta={'usage':'RESEARCH_ONLY','identity':identity,'summary':summarize_semantic(rows),'original_indices_preserved':True})
    fig.layout.annotations=()
    fig.add_annotation(text='Self/environment categories overlap. Green is model-only research evidence; no physical safety, RJ45 success or continuous path claim.',xref='paper',yref='paper',x=0,y=-.1,showarrow=False)
    return fig


def run_research_workspace(root,backend,output,prefixes,*,source,candidate_srdf,allow_candidate=False,gripper=.3,clearance=None,batch_size=64):
    root=Path(root).resolve();out=Path(output).absolute()
    if os.path.lexists(out):raise ValueError('refusing existing research output path')
    if not prefixes:raise ValueError('at least one validated input prefix required')
    model=load_model(root/'config/workspace.yaml');scene=load_scene(root/'config/scenes/idc_prototype_v0.yaml')
    strict=SemanticConfig.load(root/'config/collision/semantic/jetrover_semantic_v0.srdf',model)
    candidate=SemanticConfig.load(candidate_srdf,model,allow_candidate=allow_candidate)
    if candidate.review_status!='candidate_baseline_unverified':raise ValueError('research candidate profile required')
    loaded=[];counts=set()
    for value in prefixes:
        prefix=Path(value).resolve();arrays,metadata=load_results(prefix,model=model);n=len(arrays['q'])
        if n>1000 or n in counts:raise ValueError('distinct datasets of at most 1,000 rows required; no 10,000 gate override')
        counts.add(n);loaded.append((prefix,arrays,metadata))
    clear={'self_m':0.,'environment_m':0.} if clearance is None else dict(clearance)
    if set(clear)!={'self_m','environment_m'} or any(type(v) not in (int,float) or not math.isfinite(v) or v<0 for v in clear.values()):raise ValueError('finite nonnegative clearance required')
    model.joint_positions(model.limits.mean(axis=1),{'r_joint':gripper})
    if type(batch_size) is not int or batch_size<1:raise ValueError('positive integer batch size required')
    out.mkdir(parents=True);(out/'figures').mkdir();_write(out/'started.json',{'status':'STARTED','usage':'RESEARCH_ONLY'})
    audit=validate_assembly(root,backend,out,source,model,scene,strict,candidate)
    _write(out/'assembly_validation.json',audit)
    studies={};reviews=audit['pair_review'];representatives=[]
    configs={'strict':strict,'candidate':candidate}
    for prefix,arrays,metadata in loaded:
        tag='n'+str(len(arrays['q']));directory=out/tag;directory.mkdir();rows={};identities={};cleanups={};indices=[]
        input_hashes={ext:_hash(Path(str(prefix)+ext)) for ext in ['.npz','.json']}
        for label,config in configs.items():
            with Evaluator(model,scene,None,backend,semantic=config,gripper=gripper,clearance=clear,timeout=60) as e:
                values=e.evaluate(arrays['q'],batch_size=batch_size);check_input_rows(arrays,values)
                rows[label]=values;identities[label]=e.identity
                _write(directory/(label+'_ready.json'),e.ready)
                if label=='candidate':indices=representative_indices(arrays,values)
                # Strict/candidate viewer for the same first recovered research candidate.
                for role,index in (indices if label=='candidate' else []):
                    r=e.client.query(model,arrays['q'][index],auxiliary_positions={'r_joint':gripper});r.update(q=arrays['q'][index].tolist(),sample_index=index)
                    expected=values[index]
                    for key in ['raw_self_collision','semantic_self_collision','environment_collision','semantic_blocking_pairs']:
                        if r[key]!=expected[key]:raise ValueError('representative requery disagrees with evaluated row')
                    file=f'{tag}_{role}_{index}.html';fig=build_collision_figure(scene,robot_surfaces(model,e.ready,r),r);export_collision_viewer(fig,r,out/'figures'/file)
                    representatives.append({'dataset':tag,'input_prefix':str(prefix),'role':role,'sample_index':index,'q':r['q'],'ee_position':expected['ee_position'],'ee_orientation':expected['ee_orientation'],
                        'configuration_decision':expected['configuration_decision'],'identity':e.identity,'viewer':'figures/'+file,'native_query':r})
            cleanups[label]=e.client.cleanup
        # Requery strict at the same recovered q for a paired full-mesh viewer.

        if indices and indices[0][0].startswith('candidate'):
            index=indices[0][1]
            with Evaluator(model,scene,None,backend,semantic=strict,gripper=gripper,clearance=clear,timeout=60) as e:
                value=e.evaluate(arrays['q'][index:index+1],start_index=index)[0];r=dict(e.last_native);r.update(q=arrays['q'][index].tolist(),sample_index=index)
                file=f'{tag}_strict_same_candidate_{index}.html';export_collision_viewer(build_collision_figure(scene,robot_surfaces(model,e.ready,r),r),r,out/'figures'/file)
                representatives.append({'dataset':tag,'role':'strict_same_candidate','sample_index':index,'q':r['q'],'configuration_decision':value['configuration_decision'],'identity':e.identity,'viewer':'figures/'+file,'native_query':r})
            cleanups['strict_representative']=e.client.cleanup
        if not all(v['exit_normal'] for v in cleanups.values()):raise ValueError('evaluation backend exited abnormally')
        comparison=compare_rows(rows['strict'],rows['candidate'],strict.wire()['semantic']['disabled_pairs'],candidate.wire()['semantic']['disabled_pairs'])
        summaries={label:summarize_semantic(values) for label,values in rows.items()}
        for item in reviews['entries']:
            pair=tuple(item['links'])
            item['runtime_pair_observations'][tag]={'raw_contact_rows':sum(any(p['kind']=='self' and tuple(sorted([p['first'],p['second']]))==pair for p in r['raw_collision_pairs']) for r in rows['candidate']),
                'strict_blocking_rows':sum(list(pair) in r['semantic_blocking_pairs'] for r in rows['strict']),
                'candidate_blocking_rows':sum(list(pair) in r['semantic_blocking_pairs'] for r in rows['candidate']),
                'frequency_is_physical_permission':False}
        for label in configs:
            masks=semantic_masks(rows[label]);np.savez_compressed(directory/(label+'_workspace_results.npz'),**arrays,sample_index=np.arange(len(arrays['q'])),**masks)
            with (out/'figures'/f'{tag}_{label}_workspace.html').open('x') as f:f.write(standalone_html(_workspace_figure(arrays,metadata,rows[label],identities[label],label,prefix)))
        _write(directory/'evaluations.json',rows);_write(directory/'comparison.json',comparison)
        with (directory/'per_sample.csv').open('x',newline='') as f:
            fields=['model','sample_index','q','ee_position','ee_orientation','raw_self_collision','semantic_self_collision','environment_collision','semantic_decision','model_configuration_valid','semantic_blocking_pairs','disabled_collision_pairs','semantic_clearance','configuration_decision','query_time'];writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
            for label,values in rows.items():
                for r in values:writer.writerow({'model':label,**{k:json.dumps(r[k],separators=(',',':')) if isinstance(r.get(k),(dict,list)) else r.get(k) for k in fields if k!='model'}})
        with (directory/'comparison.csv').open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(comparison[0]));writer.writeheader()
            for row in comparison:writer.writerow({k:json.dumps(v) if isinstance(v,list) else v for k,v in row.items()})
        for ext,digest in input_hashes.items():
            if _hash(Path(str(prefix)+ext))!=digest:raise ValueError('historical input bytes changed')
        studies[tag]={'usage':'RESEARCH_ONLY','input_prefix':str(prefix),'input_hashes':input_hashes,'summary':summaries,'identities':identities,'cleanup':cleanups,
            'recovered_sample_indices':[r['sample_index'] for r in comparison if r['recovered_research_candidate']],
            'newly_rejected_indices':[r['sample_index'] for r in comparison if r['newly_rejected']],
            'pair_changes_explained':True,'all_q_fk_indices_preserved':True,'negative_z_retained':int((arrays['position'][:,2]<0).sum())}
        _write(directory/'summary.json',studies[tag])
        print(json.dumps({'dataset':tag,'strict_pass':summaries['strict']['model_pass'],'research_candidates':summaries['candidate']['research_candidates'],'candidate_self':summaries['candidate']['semantic_self_collision'],'environment':summaries['candidate']['environment_collision']}),flush=True)
    _write(out/'pair_review_29.json',reviews);_write(out/'representatives.json',representatives)
    _write(out/'summary.json',{'usage':'RESEARCH_ONLY','studies':studies,'normal_reference_status':'PENDING','whole_pair_physical_approval':'PENDING','planning_approved':False,'hardware_validated':False})
    files={str(p.relative_to(out)):_hash(p) for p in out.rglob('*') if p.is_file()}
    source_files={str(p.relative_to(root)):_hash(p) for name in ['src','scripts','cpp','config'] for p in (root/name).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    _write(out/'manifest.json',{'usage':'RESEARCH_ONLY','status':'COMPLETED','created_utc':datetime.now(timezone.utc).isoformat(),'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        'source_sha256':source_files,'input_datasets':{k:{'prefix':v['input_prefix'],'sha256':v['input_hashes']} for k,v in studies.items()},'gripper_rad':gripper,'clearance':clear,'studies':list(studies),'files':files})
    return studies


def verify_research_workspace(output,model):
    """Verify artifact bytes and exact original arrays; re-evaluate no collision queries."""
    out=Path(output).resolve();manifest=json.loads((out/'manifest.json').read_text())
    if manifest.get('status')!='COMPLETED' or manifest.get('usage')!='RESEARCH_ONLY':raise ValueError('not a completed research artifact')
    for name,digest in manifest['files'].items():
        path=(out/name).resolve()
        if not path.is_relative_to(out) or _hash(path)!=digest:raise ValueError('artifact hash/path mismatch: '+name)
    for tag,info in manifest['input_datasets'].items():
        arrays,_=load_results(info['prefix'],model=model)
        for ext,digest in info['sha256'].items():
            if _hash(info['prefix']+ext)!=digest:raise ValueError('input identity changed')
        rows=json.loads((out/tag/'evaluations.json').read_text())
        for label in ['strict','candidate']:
            check_input_rows(arrays,rows[label])
            with np.load(out/tag/(label+'_workspace_results.npz'),allow_pickle=False) as saved:
                for name,values in arrays.items():np.testing.assert_array_equal(saved[name],values)
                np.testing.assert_array_equal(saved['sample_index'],np.arange(len(arrays['q'])))
                for name,mask in semantic_masks(rows[label]).items():np.testing.assert_array_equal(saved[name],mask)
    return {'status':'PASS','usage':'RESEARCH_ONLY','artifact_hashes':len(manifest['files']),'datasets':list(manifest['input_datasets']),'q_fk_indices_unchanged':True}
