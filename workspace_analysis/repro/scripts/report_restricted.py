#!/usr/bin/env python3
"""Summarize recorded T5.3-A failures with explicit stage denominators."""
import argparse
import json
from pathlib import Path
from hashlib import sha256
from collections import Counter
import sys
sys.dont_write_bytecode=True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    run=args.run.resolve()
    manifest=json.loads((run/'manifest.json').read_text())
    for name,h in manifest['files'].items():
        path=(run/name).resolve()
        if not path.is_relative_to(run) or sha256(path.read_bytes()).hexdigest()!=h:
            raise ValueError('artifact hash/path differs: '+name)
    args.output_dir.mkdir(parents=True,exist_ok=False)
    failures=[];totals=Counter();conditions=[];stage_statistics=[];planner_statistics=[]
    def failure(condition,goal,stage,check,source):
        states=check.get('states',[])
        reasons=Counter(code for state in states for code in state['reasons'])
        pairs=Counter((p['first'],p['second']) for state in states for p in state.get('environment_pairs',[]))
        stage_statistics.append({'condition':condition,'goal_solution_id':goal,'stage':stage,
            'checked_states':len(states),'rejected_states':sum(not s['valid'] for s in states),
            'reason_occurrences':dict(reasons),
            'environment_pairs':[{'first':a,'second':b,'sample_occurrences':n} for (a,b),n in pairs.items()]})
        state=next((s for s in check.get('states',[]) if not s['valid']),None)
        if state is None:
            return
        index=next(i for i,s in enumerate(check['states']) if not s['valid'])
        provenance=check.get('provenance',[])
        failures.append({'condition':condition,'goal_solution_id':goal,'stage':stage,
            'failure_codes':['JOINT_LIMIT' if c=='URDF_JOINT_LIMIT' else c for c in state['reasons']],
            'world_tcp_xyz_m':state.get('workspace',{}).get('world_tcp_xyz_m'),
            'q':state['q'],'environment_pairs':state.get('environment_pairs'),
            'semantic_blocking_pairs':state.get('semantic_blocking_pairs'),
            'workspace_violation':state.get('workspace',{}).get('violations'),
            'first_failed_sample':index,'source_provenance':provenance[index] if provenance else None,
            'clearance':state.get('semantic_clearance'),'source_artifact':source})
    for name in manifest['conditions']:
        condition=run/name
        result=json.loads((condition/'result.json').read_text())
        ledger=json.loads((condition/'collision_queries.json').read_text())['records']
        counts=Counter()
        for goal in result['goals']:
            counts['saved_goals']+=1
            eligible=result['start_check']['valid'] and goal['goal_check']['valid'] and goal['task_check']['task_pose_feasible']
            counts['valid_endpoint_goal_pairs']+=int(eligible)
            counts['invalid_start']+=int(not result['start_check']['valid'])
            counts['invalid_goal']+=int(not goal['goal_check']['valid'])
            failure(name,goal['solution_id'],'START',{'states':[result['start_check']]},str(condition/'result.json'))
            failure(name,goal['solution_id'],'GOAL',{'states':[goal['goal_check']]},str(condition/'result.json'))
            for method,check in goal['historical_paths'].items():
                counts['historical_'+method+'_valid']+=int(check['valid'])
                failure(name,goal['solution_id'],'HISTORICAL_'+method.upper(),check,check['source_artifact'])
            prior_states=[goal['goal_check']]
            for check in [*goal['historical_paths'].values(),goal['historical_approach']]:
                prior_states.extend(check.get('states',[]))
            query_start=max(s['query_id'] for s in prior_states)+1
            for trial in goal['paths']:
                method=trial['method']
                query_count=trial['query_statistics']['native_collision_queries']
                native_states=ledger[query_start:query_start+query_count]
                if len(native_states)!=query_count or [s['q'] for s in native_states[:2]]!=[result['start_q'],goal['q']]:
                    raise ValueError('planner query interval provenance differs')
                checked_clearance={}
                for kind in ('self','environment'):
                    values=[s['semantic_clearance']['native'][kind]['separation_m'] for s in native_states
                            if s.get('semantic_clearance',{}).get('native',{}).get(kind,{}).get('available')]
                    checked_clearance[kind]=min(values) if values else None
                planner_statistics.append({'condition':name,'goal':goal['solution_id'],'planner':method,
                    'status':trial['status'],'native_query_interval':[query_start,query_start+query_count],
                    'search_s':trial['search_elapsed_s'],'validation_s':trial['validation_elapsed_s'],
                    'path_length_rad':trial['path_length_rad'],
                    'path_clearance':None if trial['validation'] is None else trial['validation']['clearance'],
                    'all_queried_states_minimum_separation_m':checked_clearance,
                    'queried_states_include_rejected_search_states':True})
                query_start+=query_count
                counts[method+'_trials']+=1;counts[method+'_successes']+=int(trial['success'])
                if trial.get('validation') is not None:
                    failure(name,goal['solution_id'],'NEW_'+method.upper(),trial['validation'],str(condition/'result.json'))
                elif not trial['success']:
                    failures.append({'condition':name,'goal_solution_id':goal['solution_id'],'stage':'SEARCH_'+method.upper(),
                        'failure_codes':[trial['status']],'world_tcp_xyz_m':None,'q':None,'environment_pairs':None,
                        'workspace_violation':None,'first_failed_sample':None,'clearance':None,'source_artifact':str(condition/'result.json')})
            approach=goal['historical_approach']
            if approach.get('status')!='NOT_RUN':
                counts['approach_trials']+=1;counts['approach_valid']+=int(approach['valid'])
                failure(name,goal['solution_id'],'HISTORICAL_APPROACH',approach,approach['source_artifact'])
            reached=any(t['success'] for t in goal['paths'])
            counts['path_and_approach_valid_goals']+=int(reached and approach.get('valid',False))
        conditions.append({'condition':name,'workspace_on':result['workspace_config']['enabled'],**dict(counts)})
        totals.update(counts)
    def write(name,data):
        with (args.output_dir/name).open('x') as f:json.dump(data,f,indent=2,allow_nan=False)
    write('failure_diagnosis.json',{'usage':'RESEARCH_ONLY','records':failures,
        'failure_codes':dict(Counter(code for f in failures for code in f['failure_codes'])),
        'all_checked_stage_statistics':stage_statistics,
        'pair_frequency_is_diagnostic_only':True})
    write('analysis_summary.json',{'usage':'RESEARCH_ONLY','conditions':conditions,'totals':dict(totals),
        'planner_statistics':planner_statistics,
        'source_collection_manifest_sha256':sha256((run/'manifest.json').read_bytes()).hexdigest(),
        'denominators':'goals per condition; planner trials only for valid endpoints; approaches reported separately',
        'continuous_safety_proven':False})
    print(json.dumps({'conditions':len(conditions),'failures':len(failures),'totals':dict(totals)}))


if __name__=='__main__':main()
