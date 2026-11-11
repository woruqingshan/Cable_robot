#!/usr/bin/env python3
"""Bounded RESEARCH_ONLY World/Base pilot using existing task benchmark algorithms."""
import argparse
import json
from pathlib import Path
import sys
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',type=Path,required=True)
    p.add_argument('--seed-prefix',type=Path,required=True)
    p.add_argument('--world-scene',type=Path,default=ROOT/'config/scenes/idc_prototype_v0.yaml')
    p.add_argument('--task-template',type=Path,default=ROOT/'config/tasks/p22_research_v0.yaml')
    p.add_argument('--pilot-config',type=Path,default=ROOT/'config/base_pose/s1_pilot_v0.json')
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--allow-candidate-baseline',action='store_true')
    p.add_argument('--max-time-s',type=float,default=20.)
    p.add_argument('--max-iterations',type=int,default=1500)
    a=p.parse_args()
    from workspace_analysis.experiments.base_pose_benchmark import run_base_pose_benchmark
    result=run_base_pose_benchmark(ROOT,a.backend,a.world_scene,a.task_template,a.seed_prefix,a.pilot_config,a.output_dir,
        allow_candidate=a.allow_candidate_baseline,planning={'max_time_s':a.max_time_s,'max_iterations':a.max_iterations})
    print(json.dumps({'output':str(a.output_dir),'successful_ports_by_station':result['successful_ports_by_station']}))


if __name__=='__main__':main()
