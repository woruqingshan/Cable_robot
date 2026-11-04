"""Full real T5.1 readback and exact samples; no planner or Native process allowed."""
import json
import os
import tempfile
from pathlib import Path
import numpy as np
import pytest
from test_resources import ARTIFACTS
from jetrover_idc_visualization.reader import read_catalog
from jetrover_idc_visualization.controller import ReplayController
from jetrover_idc_visualization.errors import ReplayError

ROOT=Path(__file__).resolve().parents[1]
RUN='task_benchmark/t5_1_s1_20261104_03'
PROFILE=ROOT/'ros2/jetrover_idc_visualization/config/t51_inputs.json'


@pytest.fixture(scope='module')
def catalog():
    return read_catalog(ROOT,ARTIFACTS/RUN,PROFILE,
        expected_manifest_sha256='3e3a0d27123c197c8ef9a41c8e1a111e2181f0998740e0fa0ca95903d58cfbcd')


def test_real_ports_goals_planners_and_every_original_sample(catalog):
    assert len(catalog.ports['P12'].goals)==4 and len(catalog.ports['P22'].goals)==2
    n=0
    for port in catalog.ports.values():
        if not port.goals:
            assert port.status=='IK_NO_SOLUTION_WITHIN_BUDGET'
            c=ReplayController(catalog);c.select(port.port_id,None,None,'Path')
            with pytest.raises(ReplayError):c.play()
        for goal in port.goals:
            assert set(goal.paths)=={'linear','rrt_connect'}
            for trajectory in [*goal.paths.values(),goal.approach]:
                name=trajectory.source_frames[0][0]
                with np.load(ARTIFACTS/RUN/name,allow_pickle=False) as original:
                    np.testing.assert_array_equal(trajectory.q,original['q'])
                    np.testing.assert_array_equal(trajectory.tcp,original['T_base_tcp'])
                    np.testing.assert_array_equal(trajectory.query_id,original['query_id'])
                n+=len(trajectory.q)
            for method in goal.paths:
                c=ReplayController(catalog);c.select(port.port_id,goal.solution_id,method,'Path + Approach');c.play()
                rows=[]
                while c.state!='FINISHED':rows.append(c.tick()['q'])
                np.testing.assert_array_equal(rows,np.concatenate([goal.paths[method].q,goal.approach.q[1:]]))
    assert n>2000 and catalog.verification['full_verification']['paths_checked']==12
    assert catalog.verification['native_backend_started'] is False


def test_relocated_run_never_reads_original_absolute_run_paths_and_never_plans(tmp_path,monkeypatch):
    import workspace_analysis.planning.ik as ik
    import workspace_analysis.planning.rrt_connect as rrt
    from workspace_analysis.collision.client import CollisionClient
    def forbidden(*args,**kwargs):raise AssertionError('Replay must not run IK/RRT/Native')
    monkeypatch.setattr(ik.NumericalIK,'solve',forbidden)
    monkeypatch.setattr(rrt,'rrt_connect',forbidden)
    monkeypatch.setattr(CollisionClient,'__init__',forbidden)
    original=ARTIFACTS/RUN
    active=[False]
    import sys
    def guard(event,args):
        if active[0] and event=='open' and args and isinstance(args[0],(str,bytes)):
            p=Path(os.fsdecode(args[0])).absolute()
            if p==original or original in p.parents:raise AssertionError('implicit original run read: '+str(p))
    sys.addaudithook(guard)
    # Use a unique SAME-filesystem temporary output, not cross-mount /tmp links.
    # Only directory entries are created; no large copies or input-byte mutation.
    with tempfile.TemporaryDirectory(prefix='.t52_relocated_',dir=ARTIFACTS) as temporary:
        relocated=Path(temporary)
        for p in original.rglob('*'):
            if p.is_file():
                target=relocated/p.relative_to(original);target.parent.mkdir(parents=True,exist_ok=True);os.link(p,target)
        active[0]=True
        try:loaded=read_catalog(ROOT,relocated,PROFILE)
        finally:active[0]=False
    assert len(loaded.ports)==9 and loaded.verification['status']=='PASS'


def test_wrong_manifest_anchor_and_dynamic_path_reference_are_rejected(tmp_path):
    with pytest.raises(ReplayError,match='manifest.json'):
        read_catalog(ROOT,ARTIFACTS/RUN,PROFILE,expected_manifest_sha256='0'*64)
    # A forged small manifest cannot cause a read outside the run, even before hashing.
    (tmp_path/'manifest.json').write_text(json.dumps({'schema_version':1,'stage':'T5.1','status':'COMPLETED','files':{'../escape':'0'*64}}))
    with pytest.raises(ReplayError):read_catalog(ROOT,tmp_path,PROFILE)
