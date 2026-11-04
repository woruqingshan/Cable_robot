"""Small declared synthetic samples for playback/transport tests, not planning evidence."""
from pathlib import Path
import numpy as np
from workspace_analysis.robot.model import model_from_xml
from workspace_analysis.robot.fk import ForwardKinematics
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.task.spec import load_task
from workspace_analysis.task.template import tasks_for_ports
from workspace_analysis.task.targets import build_targets
from jetrover_idc_visualization.data import Catalog, Goal, Port, Trajectory

ROOT = Path(__file__).resolve().parents[1]


def synthetic_model():
    links = '<link name="base_link"/>'+''.join(f'<link name="s{i}"/>' for i in range(5))+'<link name="end_effector_link"/><link name="finger"/><link name="mimic_finger"/>'
    joints = ''.join(f'<joint name="joint{i+1}" type="revolute"><parent link="'+('base_link' if i==0 else f's{i-1}')+
        f'"/><child link="s{i}"/><origin xyz="0 0 .02"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint>' for i in range(5))
    joints += '<joint name="tip" type="fixed"><parent link="s4"/><child link="end_effector_link"/></joint>'
    for name,link,mimic in [('r_joint','finger',''),('l_joint','mimic_finger','<mimic joint="r_joint" multiplier="-1" offset="0.1"/>')]:
        joints += f'<joint name="{name}" type="revolute"><parent link="end_effector_link"/><child link="{link}"/><axis xyz="1 0 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/>{mimic}</joint>'
    return model_from_xml('<robot name="synthetic_replay_transport">'+links+joints+'</robot>')


def catalog_fixture(model=None):
    model = synthetic_model() if model is None else model
    scene = load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    template = load_task(ROOT/'config/tasks/p22_research_v0.yaml')
    ports = [p['id'] for p in scene.ports]; specs = tasks_for_ports(template,scene,ports)
    targets = {s.data['target']['port_id']:build_targets(s,scene) for s in specs}
    fk = ForwardKinematics(model); start = np.zeros(5)
    def trajectory(q,name,phase):
        n = len(q)
        return Trajectory(q,np.array([fk(row)@template.T_ee_tcp for row in q]),np.ones(n,dtype=bool),
            np.arange(n,dtype=np.int64),np.full(n,.01),np.full(n,.02),tuple((name,i) for i in range(n)),(phase,)*n)
    goals = []
    for i in range(2):
        goal = np.array([.12,-.1,.15,-.05,.03])*(i+1)
        path = trajectory(np.linspace(start,goal,12),'synthetic/goal'+str(i)+'_path.npz','Path')
        approach = trajectory(np.linspace(goal,goal+.01,5),'synthetic/goal'+str(i)+'_approach.npz','Approach')
        goals.append(Goal(i,tuple(goal),'SYNTHETIC_PROTOCOL_TEST_ONLY',{'linear':path,'rrt_connect':path},approach))
    records = {p:Port(p,'SYNTHETIC_PROTOCOL_TEST_ONLY' if p in ('P12','P22') else 'IK_NO_SOLUTION_WITHIN_BUDGET',
        'Explicit synthetic transport fixture; not a collision/IK certificate',tuple(goals if p=='P22' else goals[:1] if p=='P12' else [])) for p in ports}
    return Catalog(records,model,scene,targets,tuple(start),fk(start)@template.T_ee_tcp,.3,'0'*64,
                   {'status':'PASS','scope':'SYNTHETIC_PROTOCOL_TEST_ONLY'})
