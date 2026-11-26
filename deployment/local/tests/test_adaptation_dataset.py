import copy
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
spec=importlib.util.spec_from_file_location('dataset',Path(__file__).parents[1]/'validate_adaptation_dataset.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class DatasetTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        (self.root/'rgb.png').write_bytes(b'explicit synthetic placeholder; not camera evidence')
        self.manifest={'schema_version':1,'channels':m.CHANNELS,'units':['rad']*6,'state_semantics':'nominal_joint_position','action_semantics':'absolute_commanded_target','data_kind':'synthetic_test','max_alignment_ns':1,'sample_period_ns':10,'joint_bounds_rad':[[-3,3]]*6,'calibration_id':'synthetic','clock_domain':'test','camera_id':'wrist'}
        self.rows=[]
        for i in range(3):
            self.rows.append(dict(episode_id='e',split='train',scene_group='scene',data_kind='synthetic_test',calibration_id='synthetic',state_source='bus_feedback',action_source='controller_sent_target',instruction='insert cable',subtask='insert',clock_domain='test',timestamp_ns=i*10,state_timestamp_ns=i*10,image_timestamp_ns=i*10,command_timestamp_ns=i*10,target_timestamp_ns=i*10+10,frame_index=i,state=[i]*6,action=[-i]*6,image_path='rgb.png',image_sha256=hashlib.sha256((self.root/'rgb.png').read_bytes()).hexdigest(),camera_id='wrist'))
    def run_audit(self):return m.audit(self.rows,self.manifest,self.root)
    def test_independent_stats(self):
        r=self.run_audit();self.assertEqual(r['stats']['observation.state']['mean'],[1]*6);self.assertEqual(r['stats']['action']['mean'],[-1]*6)
    def test_validation_excluded(self):
        r=copy.deepcopy(self.rows[0]);r.update(episode_id='v',scene_group='v',split='validation',state=[3]*6);self.rows.append(r)
        self.assertEqual(self.run_audit()['stats']['observation.state']['count'],3)
    def test_rejections(self):
        for field,value in [('state',[float('nan')]*6),('action',[4]*6),('action_source','feedback'),('image_timestamp_ns',100),('frame_index',2),('image_sha256','bad'),('image_path','../escape'),('clock_domain','other')]:
            with self.subTest(field=field):
                old=self.rows[0][field];self.rows[0][field]=value
                with self.assertRaises(ValueError):self.run_audit()
                self.rows[0][field]=old
    def test_scene_leakage(self):
        self.rows[1]['split']='test'
        with self.assertRaises(ValueError):self.run_audit()
    def test_real_gate(self):
        self.manifest['data_kind']='real_demonstration'
        for r in self.rows:r.update(data_kind='real_demonstration',feedback_valid=True,physical_calibrated=False,timing_verified=True)
        with self.assertRaises(ValueError):self.run_audit()
    def test_empty_training_rejected(self):
        for r in self.rows:r['split']='test'
        with self.assertRaises(ValueError):self.run_audit()

if __name__=='__main__':unittest.main()
