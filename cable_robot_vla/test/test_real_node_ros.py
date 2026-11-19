"""Run in an isolated ROS domain; these tests never command hardware."""
import json
import time
import pytest


def test_record_only_owns_no_actuator_publishers(tmp_path):
    rclpy=pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from cable_robot_vla.real_observation_node import RealObservationNode
    rclpy.init()
    node=None
    try:
        node=RealObservationNode(parameter_overrides=[Parameter("camera_topic",value="/fixture/image"),
            Parameter("scene_sha256",value="a"*64),Parameter("output_directory",value=str(tmp_path))])
        assert node.client is None
        owned=[p.topic_name for p in node.publishers]
        assert set(owned)<= {"/rosout","/parameter_events","/idc_vla/real/diagnostics"}
        assert "/idc_vla/real/diagnostics" in owned
        end=time.monotonic()+1.1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        assert node.counts["valid"]==0 and node.counts["rejected"]>0
        assert node.last_rejection=="CACHED_COMMAND_STATE"
        owned={publisher.topic_name for publisher in node.publishers}
        assert owned <= {"/rosout","/parameter_events","/idc_vla/real/diagnostics"}
        changed=node.set_parameters([Parameter("use_sim_time",value=True)])
        assert not changed[0].successful
        assert node.get_parameter("use_sim_time").value is False
        assert node.recorder.path.joinpath("observations.jsonl").read_text()==""
    finally:
        if node:node.close();node.destroy_node()
        rclpy.shutdown()


def test_sim_clock_rejected(tmp_path):
    rclpy=pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from cable_robot_vla.real_observation_node import RealObservationNode
    rclpy.init()
    try:
        with pytest.raises(Exception,match="system clock"):
            RealObservationNode(parameter_overrides=[Parameter("use_sim_time",value=True)])
    finally:rclpy.shutdown()


def test_network_failure_keeps_recorder_alive(tmp_path,monkeypatch):
    rclpy=pytest.importorskip('rclpy')
    import httpx
    from rclpy.parameter import Parameter
    from vla_protocol.observation import Observation
    from cable_robot_vla.policy_client import PolicyClient
    import cable_robot_vla.real_observation_node as module
    from deployment.local.test_support import observation
    from uuid import uuid4
    def failure(request):raise httpx.ConnectError('fixture offline',request=request)
    monkeypatch.setattr(module,'PolicyClient',lambda **kwargs:PolicyClient(transport=httpx.MockTransport(failure),**kwargs))
    class FixtureSource:
        session_id=uuid4().hex
        evidence={'hardware_validation':False,'origin':'TEST_FIXTURE'}
        image_provenance={'origin':'TEST_FIXTURE'}
        def snapshot(self,sequence,now):
            data=observation().model_dump()
            data.update(session_id=self.session_id,request_id=uuid4().hex,observation_seq=sequence,simulated=False,calibration_status='real_uncalibrated',
                source_profile={'profile_id':'real_uncalibrated_v1'},capabilities=['source_profiles_v1'])
            data['gripper']['simulated']=False
            for part in [data['joints'],data['gripper'],*data['images']]:part['stamp']={'nanoseconds':now,'clock_domain':'ros_system'}
            return Observation.model_validate(data)
        def close(self):pass
    rclpy.init();node=None
    try:
        node=module.RealObservationNode(parameter_overrides=[Parameter('camera_topic',value='/fixture/image'),
            Parameter('scene_sha256',value='a'*64),Parameter('output_directory',value=str(tmp_path)),
            Parameter('enable_inference',value=True)])
        node.core.close();node.core=FixtureSource()
        node._tick()
        deadline=time.monotonic()+2
        while node.client.busy and time.monotonic()<deadline:
            node._poll();time.sleep(.005)
        node._tick()
        assert node.counts['valid']==2
        assert node.counts['network_errors']>=1
        assert len((node.recorder.path/'observations.jsonl').read_text().splitlines())==2
    finally:
        if node:node.close();node.destroy_node()
        rclpy.shutdown()
