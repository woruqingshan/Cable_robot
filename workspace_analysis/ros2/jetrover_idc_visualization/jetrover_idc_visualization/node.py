"""Private-domain JointState/marker replay only. No controller clients or planners."""
import json
import math
import os
from queue import Queue, Empty
from threading import Thread
from typing import Callable
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from .controller import ReplayController
from .data import Catalog, joint_values
from .errors import ReplayBlocked, ReplayError
from .markers import scene_specs, port_specs, marker_array
from .protocol import NAMESPACE, JOINT_TOPIC, SCENE_TOPIC, PORT_TOPIC, COMMAND_TOPIC, STATUS_TOPIC, command


def require_isolated_ros() -> None:
    try: domain = int(os.environ.get('ROS_DOMAIN_ID','0'))
    except ValueError: domain = 0
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not 1 <= domain <= 101:
        raise ReplayError('ROS_ISOLATION', 'set ROS_LOCALHOST_ONLY=1 and private ROS_DOMAIN_ID in 1..101')


class ReplayNode(Node):
    """Own playback; background verification never touches ROS or GUI state."""
    def __init__(self, *, catalog_loader: Callable[[], Catalog] | None = None, **kwargs):
        require_isolated_ros()
        super().__init__('replay',namespace=NAMESPACE,**kwargs)
        if self.get_namespace() != NAMESPACE: raise ReplayError('ROS_ISOLATION','fixed replay namespace required')
        for key,default in [('workspace_root',''),('benchmark_dir',''),('input_profile',''),('expected_manifest_sha256',''),('replay_hz',20.)]:
            self.declare_parameter(key,default)
        hz = self.get_parameter('replay_hz').value
        if type(hz) not in (float,int) or not math.isfinite(hz) or not .1 <= hz <= 100:
            raise ReplayError('REPLAY_RATE','fixed visualization rate must be .1..100 Hz')
        durable = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.joints = self.create_publisher(JointState,JOINT_TOPIC,10)
        self.scene_pub = self.create_publisher(MarkerArray,SCENE_TOPIC,durable)
        self.ports_pub = self.create_publisher(MarkerArray,PORT_TOPIC,durable)
        self.status_pub = self.create_publisher(String,STATUS_TOPIC,durable)
        for pub,expected in [(self.joints,JOINT_TOPIC),(self.scene_pub,SCENE_TOPIC),(self.ports_pub,PORT_TOPIC),(self.status_pub,STATUS_TOPIC)]:
            if pub.topic_name != expected: raise ReplayError('ROS_ISOLATION','refusing remapped replay publisher')
        self.sub = self.create_subscription(String,COMMAND_TOPIC,self.receive,10)
        self.session_id = uuid.uuid4().hex; self.controller = None; self.catalog = None
        self.error = None; self.last_request = None; self.last_command_ok = None
        self.loading = True; self.queue = Queue(); self.closed = False; self.status_count = 0
        self._port_tick = 0
        inputs = {name:self.get_parameter(name).value for name in ('workspace_root','benchmark_dir','input_profile','expected_manifest_sha256')}
        def load():
            try:
                if catalog_loader is not None: catalog = catalog_loader()
                else:
                    from .reader import read_catalog
                    catalog = read_catalog(inputs['workspace_root'],inputs['benchmark_dir'],inputs['input_profile'],
                        expected_manifest_sha256=inputs['expected_manifest_sha256'] or None)
                self.queue.put(catalog)
            except Exception as exc: self.queue.put(exc)
        self.worker = Thread(target=load,name='read_only_artifact_verification',daemon=True); self.worker.start()
        self.timer = self.create_timer(1./hz,self.update)
        self.publish_status()

    def receive(self, message: String) -> None:
        try:
            from .storage import json_object
            if len(message.data.encode()) <= 4096:
                request = json_object(message.data).get('request_id')
                if isinstance(request,str) and len(request) <= 80: self.last_request = request
            if self.controller is None: raise ReplayBlocked('NOT_READY','artifact verification not complete')
            data = command(message.data,self.session_id,self.controller.revision)
            self.last_request = data['request_id']
            if data['action'] == 'select':
                self.controller.select(data['port_id'],data['goal_id'],data['planner'],data['phase'])
                self._port_tick = 0
            else: getattr(self.controller,data['action'])()
            self.error = None; self.last_command_ok = True
        except ReplayError as exc:
            self.error = exc.report(); self.last_command_ok = False
        self.publish_status()

    def update(self) -> None:
        if self.closed: return
        if self.loading:
            try: value = self.queue.get_nowait()
            except Empty: return
            self.loading = False
            if isinstance(value,Exception):
                self.error = value.report() if isinstance(value,ReplayError) else ReplayError('VERIFICATION_FAILED',str(value)).report()
                self.get_logger().error(json.dumps(self.error)); self.publish_status(); return
            self.catalog = value; self.controller = ReplayController(value)
            self.get_logger().info('Verified replay catalog '+value.manifest_sha256+' ports='+str(list(value.ports)))
            self.scene_pub.publish(marker_array(scene_specs(value.scene),value.model.base_frame,self.get_clock().now().to_msg(),clear=True))
        if self.controller is None: return
        frame = self.controller.tick(); stamp = self.get_clock().now().to_msg()
        names,values = joint_values(self.catalog.model,frame['q'],self.catalog.gripper)
        state = JointState(); state.header.stamp = stamp; state.name = list(names); state.position = list(values)
        self.joints.publish(state)
        if self._port_tick % 4 == 0:
            self.ports_pub.publish(marker_array(port_specs(self.catalog.scene,self.controller.port_id,
                self.catalog.targets[self.controller.port_id],tcp=frame['tcp'],trajectory=self.controller.sequence),
                self.catalog.model.base_frame,stamp,clear=True))
        self._port_tick += 1
        self.publish_status()

    def publish_status(self) -> None:
        self.status_count += 1
        data = {'version':1,'session_id':self.session_id,'status_sequence':self.status_count,
                'stage':'VERIFYING_ARTIFACTS' if self.loading else 'READY' if self.controller else 'BLOCKED',
                'error':self.error,'last_request_id':self.last_request,'last_command_ok':self.last_command_ok,
                'catalog':{} if self.catalog is None else self.catalog.describe(),
                'manifest_sha256':None if self.catalog is None else self.catalog.manifest_sha256,
                'usage':'RESEARCH_ONLY','fixed_sample_playback':True,'hardware_control':False}
        if self.controller is not None: data.update(self.controller.status())
        msg = String(); msg.data = json.dumps(data,allow_nan=False); self.status_pub.publish(msg)

    def destroy_node(self):
        self.closed = True
        if hasattr(self,'timer'): self.timer.cancel()
        return super().destroy_node()


def main(args=None):
    import sys
    from rclpy.executors import ExternalShutdownException
    sys.dont_write_bytecode = True
    rclpy.init(args=args); node = None
    try:
        node = ReplayNode(); rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
