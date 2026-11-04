"""Responsive Qt control panel; only JSON commands/status, no artifact/FK code."""
import json
import signal
import sys
import time
import uuid
from PyQt5.QtCore import QTimer, QSignalBlocker
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QPushButton
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from .protocol import NAMESPACE, COMMAND_TOPIC, STATUS_TOPIC
from .storage import json_object


class ReplayPanel(QWidget):
    """ROS callbacks run in short Qt timer turns; expensive validation is elsewhere."""
    def __init__(self, node: Node):
        super().__init__(); self.node = node; self.latest = None; self.catalog = {}; self.session = None
        self.executor = SingleThreadedExecutor(context=node.context); self.executor.add_node(node)
        self.pending = None; self.pending_since = 0.; self.syncing = False
        self.setWindowTitle('JetRover IDC — verified research replay'); self.resize(660,370)
        layout = QVBoxLayout(self)
        title = QLabel('RESEARCH_ONLY — saved samples; no hardware execution / insertion')
        title.setWordWrap(True); layout.addWidget(title)
        row = QHBoxLayout(); layout.addLayout(row)
        self.ports = QComboBox(); self.goals = QComboBox(); self.planners = QComboBox(); self.phases = QComboBox()
        for label,name,widget in [('Port','ports',self.ports),('IK goal','goals',self.goals),
                                  ('Planner','planners',self.planners),('Phase','phases',self.phases)]:
            column = QVBoxLayout(); column.addWidget(QLabel(label)); widget.setObjectName(name); column.addWidget(widget); row.addLayout(column)
        buttons = QHBoxLayout(); layout.addLayout(buttons)
        self.play = QPushButton('Play'); self.pause = QPushButton('Pause'); self.reset = QPushButton('Reset')
        for name,widget in [('play',self.play),('pause',self.pause),('reset',self.reset)]:
            widget.setObjectName(name); buttons.addWidget(widget)
        self.info = QLabel('Waiting for /idc_replay/status …'); self.info.setWordWrap(True); self.info.setObjectName('status')
        layout.addWidget(self.info)
        self.identity = QLabel(''); self.identity.setWordWrap(True); layout.addWidget(self.identity)
        layout.addWidget(QLabel('Fixed sample rate is illustrative. Reset/selection jumps are virtual repositioning.'))
        from std_msgs.msg import String
        self.publisher = node.create_publisher(String,COMMAND_TOPIC,10)
        durable = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.sub = node.create_subscription(String,STATUS_TOPIC,self.receive,durable)
        self.ports.currentIndexChanged.connect(self.port_changed)
        self.goals.currentIndexChanged.connect(self.goal_changed)
        self.planners.currentIndexChanged.connect(self.select)
        self.phases.currentIndexChanged.connect(self.select)
        self.play.clicked.connect(lambda:self.send('play'))
        self.pause.clicked.connect(lambda:self.send('resume' if self.latest and self.latest.get('state') == 'PAUSED' else 'pause'))
        self.reset.clicked.connect(lambda:self.send('reset'))
        self.timer = QTimer(self); self.timer.timeout.connect(self.poll); self.timer.start(20)
        self.enable_controls(False)

    def poll(self):
        if rclpy.ok(context=self.node.context): self.executor.spin_once(timeout_sec=0.)
        if self.pending and time.monotonic()-self.pending_since > 5:
            self.pending = None; self.info.setText('Command acknowledgement timed out; check replay node status.')
            self.enable_controls(self.latest is not None and self.latest.get('stage') == 'READY')

    def receive(self, message):
        try:
            data = json_object(message.data)
            if data.get('version') != 1 or not isinstance(data.get('catalog'),dict): raise ValueError('invalid replay status')
            self.apply_status(data)
        except (ValueError,KeyError,TypeError) as exc:
            self.info.setText('Invalid replay status: '+str(exc)); self.enable_controls(False)

    def apply_status(self, data: dict) -> None:
        self.latest = data
        if self.session != data['session_id']:
            self.session = data['session_id']; self.pending = None; self.catalog = {}
        if self.pending == data.get('last_request_id'): self.pending = None
        ready = data.get('stage') == 'READY'
        if ready and self.pending is None:
            self.syncing = True
            try:
                changed = self.catalog != data['catalog']
                if changed:
                    self.catalog = data['catalog']; self.ports.clear()
                    for port in self.catalog: self.ports.addItem(port,port)
                desired = (data['port_id'],data.get('goal_id'),data.get('planner'),data.get('phase'))
                current = (self.ports.currentData(),self.goals.currentData(),self.planners.currentData(),self.phases.currentData())
                if changed or desired != current:
                    self.ports.setCurrentIndex(self.ports.findData(data['port_id']))
                    self.fill_goals(data.get('goal_id')); self.fill_planners(data.get('planner'),data.get('phase'))
            finally: self.syncing = False
        if ready:
            index = data.get('sample_index',0); total = data.get('total_samples',0)
            self.info.setText(f"{data['state']} | {data['port_id']} | goal {data.get('goal_id')} | {data.get('planner')} | "
                f"{data.get('current_phase') or data.get('phase')} | sample {index+1 if total else 0}/{total}\n"
                f"{data.get('task_detail','')}\n"+('' if not data.get('error') else str(data['error'])))
            self.identity.setText('Manifest SHA256: '+str(data.get('manifest_sha256')))
        else:
            self.info.setText(data.get('stage','WAITING')+'\n'+str(data.get('error') or 'Validating files, identities, FK and original samples …'))
        self.enable_controls(ready and self.pending is None)

    def fill_goals(self, selected=None):
        self.goals.clear(); port = self.catalog.get(self.ports.currentData(),{})
        for goal in port.get('goals',[]): self.goals.addItem('q'+str(goal['solution_id']),goal['solution_id'])
        if selected is not None: self.goals.setCurrentIndex(self.goals.findData(selected))

    def selected_goal(self):
        return next((g for g in self.catalog.get(self.ports.currentData(),{}).get('goals',[])
                     if g['solution_id'] == self.goals.currentData()),None)

    def fill_planners(self, selected=None, phase=None):
        self.planners.clear(); self.phases.clear(); goal = self.selected_goal()
        if goal is None: self.phases.addItem('Path','Path'); return
        for method in goal['planners']: self.planners.addItem('Linear' if method == 'linear' else 'RRT-Connect',method)
        for value in goal['phases']: self.phases.addItem(value,value)
        if selected is not None: self.planners.setCurrentIndex(self.planners.findData(selected))
        if phase is not None: self.phases.setCurrentIndex(self.phases.findData(phase))

    def port_changed(self):
        if self.syncing: return
        self.syncing = True
        try: self.fill_goals(); self.fill_planners()
        finally: self.syncing = False
        self.select()

    def goal_changed(self):
        if self.syncing: return
        self.syncing = True
        try: self.fill_planners()
        finally: self.syncing = False
        self.select()

    def select(self, *_):
        if self.syncing: return
        self.send('select',port_id=self.ports.currentData(),goal_id=self.goals.currentData(),
                  planner=self.planners.currentData(),phase=self.phases.currentData())

    def send(self, action: str, **kwargs):
        if not self.latest or self.latest.get('stage') != 'READY' or self.pending is not None: return
        from std_msgs.msg import String
        request = uuid.uuid4().hex
        payload = {'version':1,'session_id':self.session,'request_id':request,
                   'selection_revision':self.latest['selection_revision'],'action':action,**kwargs}
        message = String(); message.data = json.dumps(payload,allow_nan=False)
        self.pending = request; self.pending_since = time.monotonic(); self.enable_controls(False)
        self.publisher.publish(message)

    def enable_controls(self, ready: bool):
        playable = ready and bool(self.latest and self.latest.get('playable'))
        for widget in (self.ports,self.goals,self.planners,self.phases): widget.setEnabled(ready)
        if ready and not self.selected_goal():
            for widget in (self.goals,self.planners,self.phases): widget.setEnabled(False)
        state = None if self.latest is None else self.latest.get('state')
        self.play.setEnabled(playable and state in ('STOPPED','FINISHED'))
        self.pause.setEnabled(playable and state in ('PLAYING','PAUSED'))
        self.pause.setText('Resume' if state == 'PAUSED' else 'Pause')
        self.reset.setEnabled(playable)


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    from .node import require_isolated_ros
    sys.dont_write_bytecode = True; require_isolated_ros()
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication([sys.argv[0]]); node = Node('panel',namespace=NAMESPACE)
    panel = ReplayPanel(node); panel.show()
    signal.signal(signal.SIGINT,lambda *_:app.quit())
    signal.signal(signal.SIGTERM,lambda *_:app.quit())
    try: app.exec_()
    finally:
        panel.timer.stop(); panel.executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
