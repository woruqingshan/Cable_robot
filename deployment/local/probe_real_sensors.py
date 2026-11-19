#!/usr/bin/env python3
"""Bounded passive ROS audit. Never starts drivers or creates motion interfaces."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from uuid import uuid4


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", required=True)
    parser.add_argument("--duration",type=float,default=8.)
    parser.add_argument("--read-servo-positions",action="store_true")
    args=parser.parse_args()
    if not 0 < args.duration <= 30:
        parser.error("duration must be in (0,30]")
    output=Path(args.output_directory)/("probe_"+uuid4().hex)
    output.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo,JointState
    from cable_robot_vla.real_observation import rgb_message
    from vla_protocol.png import encode_rgb
    from hashlib import sha256
    rclpy.init();node=rclpy.create_node("idc_passive_sensor_audit")
    report={"schema_version":1,"started_utc":datetime.now(timezone.utc).isoformat(),
        "audit_mode":"read_only_no_motion", "streams":{},"servo_reads":[],"actuator_interfaces_created":[]}
    subscriptions=[]
    first={};times={};stamps={}
    def receive(topic,msg):
        ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        times.setdefault(topic,[]).append(time.monotonic());stamps.setdefault(topic,[]).append(ns)
        if topic in first:return
        row={"source_stamp_ns":ns,"frame_id":msg.header.frame_id}
        if isinstance(msg,Image):
            row.update(width=msg.width,height=msg.height,encoding=msg.encoding,step=msg.step,data_bytes=len(msg.data))
            if msg.encoding in ("rgb8","bgr8"):
                try:
                    rgb=rgb_message(msg);png=encode_rgb(rgb.width,rgb.height,rgb.data)
                    name="image_"+str(len(first))+".png";(output/name).write_bytes(png)
                    row.update(png_path=name,png_sha256=sha256(png).hexdigest(),converted_encoding="rgb8")
                except Exception as exc:row["conversion_error"]=type(exc).__name__+":"+str(exc)
        elif isinstance(msg,CameraInfo):
            row.update(width=msg.width,height=msg.height,k=list(msg.k),p=list(msg.p),r=list(msg.r),d=list(msg.d),distortion_model=msg.distortion_model)
        else:row.update(names=list(msg.name),positions=list(msg.position),velocity=list(msg.velocity),effort=list(msg.effort))
        first[topic]=row
    try:
        start=time.monotonic()
        while time.monotonic()-start<3:rclpy.spin_once(node,timeout_sec=.1)
        topics=node.get_topic_names_and_types()
        report.update(topics=topics,nodes=node.get_node_names_and_namespaces(),services=node.get_service_names_and_types())
        try:
            from rclpy.action import get_action_names_and_types
            report["actions"]=get_action_names_and_types(node)
        except ImportError:report["actions"]="API unavailable"
        for topic,types in topics:
            cls=Image if "sensor_msgs/msg/Image" in types else CameraInfo if "sensor_msgs/msg/CameraInfo" in types else JointState if "sensor_msgs/msg/JointState" in types else None
            if cls:subscriptions.append(node.create_subscription(cls,topic,lambda m,t=topic:receive(t,m),qos_profile_sensor_data))
        start=time.monotonic()
        while time.monotonic()-start<args.duration:rclpy.spin_once(node,timeout_sec=.1)
        for topic,row in first.items():
            arrivals=times[topic]
            row.update(samples=len(arrivals),receive_rate_hz=(len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else None,
                min_stamp_ns=min(stamps[topic]),max_stamp_ns=max(stamps[topic]))
            report["streams"][topic]=row
        if args.read_servo_positions:
            from ros_robot_controller_msgs.srv import GetBusServoState
            from ros_robot_controller_msgs.msg import GetBusServoCmd
            client=node.create_client(GetBusServoState,"/ros_robot_controller/bus_servo/get_state")
            if not client.wait_for_service(timeout_sec=2):
                report["servo_read_status"]="service_not_discovered; no request sent"
            else:
                for sample in range(3):
                    req=GetBusServoState.Request()
                    for servo_id in (1,2,3,4,5,10):
                        command=GetBusServoCmd();command.id=servo_id;command.get_position=1;req.cmd.append(command)
                    start_ns=node.get_clock().now().nanoseconds;tick=time.monotonic()
                    future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=3)
                    row={"sample":sample,"requested_ids":[1,2,3,4,5,10],"request_ns":start_ns,
                        "response_ns":node.get_clock().now().nanoseconds,"duration_s":time.monotonic()-tick,
                        "acquisition_timestamp":None,"valid_canonical_state":False}
                    if future.done() and future.result():
                        response=future.result();row.update(success=response.success,
                            states=[{"present_id":list(s.present_id),"position":list(s.position)} for s in response.state])
                    else:row["error"]="response_timeout"
                    report["servo_reads"].append(row)
        report["finished_utc"]=datetime.now(timezone.utc).isoformat()
    finally:
        (output/"audit.json").write_text(json.dumps(report,ensure_ascii=False,allow_nan=False,indent=2)+"\n")
        node.destroy_node();rclpy.shutdown()
    print(json.dumps({"evidence_directory":str(output),"visible_topics":len(report.get("topics",[])),
        "streams_received":len(report["streams"]),"servo_read_status":report.get("servo_read_status")},ensure_ascii=False))


if __name__=="__main__":main()
