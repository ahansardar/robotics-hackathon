import json, math
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from turtle_pursuit.adapters.ros_adapter import RosStateAdapter
from turtle_pursuit.common.geometry import distance
from turtle_pursuit.evaluation.match import CaptureDetector

class EvaluatorNode(Node):
    def __init__(self):
        super().__init__('match_evaluator');
        for k,v in {'capture_radius':.5,'capture_hold':1.0,'match_duration':180.0,'stale_timeout':2.0,'result_file':'/tmp/turtle_pursuit_result.json'}.items(): self.declare_parameter(k,v)
        self.adapter=RosStateAdapter(self); self.detector=CaptureDetector(self.get_parameter('capture_radius').value,self.get_parameter('capture_hold').value); self.start=None; self.last_c=None; self.last_r=None; self.cpath=0.; self.rpath=0.; self.minimum=float('inf'); self.done=False; self.cmode=''; self.rmode=''; self.collisions=0; self.bump={'catcher':False,'runner':False}; self.state_pub=self.create_publisher(String,'/match/state',10); self.marker_pub=self.create_publisher(MarkerArray,'/match/markers',10); self.create_subscription(String,'/match/catcher_mode',lambda m:setattr(self,'cmode',m.data),10); self.create_subscription(String,'/match/runner_mode',lambda m:setattr(self,'rmode',m.data),10); self.create_subscription(HazardDetectionVector,'/catcher/hazard_detection',lambda m:self.hazard('catcher',m),10); self.create_subscription(HazardDetectionVector,'/runner/hazard_detection',lambda m:self.hazard('runner',m),10); self.create_timer(.05,self.tick)
    def hazard(self,role,msg):
        active=any(d.type in (HazardDetection.BUMP,HazardDetection.STALL) for d in msg.detections)
        if active and not self.bump[role]: self.collisions+=1
        self.bump[role]=active
    def tick(self):
        if self.done or self.adapter.stale(self.get_parameter('stale_timeout').value): return
        now=self.adapter.now(); c=self.adapter.get_catcher_pose(); r=self.adapter.get_runner_pose()
        if self.start is None: self.start=now; self.last_c=c; self.last_r=r
        elapsed=now-self.start; sep=distance(c,r); self.minimum=min(self.minimum,sep)
        self.cpath+=distance(c,self.last_c); self.rpath+=distance(r,self.last_r); self.last_c=c; self.last_r=r
        captured=self.detector.update(sep,now); timeout=elapsed>=self.get_parameter('match_duration').value
        state='CAPTURED' if captured else ('SURVIVED' if timeout else 'RUNNING')
        hold=0 if self.detector.entered is None else max(0.,now-self.detector.entered)
        duration=self.get_parameter('match_duration').value
        cv=self.adapter.get_catcher_velocity(); rv=self.adapter.get_runner_velocity()
        telemetry={'state':state,'elapsed':round(elapsed,3),'remaining':round(max(0.,duration-elapsed),3),'duration':duration,'separation':round(sep,3),'minimum_separation':round(self.minimum,3),'hold':round(hold,3),'hold_required':self.detector.hold,'capture_progress':round(min(1.,hold/self.detector.hold),3),'catcher_mode':self.cmode,'runner_mode':self.rmode,'catcher_speed':round(math.hypot(cv.vx,cv.vy),3) if cv else 0.,'runner_speed':round(math.hypot(rv.vx,rv.vy),3) if rv else 0.,'catcher_path_length':round(self.cpath,3),'runner_path_length':round(self.rpath,3),'collisions':self.collisions}
        msg=String(); msg.data=json.dumps(telemetry); self.state_pub.publish(msg); self.publish_markers(c,r,sep,state)
        if captured or timeout: self.finish(state,elapsed,sep)
    def publish_markers(self,c,r,sep,state):
        arr=MarkerArray()
        for ident,p,color in ((0,c,(1.,0.,0.)),(1,r,(0.,1.,0.))):
            m=Marker(); m.header.frame_id='odom'; m.header.stamp=self.get_clock().now().to_msg(); m.ns='robots'; m.id=ident; m.type=Marker.SPHERE; m.action=Marker.ADD; m.pose.position.x=p.x; m.pose.position.y=p.y; m.pose.position.z=.2; m.pose.orientation.w=1.; m.scale.x=m.scale.y=m.scale.z=.25; m.color.r,m.color.g,m.color.b=color; m.color.a=1.; arr.markers.append(m)
        ring=Marker(); ring.header=arr.markers[0].header; ring.ns='capture_radius'; ring.id=2; ring.type=Marker.CYLINDER; ring.action=Marker.ADD; ring.pose.position.x=r.x; ring.pose.position.y=r.y; ring.pose.position.z=.01; ring.pose.orientation.w=1.; ring.scale.x=ring.scale.y=2*self.get_parameter('capture_radius').value; ring.scale.z=.01; ring.color.r=1.; ring.color.g=.6; ring.color.a=.2; arr.markers.append(ring); self.marker_pub.publish(arr)
    def finish(self,state,elapsed,final_separation):
        self.done=True; self.adapter.stop_all(); result={'capture_success':state=='CAPTURED','result':state,'capture_time':round(elapsed,3) if state=='CAPTURED' else None,'survival_time':round(elapsed,3),'minimum_separation':round(self.minimum,3),'final_separation':round(final_separation,3),'collisions':self.collisions,'catcher_path_length':round(self.cpath,3),'runner_path_length':round(self.rpath,3),'catcher_mode':self.cmode,'runner_mode':self.rmode}; path=Path(self.get_parameter('result_file').value); path.parent.mkdir(parents=True,exist_ok=True); path.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8'); self.get_logger().info('FINAL RESULT '+json.dumps(result))
def main(args=None):
    rclpy.init(args=args); n=EvaluatorNode()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok(): n.adapter.stop_all()
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
