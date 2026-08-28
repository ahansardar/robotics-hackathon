import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtle_pursuit.adapters.ros_adapter import RosStateAdapter
from turtle_pursuit.catcher.strategy import CatcherStrategy
from turtle_pursuit.common.geometry import Command
from turtle_pursuit.control.motion import MotionLimiter, avoid_scan
from turtle_pursuit.tracking.velocity import VelocityEstimator

class CatcherNode(Node):
    def __init__(self):
        super().__init__('catcher_controller'); self.declare_parameter('strategy','predictive'); self.declare_parameter('control_rate',20.0)
        for k,v in {'max_linear':.46,'max_angular':1.8,'linear_accel':.7,'angular_accel':2.8,'stale_timeout':2.0,'prediction_horizon':4.0,'prediction_step':.2,'velocity_alpha':.35,'chase_distance':1.6,'capture_control_distance':.55,'capture_radius':.5,'capture_speed':.16,'turn_gain':2.2}.items(): self.declare_parameter(k,v)
        self.cfg={k:self.get_parameter(k).value for k in ('max_linear','max_angular','linear_accel','angular_accel','stale_timeout','prediction_horizon','prediction_step','velocity_alpha','chase_distance','capture_control_distance','capture_radius','capture_speed','turn_gain')}
        self.adapter=RosStateAdapter(self); self.adapter.subscribe_scan('catcher'); self.strategy=CatcherStrategy(self.cfg); self.est=VelocityEstimator(self.cfg['velocity_alpha']); self.limiter=MotionLimiter(self.cfg['max_linear'],self.cfg['max_angular'],self.cfg['linear_accel'],self.cfg['angular_accel']); self.mode_pub=self.create_publisher(String,'/match/catcher_mode',10); self.last=self.adapter.now(); self.create_timer(1.0/self.get_parameter('control_rate').value,self.tick)
    def tick(self):
        now=self.adapter.now(); dt=max(.001,min(.2,now-self.last)); self.last=now
        if self.adapter.stale(self.cfg['stale_timeout']): cmd=Command(); mode='SEARCH'
        else:
            rv=self.est.update(self.adapter.get_runner_pose()); cmd=self.strategy.command(self.adapter.get_catcher_pose(),self.adapter.get_runner_pose(),rv,self.get_parameter('strategy').value); mode=self.strategy.mode
            if self.adapter.scan: cmd=avoid_scan(cmd,*self.adapter.scan[:3])
        self.adapter.send_catcher_velocity(self.limiter.apply(cmd,dt)); m=String(); m.data=mode; self.mode_pub.publish(m)
    def destroy_node(self):
        if rclpy.ok(): self.adapter.send_catcher_velocity(Command())
        return super().destroy_node()
def main(args=None):
    rclpy.init(args=args); n=CatcherNode()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
