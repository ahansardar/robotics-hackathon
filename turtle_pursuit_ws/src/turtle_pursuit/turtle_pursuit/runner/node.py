import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtle_pursuit.adapters.ros_adapter import RosStateAdapter
from turtle_pursuit.common.geometry import Command
from turtle_pursuit.control.motion import MotionLimiter, avoid_scan
from turtle_pursuit.runner.strategy import RunnerStrategy

class RunnerNode(Node):
    def __init__(self):
        super().__init__('runner_controller'); self.declare_parameter('strategy','strategic'); self.declare_parameter('seed',1); self.declare_parameter('control_rate',20.0)
        vals={'max_linear':.34,'max_angular':1.8,'linear_accel':.6,'angular_accel':2.5,'stale_timeout':.6,'arena_half':5.0,'boundary_margin':.55,'lookahead':1.25,'turn_gain':2.2,'distance_weight':1.0,'clearance_weight':1.7,'open_weight':.6,'smooth_weight':.35}
        for k,v in vals.items(): self.declare_parameter(k,v)
        self.cfg={k:self.get_parameter(k).value for k in vals}; self.adapter=RosStateAdapter(self); self.adapter.subscribe_scan('runner'); self.strategy=RunnerStrategy(self.cfg,self.get_parameter('seed').value); self.limiter=MotionLimiter(self.cfg['max_linear'],self.cfg['max_angular'],self.cfg['linear_accel'],self.cfg['angular_accel']); self.mode_pub=self.create_publisher(String,'/match/runner_mode',10); self.last=self.adapter.now(); self.create_timer(1/self.get_parameter('control_rate').value,self.tick)
    def tick(self):
        now=self.adapter.now(); dt=max(.001,min(.2,now-self.last)); self.last=now
        if self.adapter.stale(self.cfg['stale_timeout']): cmd=Command(); mode='WAITING'
        else:
            cmd=self.strategy.command(self.adapter.get_catcher_pose(),self.adapter.get_runner_pose(),self.get_parameter('strategy').value); mode=self.strategy.mode
            if self.adapter.scan: cmd=avoid_scan(cmd,*self.adapter.scan[:3])
        self.adapter.send_runner_velocity(self.limiter.apply(cmd,dt)); m=String(); m.data=mode; self.mode_pub.publish(m)
    def destroy_node(self):
        if rclpy.ok(): self.adapter.send_runner_velocity(Command())
        return super().destroy_node()
def main(args=None):
    rclpy.init(args=args); n=RunnerNode()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
