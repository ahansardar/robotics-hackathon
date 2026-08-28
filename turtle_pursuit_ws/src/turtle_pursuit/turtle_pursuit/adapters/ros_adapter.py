from dataclasses import dataclass
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from turtle_pursuit.common.geometry import Pose2D, Velocity2D, quaternion_to_yaw, Command

@dataclass
class Observation:
    pose: Pose2D = None; velocity: Velocity2D = None; received: float = 0.0

class RosStateAdapter:
    """Replaceable ROS adapter. Algorithms only consume Pose2D/Velocity2D/scan tuples."""
    def __init__(self,node):
        self.node=node; self.catcher=Observation(); self.runner=Observation(); self.scan=None
        self.catcher_pub=node.create_publisher(TwistStamped,'/catcher/diffdrive_controller/cmd_vel',10)
        self.runner_pub=node.create_publisher(TwistStamped,'/runner/diffdrive_controller/cmd_vel',10)
        # The installed TurtleBot simulator publishes global Gazebo truth here.
        # Swap these subscriptions for odometry/perception in a competition adapter.
        node.create_subscription(Odometry,'/catcher/sim_ground_truth_pose',lambda m:self._odom(m,self.catcher),qos_profile_sensor_data)
        node.create_subscription(Odometry,'/runner/sim_ground_truth_pose',lambda m:self._odom(m,self.runner),qos_profile_sensor_data)
    def subscribe_scan(self, role):
        self.node.create_subscription(LaserScan,f'/{role}/scan',self._scan,qos_profile_sensor_data)
    def _odom(self,m,o):
        p=m.pose.pose.position; q=m.pose.pose.orientation; t=self.now()
        o.pose=Pose2D(p.x,p.y,quaternion_to_yaw(q.x,q.y,q.z,q.w),t)
        o.velocity=Velocity2D(m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z); o.received=t
    def _scan(self,m): self.scan=(list(m.ranges),m.angle_min,m.angle_increment,self.now())
    def now(self): return self.node.get_clock().now().nanoseconds*1e-9
    def get_catcher_pose(self): return self.catcher.pose
    def get_runner_pose(self): return self.runner.pose
    def get_catcher_velocity(self): return self.catcher.velocity
    def get_runner_velocity(self): return self.runner.velocity
    def get_obstacles(self): return self.scan
    def stale(self, timeout):
        n=self.now(); return not self.catcher.pose or not self.runner.pose or n-self.catcher.received>timeout or n-self.runner.received>timeout
    def _twist(self,c):
        m=TwistStamped(); m.header.stamp=self.node.get_clock().now().to_msg(); m.header.frame_id='base_link'; m.twist.linear.x=float(c.linear); m.twist.angular.z=float(c.angular); return m
    def send_catcher_velocity(self,c): self.catcher_pub.publish(self._twist(c))
    def send_runner_velocity(self,c): self.runner_pub.publish(self._twist(c))
    def stop_all(self): self.send_catcher_velocity(Command()); self.send_runner_velocity(Command())
