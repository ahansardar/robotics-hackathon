import math
from dataclasses import dataclass
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from rclpy.qos import qos_profile_sensor_data
from turtle_pursuit.common.geometry import Pose2D, Velocity2D, normalize_angle, quaternion_to_yaw, Command
from turtle_pursuit.perception.camera import detect_colored_target, detection_to_world

@dataclass
class Observation:
    pose: Pose2D = None; velocity: Velocity2D = None; received: float = 0.0
    # Last time this pose came from privileged sim ground truth, kept separate from
    # `received` so a wheel-odometry/camera fallback knows when it is safe to take over.
    gt_received: float = -1.0

class RosStateAdapter:
    """Replaceable ROS adapter. Algorithms only consume Pose2D/Velocity2D/scan tuples."""
    def __init__(self,node):
        self.node=node; self.catcher=Observation(); self.runner=Observation(); self.scan=None
        self.camera_received=0.0; self.camera_detection=None; self._color=None; self._depth=None; self._camera_info=None
        self._spawn={}; self._gt_timeout={}; self._odom_fallback_active={}
        defaults={
            'catcher_pose_topic':'/catcher/sim_ground_truth_pose',
            'runner_pose_topic':'/runner/sim_ground_truth_pose',
            'catcher_cmd_topic':'/catcher/diffdrive_controller/cmd_vel',
            'runner_cmd_topic':'/runner/diffdrive_controller/cmd_vel',
            'catcher_scan_topic':'/catcher/scan',
            'runner_scan_topic':'/runner/scan',
            'catcher_color_topic':'/catcher/camera/color/image_raw',
            'runner_color_topic':'/runner/camera/color/image_raw',
            'catcher_depth_topic':'/catcher/camera/depth/image_raw',
            'runner_depth_topic':'/runner/camera/depth/image_raw',
            'catcher_camera_info_topic':'/catcher/camera/color/camera_info',
            'runner_camera_info_topic':'/runner/camera/color/camera_info',
        }
        for name,value in defaults.items():
            if not node.has_parameter(name): node.declare_parameter(name,value)
        self.topics={name:node.get_parameter(name).value for name in defaults}
        self.catcher_pub=node.create_publisher(TwistStamped,self.topics['catcher_cmd_topic'],10)
        self.runner_pub=node.create_publisher(TwistStamped,self.topics['runner_cmd_topic'],10)
        # The installed TurtleBot simulator publishes global Gazebo truth here.
        # Swap these subscriptions for odometry/perception in a competition adapter.
        node.create_subscription(Odometry,self.topics['catcher_pose_topic'],lambda m:self._odom(m,self.catcher,'catcher'),qos_profile_sensor_data)
        node.create_subscription(Odometry,self.topics['runner_pose_topic'],lambda m:self._odom(m,self.runner,'runner'),qos_profile_sensor_data)
    def subscribe_scan(self, role):
        self.node.create_subscription(LaserScan,self.topics[f'{role}_scan_topic'],self._scan,qos_profile_sensor_data)
    def subscribe_camera(self, observer_role, target_role):
        self.node.create_subscription(Image,self.topics[f'{observer_role}_color_topic'],lambda m:self._image(m,observer_role,target_role),qos_profile_sensor_data)
        self.node.create_subscription(Image,self.topics[f'{observer_role}_depth_topic'],self._depth_image,qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo,self.topics[f'{observer_role}_camera_info_topic'],self._camera_information,qos_profile_sensor_data)
    def enable_self_odometry_fallback(self,role,spawn_x,spawn_y,spawn_yaw,ground_truth_timeout=1.0,odom_topic=None):
        """Keep this robot's own pose alive from wheel odometry if privileged sim
        ground truth is never published or goes stale mid-match, instead of the
        node silently freezing. Odometry starts at (0,0,0) in its own frame, so we
        compose it with the robot's known arena spawn pose (a fixed, one-time
        transform) to recover a world-frame estimate."""
        self._spawn[role]=Pose2D(spawn_x,spawn_y,spawn_yaw,0.0); self._gt_timeout[role]=ground_truth_timeout
        topic=odom_topic or f'/{role}/odom'
        self.node.create_subscription(Odometry,topic,lambda m:self._self_odom(m,role),qos_profile_sensor_data)
    def _odom(self,m,o,role=None):
        p=m.pose.pose.position; q=m.pose.pose.orientation; t=self.now()
        o.pose=Pose2D(p.x,p.y,quaternion_to_yaw(q.x,q.y,q.z,q.w),t)
        o.velocity=Velocity2D(m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z); o.received=t; o.gt_received=t
        # Mirror image of the warning in _self_odom: without this, a ground-truth
        # flicker (drop, recover, drop again) only ever logs its first occurrence,
        # since the fallback-active flag was otherwise write-only. Log recovery
        # and clear the flag so a second degradation later in the match logs again.
        if role is not None and self._odom_fallback_active.get(role,False):
            self._odom_fallback_active[role]=False
            self.node.get_logger().warn(f'{role}: sim_ground_truth_pose recovered, back to ground-truth localization')
    def _self_odom(self,m,role):
        obs=getattr(self,role); now=self.now()
        if now-obs.gt_received<=self._gt_timeout.get(role,1.0):
            return  # Ground truth is fresh; it is more accurate than integrated wheel odometry.
        if not self._odom_fallback_active.get(role,False):
            self._odom_fallback_active[role]=True
            self.node.get_logger().warn(f'{role}: sim_ground_truth_pose unavailable/stale, self-localizing from wheel odometry instead')
        spawn=self._spawn[role]
        p=m.pose.pose.position; q=m.pose.pose.orientation
        local_yaw=quaternion_to_yaw(q.x,q.y,q.z,q.w)
        cos_s,sin_s=math.cos(spawn.yaw),math.sin(spawn.yaw)
        world_yaw=normalize_angle(spawn.yaw+local_yaw)
        obs.pose=Pose2D(spawn.x+p.x*cos_s-p.y*sin_s,spawn.y+p.x*sin_s+p.y*cos_s,world_yaw,now)
        vx,vy,wz=m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z
        obs.velocity=Velocity2D(vx*math.cos(world_yaw)-vy*math.sin(world_yaw),vx*math.sin(world_yaw)+vy*math.cos(world_yaw),wz)
        obs.received=now
    def _scan(self,m): self.scan=(list(m.ranges),m.angle_min,m.angle_increment,self.now())
    def _depth_image(self,m): self._depth=m
    def _camera_information(self,m): self._camera_info=m
    def _image(self,m,observer_role,target_role):
        self._color=m; self.camera_received=self.now()
        color='red' if target_role=='catcher' else 'blue'
        self.camera_detection=detect_colored_target(m,self._depth,self._camera_info,color)
        observer=getattr(self,observer_role); target=getattr(self,target_role)
        if observer.pose is None or self.camera_detection is None or self.camera_detection.confidence < .15:
            return
        # Reject a detection implying a jump no TurtleBot 4 Lite could make
        # since the last trusted pose (defense in depth: color alone cannot
        # rule out every possible false-positive surface, only the ones
        # already tightened for in detect_colored_target). Skipped on the
        # very first detection, when there is no prior pose to check against.
        estimate=detection_to_world(observer.pose,self.camera_detection,previous=target.pose,max_speed=1.6)
        now=self.now()
        # Ground truth (if present and fresh) stays authoritative for the opponent too;
        # camera detection is what keeps tracking alive once it is not.
        ground_truth_fresh=now-target.gt_received<=self._gt_timeout.get(target_role,1.0)
        if estimate is not None and not ground_truth_fresh and (target.pose is None or now-target.received>.25):
            estimate.stamp=now; target.pose=estimate; target.received=now
    def now(self): return self.node.get_clock().now().nanoseconds*1e-9
    def get_catcher_pose(self): return self.catcher.pose
    def get_runner_pose(self): return self.runner.pose
    def get_catcher_velocity(self): return self.catcher.velocity
    def get_runner_velocity(self): return self.runner.velocity
    def get_obstacles(self): return self.scan
    def get_scan(self, timeout=1.0):
        return self.scan if self.scan is not None and self.now()-self.scan[3] <= timeout else None
    def camera_fresh(self, timeout=1.0): return self.camera_received>0 and self.now()-self.camera_received<=timeout
    def pose_stale(self, role, timeout):
        """Per-role freshness check. Lets a caller distinguish "I don't know
        where I am" (genuinely unsafe to command any motion) from "I don't
        currently know where my opponent is" (should keep operating on the
        best available -- possibly aged -- estimate rather than freezing;
        a robot still moving has some chance of reacquiring its target or
        continuing sensible evasion, a frozen one has none). `stale()` below
        keeps its existing both-roles semantics unchanged for the evaluator."""
        obs=getattr(self,role); n=self.now()
        return obs.pose is None or n-obs.received>timeout
    def stale(self, timeout):
        return self.pose_stale('catcher',timeout) or self.pose_stale('runner',timeout)
    def _twist(self,c):
        m=TwistStamped(); m.header.stamp=self.node.get_clock().now().to_msg(); m.header.frame_id='base_link'; m.twist.linear.x=float(c.linear); m.twist.angular.z=float(c.angular); return m
    def send_catcher_velocity(self,c): self.catcher_pub.publish(self._twist(c))
    def send_runner_velocity(self,c): self.runner_pub.publish(self._twist(c))
    def stop_all(self): self.send_catcher_velocity(Command()); self.send_runner_velocity(Command())
