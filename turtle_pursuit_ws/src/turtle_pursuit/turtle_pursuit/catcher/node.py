import json, math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from turtle_pursuit.adapters.ros_adapter import RosStateAdapter
from turtle_pursuit.catcher.strategy import CatcherStrategy
from turtle_pursuit.common.geometry import Command, distance, normalize_angle
from turtle_pursuit.control.motion import AdaptiveNavigator, MotionLimiter, boundary_recovery
from turtle_pursuit.perception.obstacles import ObstacleMapper
from turtle_pursuit.tracking.velocity import VelocityEstimator

class CatcherNode(Node):
    def __init__(self):
        super().__init__('catcher_controller'); self.declare_parameter('strategy','predictive'); self.declare_parameter('control_rate',20.0); self.declare_parameter('require_lidar',False); self.declare_parameter('require_camera',False); self.declare_parameter('sensor_timeout',1.0); self.declare_parameter('sensor_startup_grace',5.0); self.declare_parameter('ground_truth_timeout',1.0); self.declare_parameter('spawn_x',-1.5); self.declare_parameter('spawn_y',0.0); self.declare_parameter('spawn_yaw',0.0)
        vals={'max_linear':.70,'cruise_linear':.44,'catcher_cruise_distance':1.0,'catcher_boost_distance':3.5,'max_angular':1.8,'linear_accel':1.2,'angular_accel':2.8,'stale_timeout':2.0,'prediction_horizon':4.0,'prediction_step':.2,'velocity_alpha':.35,'chase_distance':1.6,'capture_control_distance':.55,'capture_radius':.5,'capture_speed':.16,'turn_gain':2.2,'arena_half':5.0,'anti_shield_trigger':1.65,'anti_shield_radius':1.12,'anti_shield_step':.72,'flank_association_distance':.75,'flank_angular_velocity_threshold':.08,'obstacle_map_resolution':.15,'obstacle_map_range':4.5,'obstacle_wall_margin':.70,'obstacle_min_cluster_cells':4,'obstacle_map_min_range':.12,'obstacle_map_ttl':15.0,'obstacle_opponent_exclusion':.48,'obstacle_max_component_span':2.2,'obstacle_bridge_gap_cells':2,'lidar_stop_distance':.650,'lidar_influence_distance':1.350}
        vals.update({'boundary_margin':.55,'navigator_robot_radius':.32,'navigator_clearance_cap':3.0,'navigator_stall_window':1.25,'navigator_stall_distance':.055,'navigator_recovery_time':1.2,'navigator_recovery_reverse_time':.35,'navigator_recovery_reverse_speed':.38,'navigator_recovery_escape_speed':.62,'navigator_recovery_cooldown':1.5,'navigator_heading_samples':48,'navigator_clearance_weight':1.35,'navigator_goal_weight':2.1,'navigator_continuity_weight':.75,'catcher_navigator_robot_radius':.29,'catcher_navigator_clearance_weight':1.05,'catcher_navigator_goal_weight':3.2,'catcher_navigator_continuity_weight':.5})
        for k,v in vals.items(): self.declare_parameter(k,v)
        self.declare_parameter('shield_obstacles',[],ParameterDescriptor(dynamic_typing=True))
        self.cfg={k:self.get_parameter(k).value for k in vals}; self.cfg['shield_obstacles']=self.get_parameter('shield_obstacles').value
        navigator_cfg=dict(self.cfg); navigator_cfg.update({'navigator_robot_radius':self.cfg['catcher_navigator_robot_radius'],'navigator_clearance_weight':self.cfg['catcher_navigator_clearance_weight'],'navigator_goal_weight':self.cfg['catcher_navigator_goal_weight'],'navigator_continuity_weight':self.cfg['catcher_navigator_continuity_weight']})
        self.adapter=RosStateAdapter(self); self.adapter.subscribe_scan('catcher'); self.adapter.subscribe_camera('catcher','runner'); self.adapter.enable_self_odometry_fallback('catcher',self.get_parameter('spawn_x').value,self.get_parameter('spawn_y').value,self.get_parameter('spawn_yaw').value,self.get_parameter('ground_truth_timeout').value); self.strategy=CatcherStrategy(self.cfg); self.mapper=ObstacleMapper(self.cfg['arena_half'],self.cfg['obstacle_map_resolution'],self.cfg['obstacle_map_range'],self.cfg['obstacle_wall_margin'],self.cfg['obstacle_min_cluster_cells'],self.cfg['obstacle_map_min_range'],self.cfg['obstacle_map_ttl'],self.cfg['obstacle_opponent_exclusion'],self.cfg['obstacle_max_component_span'],self.cfg['obstacle_bridge_gap_cells']); self.navigator=AdaptiveNavigator(navigator_cfg); self.est=VelocityEstimator(self.cfg['velocity_alpha']); self.limiter=MotionLimiter(self.cfg['max_linear'],self.cfg['max_angular'],self.cfg['linear_accel'],self.cfg['angular_accel']); self.mode_pub=self.create_publisher(String,'/match/catcher_mode',10); self.obstacle_pub=self.create_publisher(String,'/match/catcher_obstacles',10); self.last=self.adapter.now(); self.sensor_started=self.last; self.create_timer(1.0/self.get_parameter('control_rate').value,self.tick)
    def tick(self):
        now=self.adapter.now(); dt=max(.001,min(.2,now-self.last)); self.last=now
        scan=self.adapter.get_scan(self.get_parameter('sensor_timeout').value)
        grace=now-self.sensor_started<self.get_parameter('sensor_startup_grace').value
        sensor_fault=not grace and ((self.get_parameter('require_lidar').value and scan is None) or (self.get_parameter('require_camera').value and not self.adapter.camera_fresh(self.get_parameter('sensor_timeout').value)))
        if sensor_fault: cmd=Command(); mode='SENSOR_FAULT'
        elif self.adapter.stale(self.cfg['stale_timeout']): cmd=Command(); mode='SEARCH'
        else:
            catcher=self.adapter.get_catcher_pose(); runner=self.adapter.get_runner_pose()
            if scan:
                centers=self.mapper.update(catcher,*scan[:3],exclude=(runner,),stamp=scan[3]); self.strategy.set_obstacles(centers)
                message=String(); message.data=json.dumps({'stamp':scan[3],'centers':centers}); self.obstacle_pub.publish(message)
            rv=self.est.update(runner); cmd=self.strategy.command(catcher,runner,rv,self.get_parameter('strategy').value); mode=self.strategy.mode
            if scan and self.strategy.target is not None:
                target=self.strategy.target; bearing=normalize_angle(math.atan2(target.y-catcher.y,target.x-catcher.x)-catcher.yaw)
                runner_bearing=normalize_angle(math.atan2(runner.y-catcher.y,runner.x-catcher.x)-catcher.yaw)
                cmd,navigation=self.navigator.command(cmd,catcher,*scan[:3],target_bearing=bearing,exclude_bearing=runner_bearing,exclude_range=distance(catcher,runner),now=now)
                if navigation!='DIRECT':mode=f'{mode}/{navigation}'
            recovery=boundary_recovery(catcher,self.cfg['arena_half'],self.cfg.get('boundary_margin',.55),self.cfg['max_linear'],self.cfg['turn_gain'])
            if recovery is not None:cmd=recovery; mode='BOUNDARY_RETURN'
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
