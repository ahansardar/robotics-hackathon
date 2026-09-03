import json, math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from turtle_pursuit.adapters.ros_adapter import RosStateAdapter
from turtle_pursuit.common.geometry import Command, distance, normalize_angle
from turtle_pursuit.control.motion import AdaptiveNavigator, MotionLimiter, boundary_recovery
from turtle_pursuit.perception.obstacles import ObstacleMapper
from turtle_pursuit.runner.strategy import RunnerStrategy

class RunnerNode(Node):
    def __init__(self):
        super().__init__('runner_controller'); self.declare_parameter('strategy','strategic'); self.declare_parameter('seed',1); self.declare_parameter('control_rate',20.0); self.declare_parameter('require_lidar',False); self.declare_parameter('require_camera',False); self.declare_parameter('sensor_timeout',1.0); self.declare_parameter('sensor_startup_grace',5.0); self.declare_parameter('ground_truth_timeout',1.0); self.declare_parameter('spawn_x',1.5); self.declare_parameter('spawn_y',0.0); self.declare_parameter('spawn_yaw',math.pi)
        vals={'max_linear':.70,'cruise_linear':.44,'runner_full_boost_distance':3.2,'runner_boost_distance':5.0,'shield_commit_distance':2.4,'max_angular':1.8,'linear_accel':1.2,'angular_accel':2.8,'stale_timeout':.6,'arena_half':5.0,'boundary_margin':.55,'lookahead':1.25,'turn_gain':2.2,'distance_weight':1.0,'clearance_weight':1.7,'open_weight':.6,'smooth_weight':.35,'emergency_escape_distance':1.15,'adversarial_break_weight':.9,'adversarial_interval':.8,'survival_radial_weight':3.5,'safe_feint_distance':2.6,'mixed_strategy_top_k':3,'mixed_strategy_hold':.6,'shield_radius':1.05,'shield_reach_weight':1.6,'shield_open_weight':3.0,'shield_lost_margin':.35,'shield_switch_hysteresis':.75,'shield_association_distance':.75,'shield_join_ratio':1.45,'shield_join_step':.65,'shield_orbit_step':.48,'obstacle_map_resolution':.15,'obstacle_map_range':4.5,'obstacle_wall_margin':.70,'obstacle_min_cluster_cells':4,'obstacle_map_min_range':.12,'obstacle_map_ttl':15.0,'obstacle_opponent_exclusion':.48,'obstacle_max_component_span':2.2,'obstacle_bridge_gap_cells':2,'lidar_stop_distance':.650,'lidar_influence_distance':1.350}
        vals.update({'navigator_robot_radius':.32,'navigator_clearance_cap':3.0,'navigator_stall_window':1.25,'navigator_stall_distance':.055,'navigator_recovery_time':1.2,'navigator_recovery_reverse_time':.35,'navigator_recovery_reverse_speed':.38,'navigator_recovery_escape_speed':.62,'navigator_recovery_cooldown':1.5,'navigator_heading_samples':48,'navigator_clearance_weight':1.35,'navigator_goal_weight':2.1,'navigator_continuity_weight':.75})
        for k,v in vals.items(): self.declare_parameter(k,v)
        self.declare_parameter('shield_obstacles',[],ParameterDescriptor(dynamic_typing=True))
        self.cfg={k:self.get_parameter(k).value for k in vals}; self.cfg['shield_obstacles']=self.get_parameter('shield_obstacles').value; self.adapter=RosStateAdapter(self); self.adapter.subscribe_scan('runner'); self.adapter.subscribe_camera('runner','catcher'); self.adapter.enable_self_odometry_fallback('runner',self.get_parameter('spawn_x').value,self.get_parameter('spawn_y').value,self.get_parameter('spawn_yaw').value,self.get_parameter('ground_truth_timeout').value); self.strategy=RunnerStrategy(self.cfg,self.get_parameter('seed').value); self.mapper=ObstacleMapper(self.cfg['arena_half'],self.cfg['obstacle_map_resolution'],self.cfg['obstacle_map_range'],self.cfg['obstacle_wall_margin'],self.cfg['obstacle_min_cluster_cells'],self.cfg['obstacle_map_min_range'],self.cfg['obstacle_map_ttl'],self.cfg['obstacle_opponent_exclusion'],self.cfg['obstacle_max_component_span'],self.cfg['obstacle_bridge_gap_cells']); self.navigator=AdaptiveNavigator(self.cfg); self.limiter=MotionLimiter(self.cfg['max_linear'],self.cfg['max_angular'],self.cfg['linear_accel'],self.cfg['angular_accel']); self.mode_pub=self.create_publisher(String,'/match/runner_mode',10); self.obstacle_pub=self.create_publisher(String,'/match/runner_obstacles',10); self.last=self.adapter.now(); self.sensor_started=self.last; self.create_timer(1/self.get_parameter('control_rate').value,self.tick)
    def tick(self):
        now=self.adapter.now(); dt=max(.001,min(.2,now-self.last)); self.last=now
        scan=self.adapter.get_scan(self.get_parameter('sensor_timeout').value)
        grace=now-self.sensor_started<self.get_parameter('sensor_startup_grace').value
        sensor_fault=not grace and ((self.get_parameter('require_lidar').value and scan is None) or (self.get_parameter('require_camera').value and not self.adapter.camera_fresh(self.get_parameter('sensor_timeout').value)))
        if sensor_fault: cmd=Command(); mode='SENSOR_FAULT'
        elif self.adapter.stale(self.cfg['stale_timeout']): cmd=Command(); mode='WAITING'
        else:
            catcher=self.adapter.get_catcher_pose(); runner=self.adapter.get_runner_pose()
            if scan:
                centers=self.mapper.update(runner,*scan[:3],exclude=(catcher,),stamp=scan[3]); self.strategy.set_obstacles(centers)
                message=String(); message.data=json.dumps({'stamp':scan[3],'centers':centers}); self.obstacle_pub.publish(message)
            cmd=self.strategy.command(catcher,runner,self.get_parameter('strategy').value); mode=self.strategy.mode
            if scan and self.strategy.target is not None:
                target=self.strategy.target; bearing=normalize_angle(math.atan2(target.y-runner.y,target.x-runner.x)-runner.yaw)
                # Exclude the Catcher's own body from local obstacle avoidance, mirroring
                # the Catcher's exclusion of the Runner below. Otherwise, as the Catcher
                # closes in, the reactive safety layer starts treating the thing the
                # Runner is trying to escape as a wall to route around -- exactly when it
                # should be committing hardest to the strategy layer's flee heading.
                catcher_bearing=normalize_angle(math.atan2(catcher.y-runner.y,catcher.x-runner.x)-runner.yaw)
                cmd,navigation=self.navigator.command(cmd,runner,*scan[:3],target_bearing=bearing,exclude_bearing=catcher_bearing,exclude_range=distance(runner,catcher),now=now)
                if navigation!='DIRECT':mode=f'{mode}/{navigation}'
            recovery=boundary_recovery(runner,self.cfg['arena_half'],self.cfg['boundary_margin'],self.cfg['max_linear'],self.cfg['turn_gain'])
            if recovery is not None:cmd=recovery; mode='BOUNDARY_RETURN'
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
