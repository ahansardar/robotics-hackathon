import math, tempfile
from pathlib import Path
import struct
from types import SimpleNamespace
from turtle_pursuit.common.geometry import Pose2D, Velocity2D, normalize_angle, quaternion_to_yaw
from turtle_pursuit.tracking.velocity import VelocityEstimator
from turtle_pursuit.planning.interception import interception_point, project_pose
from turtle_pursuit.planning.grid import GridPlanner
from turtle_pursuit.runner.strategy import RunnerStrategy, candidate_score
from turtle_pursuit.catcher.strategy import CatcherStrategy
from turtle_pursuit.evaluation.match import CaptureDetector
from turtle_pursuit.common.config import load_config
from turtle_pursuit.control.motion import AdaptiveNavigator, MotionLimiter, avoid_scan, boundary_recovery, drive_to_bidirectional, dynamic_speed
from turtle_pursuit.common.geometry import Command
from turtle_pursuit.perception.camera import CameraDetection, detect_colored_target, detection_to_world
from turtle_pursuit.perception.obstacles import ObstacleMapper

def test_angles_and_quaternion():
    assert abs(abs(normalize_angle(3*math.pi))-math.pi)<1e-8
    assert abs(quaternion_to_yaw(0,0,math.sin(.5),math.cos(.5))-1)<1e-8
def test_velocity_estimator():
    e=VelocityEstimator(1.0); e.update(Pose2D(0,0,0,1)); v=e.update(Pose2D(2,0,.5,2)); assert v.vx==2 and v.wz==.5
def test_interception():
    p=interception_point(Pose2D(0,0),Pose2D(1,0),Velocity2D(0.1,0),1,3,.1); assert 1.0<=p.x<1.3
def test_curved_projection_follows_arc():
    p=project_pose(Pose2D(),Velocity2D(1,0,1),math.pi/2)
    assert abs(p.x-1)<1e-8 and abs(p.y-1)<1e-8
def test_grid_planner_avoids_wall():
    path=GridPlanner(5,5,{(2,0),(2,1),(2,2),(2,3)}).plan((0,0),(4,0)); assert path and (2,4) in path and not set(path)&{(2,0),(2,1),(2,2),(2,3)}
def test_capture_hold_continuous():
    d=CaptureDetector(.5,1); assert not d.update(.4,0); assert not d.update(.4,.9); assert not d.update(.6,1); assert not d.update(.4,2); assert d.update(.4,3.01)
def test_candidate_prefers_clear_space():
    cfg={'arena_half':5.,'distance_weight':1.,'clearance_weight':2.,'open_weight':1.,'smooth_weight':0.}
    c=Pose2D(0,0); r=Pose2D(3,0); assert candidate_score(c,r,Pose2D(3,2),0,cfg)>candidate_score(c,r,Pose2D(4.9,0),0,cfg)
def test_stale_safety_limiter_rejects_nan(): assert MotionLimiter().apply(Command(float('nan'),1),.1).linear==0
def test_dynamic_speed_is_smooth_and_bounded():
    assert dynamic_speed(.44,.70,-1.)==.44
    assert dynamic_speed(.44,.70,2.)==.70
    assert .44<dynamic_speed(.44,.70,.5)<.70

def test_runner_boosts_as_catcher_closes():
    cfg={'max_linear':.70,'cruise_linear':.44,'runner_full_boost_distance':3.2,'runner_boost_distance':5.}
    strategy=RunnerStrategy(cfg)
    assert strategy.speed_for_separation(3.0)==.70
    assert strategy.speed_for_separation(4.0)>strategy.speed_for_separation(6.0)

def test_catcher_boosts_over_open_distance():
    cfg={'max_linear':.70,'cruise_linear':.44,'catcher_cruise_distance':1.,'catcher_boost_distance':3.5}
    strategy=CatcherStrategy(cfg)
    assert strategy.speed_for_distance(4.)==.70
    assert strategy.speed_for_distance(2.)>strategy.speed_for_distance(.8)
def test_config_loading():
    p=Path(tempfile.mktemp()); p.write_text('control: {}\nmatch: {}\n'); assert load_config(p)=={'control':{},'match':{}}; p.unlink()

def test_standardized_runner_follows_fixed_course():
    cfg={'max_linear':.34,'arena_half':5.,'boundary_margin':.55,'lookahead':1.25,'turn_gain':2.2,'distance_weight':1.,'clearance_weight':1.,'open_weight':1.,'smooth_weight':1.}
    strategy=RunnerStrategy(cfg,seed=99)
    command=strategy.command(Pose2D(-1.5,0),Pose2D(1.5,0),'standardized')
    assert strategy.mode=='STANDARDIZED'
    assert strategy.target.x==3.6 and command.linear>0

def test_capture_controller_matches_moving_runner_speed():
    from turtle_pursuit.catcher.strategy import CatcherStrategy
    cfg={'max_linear':.46,'prediction_horizon':4.,'prediction_step':.2,'chase_distance':1.6,'capture_control_distance':.55,'capture_radius':.5,'capture_speed':.16,'turn_gain':2.2}
    strategy=CatcherStrategy(cfg)
    command=strategy.command(Pose2D(0,0),Pose2D(.5,0),Velocity2D(.3,0),'predictive')
    assert strategy.mode=='CAPTURE' and command.linear>=.3

def test_predictive_catcher_captures_circular_runner():
    from turtle_pursuit.evaluation.benchmark import run
    result=run('predictive','circular',0,20.)
    assert result['capture_success'] and result['capture_time']<10

def test_benchmark_collisions_are_measured_not_hardcoded():
    """`collisions` must reflect real body-contact transitions, not a
    placeholder -- a prior version hardcoded this field to 0 in every row,
    which read as validated collision safety that was never actually
    measured. A capture that closes to near-zero separation (stationary
    Runner, plenty of time) should register at least one contact transition;
    a deliberately generous collision_distance of 0 can never register one.
    """
    from turtle_pursuit.evaluation.benchmark import run
    closes_in=run('predictive','stationary',0,30.)
    assert closes_in['collisions']>=1
    never_touches=run('predictive','stationary',0,30.,collision_distance=0.0)
    assert never_touches['collisions']==0

def test_lidar_avoidance_does_not_reject_capture_target():
    ranges=[2.,2.,.5,2.,2.]
    desired=Command(.3,0.)
    blocked=avoid_scan(desired,ranges,-.2,.1)
    target=avoid_scan(desired,ranges,-.2,.1,target_bearing=0.,target_range=.75)
    assert blocked.linear<desired.linear and target.linear==desired.linear

def test_rgbd_detects_blue_target_and_projects_world_pose():
    width=height=12; pixels=bytearray(width*height*3)
    for y in range(4,8):
        for x in range(4,8): pixels[(y*width+x)*3+2]=255
    image=SimpleNamespace(encoding='rgb8',width=width,height=height,step=width*3,data=bytes(pixels))
    depth_data=b''.join(struct.pack('<f',2.) for _ in range(width*height))
    depth=SimpleNamespace(encoding='32FC1',width=width,height=height,step=width*4,data=depth_data,is_bigendian=False)
    info=SimpleNamespace(k=[10.,0.,6.,0.,10.,6.,0.,0.,1.])
    detection=detect_colored_target(image,depth,info,'blue')
    pose=detection_to_world(Pose2D(1.,2.,0.),detection)
    assert detection is not None and abs(detection.distance-2.)<1e-6
    assert pose.x>2.9 and abs(pose.y-2.)<.2

def _solid_patch_image(width,height,r,g,b,region=(4,8,4,8)):
    pixels=bytearray(width*height*3); x0,x1,y0,y1=region
    for y in range(y0,y1):
        for x in range(x0,x1):
            offset=(y*width+x)*3; pixels[offset]=r; pixels[offset+1]=g; pixels[offset+2]=b
    return SimpleNamespace(encoding='rgb8',width=width,height=height,step=width*3,data=bytes(pixels))

def test_camera_does_not_mistake_a_painted_obstacle_for_the_red_catcher():
    """Regression test: an arena obstacle painted diffuse (1, 0.55, 0.08) --
    roughly RGB 255/140/20 -- passed the old ratio-only red test even though
    it is a static obstacle, not the red Catcher marker. A vision fallback
    locking onto that obstacle would make a robot chase or evade furniture.
    """
    image=_solid_patch_image(12,12,255,140,20)
    assert detect_colored_target(image,None,None,'red') is None

def test_camera_still_detects_a_true_red_marker():
    image=_solid_patch_image(12,12,220,40,35)
    assert detect_colored_target(image,None,None,'red') is not None

def test_detection_to_world_rejects_an_implausible_jump():
    previous=Pose2D(0.,0.,0.,10.0)
    observer=Pose2D(0.,0.,0.,10.2)
    impossible=CameraDetection(bearing=0.0,distance=5.0,confidence=1.0)
    assert detection_to_world(observer,impossible,previous=previous,max_speed=1.5) is None
    plausible=CameraDetection(bearing=0.0,distance=0.2,confidence=1.0)
    assert detection_to_world(observer,plausible,previous=previous,max_speed=1.5) is not None

def test_runner_uses_breakaway_inside_emergency_radius():
    cfg={'max_linear':.34,'arena_half':5.,'boundary_margin':.55,'lookahead':1.25,'turn_gain':2.2,'distance_weight':1.,'clearance_weight':1.7,'open_weight':.6,'smooth_weight':.35,'emergency_escape_distance':1.15}
    strategy=RunnerStrategy(cfg,seed=1)
    strategy.command(Pose2D(0.,0.),Pose2D(.8,0.),'strategic')
    assert strategy.mode=='BREAKAWAY'

def test_adversarial_runner_uses_deterministic_maneuvers():
    cfg={'max_linear':.34,'arena_half':5.,'boundary_margin':.55,'lookahead':1.25,'turn_gain':2.2,'distance_weight':1.,'clearance_weight':1.7,'open_weight':.6,'smooth_weight':.35,'emergency_escape_distance':1.15,'adversarial_break_weight':.9,'adversarial_interval':.8}
    a=RunnerStrategy(cfg,seed=7); b=RunnerStrategy(cfg,seed=7); runner=Pose2D(2.,0.,0.,2.4); catcher=Pose2D(0.,0.)
    ca=a.command(catcher,runner,'adversarial'); cb=b.command(catcher,runner,'adversarial')
    assert a.mode=='ADVERSARIAL' and ca==cb

def test_aggressive_catcher_uses_pressure_mode():
    from turtle_pursuit.catcher.strategy import CatcherStrategy
    cfg={'max_linear':.46,'prediction_horizon':4.,'prediction_step':.2,'chase_distance':1.6,'capture_control_distance':.55,'capture_radius':.5,'capture_speed':.16,'turn_gain':2.2}
    strategy=CatcherStrategy(cfg)
    # A genuine time-consistent intercept must exist within the horizon for
    # PRESSURE to engage (self-arbitration falls back to CHASE otherwise); a
    # slow, nearby Runner comfortably allows one.
    strategy.command(Pose2D(0.,0.),Pose2D(1.5,0.),Velocity2D(.05,.02,.05),'aggressive')
    assert strategy.mode=='PRESSURE'

def test_catcher_falls_back_to_chase_when_no_feasible_intercept_exists():
    """When the CTRV search finds no real time-consistent intercept within the
    horizon, trust the honest fallback (chase the Runner's current position)
    instead of steering at an arbitrary extrapolated horizon-endpoint guess.

    The trust switch is debounced (a few sustained infeasible reads, not one),
    the same dwell-style fix used for FLANK, so a single flickered reading
    against an oscillating Runner can't chatter the aim point tick to tick.
    """
    from turtle_pursuit.catcher.strategy import CatcherStrategy
    cfg={'max_linear':.46,'prediction_horizon':4.,'prediction_step':.2,'chase_distance':1.6,'capture_control_distance':.55,'capture_radius':.5,'capture_speed':.16,'turn_gain':2.2}
    strategy=CatcherStrategy(cfg)
    for _ in range(4):
        strategy.command(Pose2D(0.,0.),Pose2D(3.,0.),Velocity2D(.2,.1,.1),'predictive')
    assert strategy.mode=='CHASE' and strategy.target.x==3. and strategy.target.y==0.

def test_predictive_catcher_flanks_a_shielding_runner_persistently():
    from turtle_pursuit.catcher.strategy import CatcherStrategy
    cfg={'max_linear':.46,'prediction_horizon':4.,'prediction_step':.2,'capture_control_distance':.55,'capture_radius':.5,'capture_speed':.16,'turn_gain':2.2,'anti_shield_trigger':1.65,'anti_shield_radius':1.12,'anti_shield_step':.72,'anti_shield_dwell':.35,'shield_obstacles':[2.,2.]}
    strategy=CatcherStrategy(cfg)
    strategy.command(Pose2D(-1.5,0.,0.,0.),Pose2D(1.5,1.,0.,0.),Velocity2D(.1,.2),'predictive')
    assert strategy.mode!='FLANK'  # dwell not met yet: a single tick shouldn't cost a detour
    strategy.command(Pose2D(-1.49,.01,0.,.4),Pose2D(1.51,.99,0.,.4),Velocity2D(.1,-.2),'predictive')
    direction=strategy.flank_direction
    assert strategy.mode=='FLANK'
    strategy.command(Pose2D(-1.48,.02,0.,.42),Pose2D(1.52,.98,0.,.42),Velocity2D(.1,-.2),'predictive')
    assert strategy.mode=='FLANK' and strategy.flank_direction==direction
    assert abs(math.hypot(strategy.target.x-2.,strategy.target.y-2.)-1.12)<1e-8

def test_flank_ignores_a_brief_pass_through_obstacle_proximity():
    """Regression test: single-tick proximity to cover must not trigger a detour.

    The old trigger fired on raw distance-to-obstacle alone, which made the
    predictive Catcher measurably slower than the naive baseline chase against
    strategic/adversarial Runners that merely pass near an obstacle while
    evading (benchmark-confirmed: 20.2s vs 17.5s mean capture time).
    """
    cfg={'max_linear':.46,'prediction_horizon':4.,'prediction_step':.2,'capture_control_distance':.55,'capture_radius':.5,'capture_speed':.16,'turn_gain':2.2,'anti_shield_trigger':1.65,'anti_shield_radius':1.12,'anti_shield_step':.72,'anti_shield_dwell':.35,'shield_obstacles':[2.,2.]}
    strategy=CatcherStrategy(cfg)
    strategy.command(Pose2D(-1.5,0.,0.,0.),Pose2D(1.5,1.,0.,0.),Velocity2D(.6,.6),'predictive')
    assert strategy.mode!='FLANK'

def test_flank_commitment_expires_and_cools_down():
    cfg={'max_linear':.46,'prediction_horizon':4.,'prediction_step':.2,'capture_control_distance':.55,'capture_radius':.5,'capture_speed':.16,'turn_gain':2.2,'anti_shield_trigger':1.65,'anti_shield_radius':1.12,'anti_shield_step':.72,'anti_shield_dwell':.1,'anti_shield_max_duration':.5,'anti_shield_cooldown':1.0,'shield_obstacles':[2.,2.]}
    strategy=CatcherStrategy(cfg)
    for stamp in (0.,.15,.3,.45,.6,.75):
        strategy.command(Pose2D(-1.5,0.,0.,stamp),Pose2D(1.5,1.,0.,stamp),Velocity2D(.1,.2),'predictive')
    assert strategy.mode!='FLANK'  # gave up after anti_shield_max_duration
    strategy.command(Pose2D(-1.5,0.,0.,.9),Pose2D(1.5,1.,0.,.9),Velocity2D(.1,.2),'predictive')
    assert strategy.mode!='FLANK'  # still inside the cooldown window

def test_bidirectional_escape_reverses_immediately_when_facing_catcher():
    command=drive_to_bidirectional(Pose2D(1.5,0.,math.pi),Pose2D(3.,0.),.46,2.2)
    assert command.linear<-.45 and abs(command.angular)<1e-8

def test_boundary_recovery_overrides_outward_tactics():
    assert boundary_recovery(Pose2D(0.,0.)) is None
    command=boundary_recovery(Pose2D(0.,-4.6,-math.pi/2),max_speed=.34)
    assert command is not None and command.linear<-.33 and abs(command.angular)<1e-8

def test_adaptive_navigator_selects_and_retains_an_open_gap():
    cfg={'max_linear':.46,'turn_gain':2.2,'lidar_stop_distance':.65,'lidar_influence_distance':1.35,'navigator_heading_samples':48}
    navigator=AdaptiveNavigator(cfg); ranges=[float('inf')]*360
    for index in range(174,187):ranges[index]=.72
    first,mode=navigator.command(Command(.4,0.),Pose2D(stamp=0.),ranges,-math.pi,2*math.pi/360,target_bearing=0.,now=0.)
    side=math.copysign(1.,first.angular)
    second,second_mode=navigator.command(Command(.4,0.),Pose2D(stamp=.1),ranges,-math.pi,2*math.pi/360,target_bearing=0.,now=.1)
    assert mode=='GAP' and second_mode=='GAP' and abs(first.angular)>.1
    assert math.copysign(1.,second.angular)==side

def test_adaptive_navigator_recovers_from_a_detected_stall():
    cfg={'max_linear':.46,'turn_gain':2.2,'lidar_stop_distance':.65,'lidar_influence_distance':1.35,'navigator_stall_window':1.0,'navigator_stall_distance':.06}
    navigator=AdaptiveNavigator(cfg); ranges=[float('inf')]*360
    for index in range(170,191):ranges[index]=.55
    for stamp in (0.,.45,.9):
        command,mode=navigator.command(Command(.4,0.),Pose2D(stamp=stamp),ranges,-math.pi,2*math.pi/360,target_bearing=0.,now=stamp)
    assert mode.startswith('RECOVERY_') and abs(command.linear)>=.1

def test_adaptive_navigator_recovery_has_a_reentry_cooldown():
    cfg={'max_linear':.70,'turn_gain':2.2,'lidar_stop_distance':.65,'lidar_influence_distance':1.35,'navigator_stall_window':1.,'navigator_stall_distance':.06,'navigator_recovery_time':.4,'navigator_recovery_cooldown':1.5}
    navigator=AdaptiveNavigator(cfg); ranges=[float('inf')]*360
    for index in range(170,191):ranges[index]=.7
    for stamp in (0.,.45,.9):
        _,mode=navigator.command(Command(.5,0.),Pose2D(stamp=stamp),ranges,-math.pi,2*math.pi/360,target_bearing=0.,now=stamp)
    assert mode.startswith('RECOVERY_')
    _,mode=navigator.command(Command(.5,0.),Pose2D(stamp=1.4),ranges,-math.pi,2*math.pi/360,target_bearing=0.,now=1.4)
    assert not mode.startswith('RECOVERY_')

def test_competitive_runner_prioritizes_radial_escape_under_threat():
    cfg={'max_linear':.46,'arena_half':5.,'boundary_margin':.55,'lookahead':1.25,'turn_gain':2.2,'distance_weight':1.,'clearance_weight':1.7,'open_weight':.6,'smooth_weight':.35,'emergency_escape_distance':1.15,'adversarial_break_weight':.9,'adversarial_interval':.8,'survival_radial_weight':3.5,'safe_feint_distance':2.6,'shield_radius':1.05,'shield_obstacles':[-2.,2.,2.,2.,-2.,-2.,2.,-2.]}
    strategy=RunnerStrategy(cfg,seed=1); catcher=Pose2D(-1.,0.); runner=Pose2D(1.,0.,math.pi,1.)
    command=strategy.command(catcher,runner,'competitive')
    assert strategy.mode=='BREAKAWAY' and command.linear<0

def test_shield_target_stays_outside_obstacle_and_opposes_catcher():
    cfg={'shield_radius':1.05,'shield_obstacles':[2.,2.]}; strategy=RunnerStrategy(cfg); catcher=Pose2D(-2.,2.); runner=Pose2D(2.,.9)
    target=strategy.shield_target(catcher,runner)
    assert abs(math.hypot(target.x-2.,target.y-2.)-1.05)<1e-8 and target.x>2.

def test_runner_rejects_wall_trap_when_open_shield_exists():
    cfg={'shield_radius':1.05,'shield_reach_weight':1.6,'shield_open_weight':3.,'arena_half':5.,'boundary_margin':.55}
    strategy=RunnerStrategy(cfg); strategy.set_obstacles([(3.4,0.),(.8,2.1)])
    strategy.shield_target(Pose2D(-1.5,0.),Pose2D(1.5,0.))
    assert strategy.shield_obstacle == (.8,2.1)

def test_shield_and_flank_targets_never_leave_arena():
    runner=CatcherStrategy({'anti_shield_trigger':1.65,'anti_shield_radius':1.12,'anti_shield_step':.72,'arena_half':5.,'boundary_margin':.55,'anti_shield_dwell':.35})
    runner.set_obstacles([(4.3,4.3)])
    runner.shield_flank_target(Pose2D(3.5,4.2,0.,0.),Pose2D(4.2,4.2,0.,0.),Velocity2D())
    target=runner.shield_flank_target(Pose2D(3.5,4.2,0.,.4),Pose2D(4.2,4.2,0.,.4),Velocity2D())
    assert target is not None and abs(target.x)<=4.45 and abs(target.y)<=4.45
    evader=RunnerStrategy({'shield_radius':1.05,'arena_half':5.,'boundary_margin':.55})
    evader.set_obstacles([(-4.3,-4.3)])
    target=evader.shield_target(Pose2D(0.,0.),Pose2D(-4.,-4.))
    assert abs(target.x)<=4.45 and abs(target.y)<=4.45

def test_competitive_runner_keeps_shield_during_symmetric_pose_noise():
    cfg={'shield_radius':1.05,'shield_obstacles':[2.,2.,2.,-2.]}
    strategy=RunnerStrategy(cfg)
    strategy.shield_target(Pose2D(-1.5,0.),Pose2D(1.5,.01))
    chosen=strategy.shield_obstacle
    strategy.shield_target(Pose2D(-1.5,0.),Pose2D(1.5,-.01))
    assert strategy.shield_obstacle == chosen

def test_lidar_mapper_discovers_shifted_obstacles_without_a_known_map():
    mapper=ObstacleMapper(arena_half=5.,resolution=.12,min_cluster_cells=3)
    obstacles=((1.3,-.8,.45),(-1.7,1.2,.5))
    for pose in (Pose2D(0.,0.,0.),Pose2D(0.,2.,0.),Pose2D(2.,0.,math.pi)):
        count=720; angle_min=-math.pi; increment=2*math.pi/count; ranges=[]
        for index in range(count):
            angle=pose.yaw+angle_min+index*increment; dx=math.cos(angle); dy=math.sin(angle); hits=[]
            for ox,oy,radius in obstacles:
                px=ox-pose.x; py=oy-pose.y; projection=px*dx+py*dy
                discriminant=projection**2-(px**2+py**2-radius**2)
                if projection>0 and discriminant>=0:hits.append(projection-math.sqrt(discriminant))
            ranges.append(min(hits) if hits else float('inf'))
        centers=mapper.update(pose,ranges,angle_min,increment)
    assert all(any(math.hypot(cx-ox,cy-oy)<.3 for cx,cy in centers) for ox,oy,_ in obstacles)

def test_both_roles_use_dynamically_detected_obstacle_centers():
    detected=[(1.1,-.6),(-1.4,1.3)]
    runner_cfg={'max_linear':.34,'arena_half':5.,'boundary_margin':.55,'lookahead':1.25,'turn_gain':2.2,'distance_weight':1.,'clearance_weight':1.7,'open_weight':.6,'smooth_weight':.35,'shield_radius':1.05,'shield_obstacles':[]}
    runner_strategy=RunnerStrategy(runner_cfg); runner_strategy.set_obstacles(detected)
    assert runner_strategy.shield_target(Pose2D(-1.,0.),Pose2D(.8,-.5)) is not None
    catcher_cfg={'max_linear':.46,'prediction_horizon':4.,'prediction_step':.2,'capture_control_distance':.55,'capture_radius':.5,'capture_speed':.16,'turn_gain':2.2,'anti_shield_trigger':1.65,'anti_shield_radius':1.12,'anti_shield_step':.72,'shield_obstacles':[]}
    catcher_strategy=CatcherStrategy(catcher_cfg); catcher_strategy.set_obstacles(detected)
    catcher_strategy.command(Pose2D(-1.,0.,0.,0.),Pose2D(.8,-.5,0.,0.),Velocity2D(.1,.1),'predictive')
    catcher_strategy.command(Pose2D(-1.,0.,0.,.4),Pose2D(.8,-.5,0.,.4),Velocity2D(.1,.1),'predictive')
    assert catcher_strategy.mode=='FLANK'

def test_lidar_mapper_expires_removed_obstacles_and_tracks_changes():
    count=361; angle_min=-math.pi; increment=2*math.pi/(count-1); pose=Pose2D()
    mapper=ObstacleMapper(resolution=.12,min_cluster_cells=3,cell_ttl=.5)
    old=[float('inf')]*count
    for index in range(177,184):old[index]=2.0
    first=mapper.update(pose,old,angle_min,increment,stamp=0.)
    assert any(math.hypot(x-2.,y)<.35 for x,y in first)
    moved=[float('inf')]*count
    for index in range(267,274):moved[index]=1.5
    second=mapper.update(pose,moved,angle_min,increment,stamp=1.)
    assert not any(math.hypot(x-2.,y)<.5 for x,y in second)
    assert any(math.hypot(x,y-1.5)<.35 for x,y in second)

def test_lidar_mapper_ray_clears_removed_obstacle_before_ttl():
    count=361; angle_min=-math.pi; increment=2*math.pi/(count-1); pose=Pose2D()
    mapper=ObstacleMapper(resolution=.12,min_cluster_cells=3,cell_ttl=30.)
    occupied=[float('inf')]*count
    for index in range(177,184):occupied[index]=2.0
    assert mapper.update(pose,occupied,angle_min,increment,stamp=0.)
    empty=[float('inf')]*count
    assert mapper.update(pose,empty,angle_min,increment,stamp=.1)==[]
