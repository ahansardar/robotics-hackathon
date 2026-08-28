import math, tempfile
from pathlib import Path
from turtle_pursuit.common.geometry import Pose2D, Velocity2D, normalize_angle, quaternion_to_yaw
from turtle_pursuit.tracking.velocity import VelocityEstimator
from turtle_pursuit.planning.interception import interception_point
from turtle_pursuit.planning.grid import GridPlanner
from turtle_pursuit.runner.strategy import RunnerStrategy, candidate_score
from turtle_pursuit.evaluation.match import CaptureDetector
from turtle_pursuit.common.config import load_config
from turtle_pursuit.control.motion import MotionLimiter
from turtle_pursuit.common.geometry import Command

def test_angles_and_quaternion():
    assert abs(abs(normalize_angle(3*math.pi))-math.pi)<1e-8
    assert abs(quaternion_to_yaw(0,0,math.sin(.5),math.cos(.5))-1)<1e-8
def test_velocity_estimator():
    e=VelocityEstimator(1.0); e.update(Pose2D(0,0,0,1)); v=e.update(Pose2D(2,0,0,2)); assert v.vx==2
def test_interception():
    p=interception_point(Pose2D(0,0),Pose2D(1,0),Velocity2D(0.1,0),1,3,.1); assert 1.0<=p.x<1.3
def test_grid_planner_avoids_wall():
    path=GridPlanner(5,5,{(2,0),(2,1),(2,2),(2,3)}).plan((0,0),(4,0)); assert path and (2,4) in path and not set(path)&{(2,0),(2,1),(2,2),(2,3)}
def test_capture_hold_continuous():
    d=CaptureDetector(.5,1); assert not d.update(.4,0); assert not d.update(.4,.9); assert not d.update(.6,1); assert not d.update(.4,2); assert d.update(.4,3.01)
def test_candidate_prefers_clear_space():
    cfg={'arena_half':5.,'distance_weight':1.,'clearance_weight':2.,'open_weight':1.,'smooth_weight':0.}
    c=Pose2D(0,0); r=Pose2D(3,0); assert candidate_score(c,r,Pose2D(3,2),0,cfg)>candidate_score(c,r,Pose2D(4.9,0),0,cfg)
def test_stale_safety_limiter_rejects_nan(): assert MotionLimiter().apply(Command(float('nan'),1),.1).linear==0
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
