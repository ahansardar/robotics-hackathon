import math
from turtle_pursuit.common.geometry import Pose2D

def interception_point(catcher, runner, velocity, speed, horizon=4.0, step=0.2):
    """Earliest constant-speed feasible intercept, sampled for predictable runtime."""
    if speed <= 0: return Pose2D(runner.x, runner.y)
    t = 0.0
    while t <= horizon + 1e-9:
        p = Pose2D(runner.x+velocity.vx*t, runner.y+velocity.vy*t)
        if math.hypot(p.x-catcher.x, p.y-catcher.y)/speed <= t + 0.05: return p
        t += step
    return Pose2D(runner.x+velocity.vx*horizon, runner.y+velocity.vy*horizon)

