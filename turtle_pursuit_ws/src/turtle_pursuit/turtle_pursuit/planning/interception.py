import math
from turtle_pursuit.common.geometry import Pose2D

def project_pose(runner, velocity, t):
    """Project a target with constant turn rate and world-frame velocity."""
    omega = velocity.wz
    if abs(omega) < 1e-3:
        return Pose2D(runner.x+velocity.vx*t, runner.y+velocity.vy*t)
    angle = omega*t
    sine = math.sin(angle)
    one_minus_cosine = 1.0-math.cos(angle)
    return Pose2D(
        runner.x+(sine*velocity.vx-one_minus_cosine*velocity.vy)/omega,
        runner.y+(one_minus_cosine*velocity.vx+sine*velocity.vy)/omega,
    )

def interception_point(catcher, runner, velocity, speed, horizon=4.0, step=0.2):
    """Earliest constant-speed feasible CTRV intercept, sampled predictably.

    The returned Pose2D carries `.feasible`: True when the search actually
    found a time-consistent point where the Catcher's path and the projected
    Runner arc coincide within the horizon; False when it exhausted the
    horizon without converging, in which case the returned pose is just the
    projected horizon endpoint -- an extrapolation guess, not a real
    intercept. Callers should be more cautious trusting an infeasible result
    (self-arbitration, not blind trust in a fixed-length forecast).
    """
    if speed <= 0:
        p = Pose2D(runner.x, runner.y); p.feasible = False; return p
    t = 0.0
    while t <= horizon + 1e-9:
        p = project_pose(runner,velocity,t)
        if math.hypot(p.x-catcher.x, p.y-catcher.y)/speed <= t + 0.05:
            p.feasible = True; return p
        t += step
    p = project_pose(runner,velocity,horizon)
    p.feasible = False
    return p
