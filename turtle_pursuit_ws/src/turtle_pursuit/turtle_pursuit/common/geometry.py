from dataclasses import dataclass
import math

@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    stamp: float = 0.0

@dataclass
class Velocity2D:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    consistency: float = 1.0

@dataclass
class Command:
    linear: float = 0.0
    angular: float = 0.0

def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return normalize_angle(math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z)))

def distance(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x-b.x, a.y-b.y)

def finite_command(c: Command) -> bool:
    return math.isfinite(c.linear) and math.isfinite(c.angular)

