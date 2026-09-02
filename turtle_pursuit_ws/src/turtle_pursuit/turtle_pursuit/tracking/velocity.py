from collections import deque
import math
from turtle_pursuit.common.geometry import Velocity2D
from turtle_pursuit.common.geometry import normalize_angle

class VelocityEstimator:
    """Estimate world velocity and turn rate with a bounded EMA.

    Turn rate matters for pursuit: treating a circling target as if it will keep
    moving along its tangent creates a stable orbit just outside capture range.
    """
    def __init__(self, alpha=0.35, window=8):
        self.alpha = alpha; self.samples = deque(maxlen=window); self.value = Velocity2D()
    def update(self, pose):
        self.samples.append(pose)
        if len(self.samples) < 2: return self.value
        a, b = self.samples[-2], self.samples[-1]; dt = b.stamp-a.stamp
        if dt <= 1e-4: return self.value
        vx, vy = (b.x-a.x)/dt, (b.y-a.y)/dt
        wz = normalize_angle(b.yaw-a.yaw)/dt
        self.value.vx += self.alpha*(vx-self.value.vx)
        self.value.vy += self.alpha*(vy-self.value.vy)
        self.value.wz += self.alpha*(wz-self.value.wz)
        return self.value
