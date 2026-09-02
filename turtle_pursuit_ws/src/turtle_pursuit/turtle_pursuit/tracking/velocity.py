from collections import deque
import math
from turtle_pursuit.common.geometry import Velocity2D
from turtle_pursuit.common.geometry import normalize_angle

class VelocityEstimator:
    """Estimate world velocity and turn rate with a bounded EMA, plus a
    turn-consistency signal the Catcher uses to discount its own forecast.

    Turn rate matters for pursuit: treating a circling target as if it will
    keep moving along its tangent creates a stable orbit just outside capture
    range. But a constant-turn-rate forecast is only trustworthy when the
    turn rate is actually persisting in one direction; a Runner reacting to
    the Catcher (juking, flipping evasion heading tick to tick) produces a
    turn-rate sign that flips back and forth, and extrapolating *that* several
    seconds ahead sends the predicted point further from the truth than just
    aiming near the Runner's current position would have. `turn_consistency`
    tracks how often consecutive raw turn-rate samples agree in sign over a
    short window, so callers can shrink their forecast horizon when it's low
    instead of trusting a fixed-length prediction unconditionally (this is
    the measured fix for `predictive` losing time to `baseline` against
    strategic/adversarial evasion: see catcher/strategy.py).
    """
    def __init__(self, alpha=0.35, window=8, consistency_window=6):
        self.alpha = alpha; self.samples = deque(maxlen=window); self.value = Velocity2D()
        self._wz_history = deque(maxlen=max(2, consistency_window)); self.turn_consistency = 1.0
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
        self._wz_history.append(wz)
        self.turn_consistency = self._consistency()
        self.value.consistency = self.turn_consistency
        return self.value
    def _consistency(self):
        history = list(self._wz_history)
        if len(history) < 2: return 1.0
        agreements = sum(1 for a, b in zip(history, history[1:]) if a*b >= 0)
        return agreements/(len(history)-1)
