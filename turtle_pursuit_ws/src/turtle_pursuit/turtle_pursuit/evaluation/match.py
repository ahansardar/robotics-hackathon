class CaptureDetector:
    def __init__(self, radius=.5, hold=1.0): self.radius=radius; self.hold=hold; self.entered=None
    def update(self, separation, now):
        if separation > self.radius: self.entered=None; return False
        if self.entered is None: self.entered=now
        return now-self.entered >= self.hold

