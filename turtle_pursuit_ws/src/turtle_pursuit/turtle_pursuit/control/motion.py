import math
from turtle_pursuit.common.geometry import Command, finite_command, normalize_angle

class MotionLimiter:
    def __init__(self, max_linear=.45, max_angular=1.8, linear_accel=.6, angular_accel=2.5):
        self.ml=max_linear; self.ma=max_angular; self.la=linear_accel; self.aa=angular_accel; self.last=Command()
    def apply(self, cmd, dt):
        if not finite_command(cmd): cmd=Command()
        l=max(-self.ml,min(self.ml,cmd.linear)); a=max(-self.ma,min(self.ma,cmd.angular))
        l=max(self.last.linear-self.la*dt,min(self.last.linear+self.la*dt,l))
        a=max(self.last.angular-self.aa*dt,min(self.last.angular+self.aa*dt,a))
        self.last=Command(l,a); return self.last

def drive_to(pose, target, max_speed=.45, turn_gain=2.2):
    heading=math.atan2(target.y-pose.y,target.x-pose.x)
    error=normalize_angle(heading-pose.yaw)
    return Command(max_speed*max(0.0,math.cos(error))**2, turn_gain*error)

def avoid_scan(cmd, ranges, angle_min, angle_increment, stop=.42, influence=1.0):
    if not ranges: return cmd
    left=right=float('inf')
    for i,r in enumerate(ranges):
        if not math.isfinite(r) or r<=0: continue
        a=normalize_angle(angle_min+i*angle_increment)
        if abs(a)<1.2:
            if a>=0: left=min(left,r)
            else: right=min(right,r)
    nearest=min(left,right)
    if nearest<stop: return Command(0.0, -1.4 if left<right else 1.4)
    if nearest<influence:
        turn=(-1 if left<right else 1)*(influence-nearest)*1.8
        return Command(cmd.linear*max(0.15,(nearest-stop)/(influence-stop)),cmd.angular+turn)
    return cmd

