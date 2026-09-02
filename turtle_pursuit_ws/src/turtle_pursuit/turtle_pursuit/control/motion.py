import math
from collections import deque
from turtle_pursuit.common.geometry import Command, Pose2D, finite_command, normalize_angle

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

def drive_to_bidirectional(pose, target, max_speed=.45, turn_gain=2.2):
    """Drive toward a target immediately, reversing when it is behind the robot."""
    heading=math.atan2(target.y-pose.y,target.x-pose.x)
    error=normalize_angle(heading-pose.yaw)
    if abs(error)<=math.pi/2:
        return Command(max_speed*max(0.0,math.cos(error))**2,turn_gain*error)
    reverse_error=normalize_angle(error-math.copysign(math.pi,error))
    return Command(-max_speed*max(0.0,math.cos(reverse_error))**2,turn_gain*reverse_error)

def dynamic_speed(cruise_speed, boost_speed, demand):
    """Blend continuously between cruise and boost from a normalized demand."""
    demand=max(0.0,min(1.0,float(demand)))
    demand=demand*demand*(3.0-2.0*demand)  # smoothstep: no speed discontinuities
    return cruise_speed+(boost_speed-cruise_speed)*demand

def boundary_recovery(pose,arena_half=5.,margin=.55,max_speed=.34,turn_gain=2.2):
    """Return an inward command after crossing the arena's inner safety line."""
    limit=arena_half-margin
    if abs(pose.x)<=limit and abs(pose.y)<=limit:return None
    inset=max(.2,margin)
    target=Pose2D(max(-limit+inset,min(limit-inset,pose.x)),
                  max(-limit+inset,min(limit-inset,pose.y)))
    return drive_to_bidirectional(pose,target,max_speed,turn_gain)

class AdaptiveNavigator:
    """Persistent, scan-driven gap selection with automatic stall recovery."""
    def __init__(self,cfg):
        self.cfg=cfg; self.heading=0.; self.last_yaw=None; self.preferred_side=1.
        self.history=deque(); self.recovery_until=-1.

    def _clearance(self,heading,points):
        radius=self.cfg.get('navigator_robot_radius',.32)
        nearest=self.cfg.get('navigator_clearance_cap',3.0)
        cosine=math.cos(heading); sine=math.sin(heading)
        for x,y in points:
            forward=x*cosine+y*sine; lateral=abs(-x*sine+y*cosine)
            if forward>0. and lateral<radius:
                nearest=min(nearest,forward-math.sqrt(max(0.,radius*radius-lateral*lateral)))
        return max(0.,nearest)

    def _stalled(self,pose,now,requested_speed,nearby):
        self.history.append((now,pose.x,pose.y,requested_speed))
        window=self.cfg.get('navigator_stall_window',1.25)
        while self.history and now-self.history[0][0]>window:self.history.popleft()
        if len(self.history)<2 or now-self.history[0][0]<window*.8:return False
        _,x,y,_=self.history[0]
        displacement=math.hypot(pose.x-x,pose.y-y)
        return requested_speed>.12 and nearby and displacement<self.cfg.get('navigator_stall_distance',.055)

    def command(self,desired,pose,ranges,angle_min,angle_increment,
                target_bearing=None,exclude_bearing=None,exclude_range=None,now=None):
        if not ranges:return desired,'DIRECT'
        now=float(pose.stamp if now is None else now)
        points=[]
        excluded_surface=max(.05,exclude_range-.25) if exclude_range is not None else None
        for index,value in enumerate(ranges):
            if not math.isfinite(value) or value<=0.:continue
            angle=normalize_angle(angle_min+index*angle_increment)
            if exclude_bearing is not None and excluded_surface is not None and abs(normalize_angle(angle-exclude_bearing))<.20 and abs(value-excluded_surface)<.30:
                continue
            points.append((value*math.cos(angle),value*math.sin(angle)))
        if not points:return desired,'DIRECT'

        desired_heading=normalize_angle(target_bearing if target_bearing is not None else desired.angular/max(.1,self.cfg.get('turn_gain',2.2)))
        direct_clearance=self._clearance(desired_heading,points)
        influence=self.cfg.get('lidar_influence_distance',1.35)
        if self.last_yaw is not None:self.heading=normalize_angle(self.heading-normalize_angle(pose.yaw-self.last_yaw))
        self.last_yaw=pose.yaw

        if self._stalled(pose,now,abs(desired.linear),direct_clearance<influence):
            self.preferred_side*=-1.; self.recovery_until=now+self.cfg.get('navigator_recovery_time',.9); self.history.clear()
        if now<self.recovery_until:
            rear=self._clearance(math.pi,points)
            if rear>self.cfg.get('lidar_stop_distance',.65):return Command(-.16,1.35*self.preferred_side),'RECOVERY'
            return Command(.12,-1.5*self.preferred_side),'RECOVERY'

        if direct_clearance>=influence:
            self.heading=desired_heading
            return desired,'DIRECT'

        samples=max(12,int(self.cfg.get('navigator_heading_samples',48)))
        candidates=[]
        for index in range(samples):
            heading=-math.pi+2.*math.pi*index/samples
            clearance=self._clearance(heading,points)
            alignment=abs(normalize_angle(heading-desired_heading))
            continuity=abs(normalize_angle(heading-self.heading))
            reverse_penalty=.35 if abs(heading)>math.pi/2 else 0.
            side_bonus=.12 if heading*self.preferred_side>0 else 0.
            score=(self.cfg.get('navigator_clearance_weight',1.35)*min(clearance,2.5)
                   -self.cfg.get('navigator_goal_weight',2.1)*alignment
                   -self.cfg.get('navigator_continuity_weight',.75)*continuity
                   -reverse_penalty+side_bonus)
            candidates.append((score,clearance,heading))
        _,clearance,chosen=max(candidates)
        if chosen!=0.:self.preferred_side=math.copysign(1.,chosen)
        self.heading=chosen
        stop=self.cfg.get('lidar_stop_distance',.65)
        scale=max(.12,min(1.,(clearance-stop)/max(.05,influence-stop)))
        max_speed=self.cfg.get('max_linear',.34); forward=abs(chosen)<=math.pi/2
        steering=chosen if forward else normalize_angle(chosen-math.copysign(math.pi,chosen))
        linear=max_speed*scale*max(.15,math.cos(steering)**2)*(1. if forward else -.7)
        return Command(linear,self.cfg.get('turn_gain',2.2)*steering),'GAP'

def avoid_scan(cmd, ranges, angle_min, angle_increment, stop=.42, influence=1.0, target_bearing=None, target_range=None):
    """Apply local lidar safety while allowing the Catcher to contact its target."""
    if not ranges: return cmd
    left=right=float('inf'); left_open=right_open=0.0
    for i,r in enumerate(ranges):
        if not math.isfinite(r) or r<=0: continue
        a=normalize_angle(angle_min+i*angle_increment)
        target_surface=max(.05,target_range-.25) if target_range is not None else None
        if target_bearing is not None and abs(normalize_angle(a-target_bearing))<.20 and abs(r-target_surface)<.30:
            continue
        travel_angle=normalize_angle(a-(math.pi if cmd.linear<0 else 0.))
        if abs(travel_angle)<1.2:
            if travel_angle>=0: left=min(left,r); left_open+=min(r,influence)
            else: right=min(right,r); right_open+=min(r,influence)
    nearest=min(left,right)
    turn_direction=1.0 if left_open>=right_open else -1.0
    if nearest<stop*.65: return Command(-math.copysign(.10,cmd.linear or 1.),1.6*turn_direction)
    if nearest<stop: return Command(0.0,1.5*turn_direction)
    if nearest<influence:
        turn=turn_direction*(influence-nearest)*1.8
        return Command(cmd.linear*max(0.15,(nearest-stop)/(influence-stop)),cmd.angular+turn)
    return cmd
