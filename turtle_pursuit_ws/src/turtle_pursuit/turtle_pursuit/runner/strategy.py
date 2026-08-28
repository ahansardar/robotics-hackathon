import math, random
from turtle_pursuit.common.geometry import Pose2D, normalize_angle
from turtle_pursuit.control.motion import drive_to

def candidate_score(catcher, runner, target, previous_heading, cfg):
    dc=math.hypot(target.x-catcher.x,target.y-catcher.y)
    clearance=min(cfg['arena_half']-abs(target.x),cfg['arena_half']-abs(target.y))
    corner=min(cfg['arena_half']-abs(target.x),cfg['arena_half']-abs(target.y))
    heading=math.atan2(target.y-runner.y,target.x-runner.x)
    smooth=abs(normalize_angle(heading-previous_heading))
    return cfg['distance_weight']*dc+cfg['clearance_weight']*clearance+cfg['open_weight']*corner-cfg['smooth_weight']*smooth

class RunnerStrategy:
    def __init__(self,cfg,seed=1):
        self.cfg=cfg; self.rng=random.Random(seed); self.heading=0.0; self.target=None; self.mode='EVADE'; self.waypoint=0
        # Fixed organizer-style Round 1 course, clear of the four arena obstacles.
        self.standard_course=[Pose2D(3.6,0.0),Pose2D(3.6,3.6),Pose2D(0.0,3.6),Pose2D(-3.6,3.6),Pose2D(-3.6,0.0),Pose2D(-3.6,-3.6),Pose2D(0.0,-3.6),Pose2D(3.6,-3.6)]
    def command(self,c,r,strategy='strategic'):
        away=math.atan2(r.y-c.y,r.x-c.x)
        if strategy=='stationary': self.mode='STATIONARY'; return drive_to(r,r,0.0)
        if strategy=='standardized':
            self.target=self.standard_course[self.waypoint]
            if math.hypot(self.target.x-r.x,self.target.y-r.y)<.35:
                self.waypoint=(self.waypoint+1)%len(self.standard_course); self.target=self.standard_course[self.waypoint]
            self.mode='STANDARDIZED'; return drive_to(r,self.target,min(.30,self.cfg['max_linear']),self.cfg['turn_gain'])
        headings=[away] if strategy=='baseline' else [away+i*math.pi/6 for i in range(-5,6)]
        candidates=[]
        for h in headings:
            jitter=self.rng.uniform(-.035,.035) if strategy=='strategic' else 0.0
            x=max(-self.cfg['arena_half']+self.cfg['boundary_margin'],min(self.cfg['arena_half']-self.cfg['boundary_margin'],r.x+self.cfg['lookahead']*math.cos(h+jitter)))
            y=max(-self.cfg['arena_half']+self.cfg['boundary_margin'],min(self.cfg['arena_half']-self.cfg['boundary_margin'],r.y+self.cfg['lookahead']*math.sin(h+jitter)))
            t=Pose2D(x,y); candidates.append((candidate_score(c,r,t,self.heading,self.cfg),h,t))
        _,self.heading,self.target=max(candidates,key=lambda x:x[0]); self.mode='STRATEGIC' if strategy=='strategic' else 'BASELINE'; return drive_to(r,self.target,self.cfg['max_linear'],self.cfg['turn_gain'])
