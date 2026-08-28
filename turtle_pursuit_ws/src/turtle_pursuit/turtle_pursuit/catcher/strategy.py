import math
from turtle_pursuit.common.geometry import Command, distance
from turtle_pursuit.control.motion import drive_to
from turtle_pursuit.planning.interception import interception_point

class CatcherStrategy:
    def __init__(self,cfg): self.cfg=cfg; self.mode='SEARCH'; self.target=None
    def command(self,c,r,rv,strategy='predictive'):
        d=distance(c,r)
        if d<=self.cfg['capture_control_distance']:
            self.mode='CAPTURE'; target=r
            runner_speed=math.hypot(rv.vx,rv.vy)
            hold_correction=max(0.0,d-self.cfg['capture_radius']*.75)*.8
            speed=min(self.cfg['max_linear'],max(self.cfg['capture_speed'],runner_speed+hold_correction))
        elif d<=self.cfg['chase_distance'] or strategy=='baseline':
            self.mode='CHASE'; target=r; speed=self.cfg['max_linear']
        else:
            self.mode='INTERCEPT'; target=interception_point(c,r,rv,self.cfg['max_linear'],self.cfg['prediction_horizon'],self.cfg['prediction_step']); speed=self.cfg['max_linear']
        self.target=target; return drive_to(c,target,speed,self.cfg['turn_gain'])
