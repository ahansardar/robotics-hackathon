import math
from turtle_pursuit.common.geometry import Command, Pose2D, distance, normalize_angle
from turtle_pursuit.control.motion import drive_to, dynamic_speed
from turtle_pursuit.planning.interception import interception_point

class CatcherStrategy:
    def __init__(self,cfg):
        self.cfg=cfg; self.mode='SEARCH'; self.target=None
        self.flank_obstacle=None; self.flank_direction=0.0; self.detected_obstacles=[]
        self.flank_dwell_start=None; self.flank_committed_since=None; self.flank_cooldown_until=None
        self.intercept_trust=1.0; self.trusting_intercept=True
    def set_obstacles(self,obstacles):
        self.detected_obstacles=list(obstacles)
    def speed_for_distance(self,distance_to_runner):
        """Boost across open distance, cruise near the Runner for control."""
        near=self.cfg.get('catcher_cruise_distance',1.0)
        far=max(near+.05,self.cfg.get('catcher_boost_distance',3.5))
        demand=(distance_to_runner-near)/(far-near)
        return dynamic_speed(self.cfg.get('cruise_linear',.44),self.cfg['max_linear'],demand)
    def _reset_flank(self):
        self.flank_obstacle=None; self.flank_direction=0.0; self.flank_dwell_start=None; self.flank_committed_since=None
    def shield_flank_target(self,c,r,rv):
        """Return a flank target only for a Runner genuinely lingering near cover.

        Triggering on raw single-tick proximity made the Catcher detour around
        any obstacle the Runner merely passed near while evading, which is what
        made `predictive` measurably slower than the naive `baseline` chase
        against strategic/adversarial Runners (benchmark-confirmed regression).
        A short dwell requirement filters out pass-throughs, and a hard cap on
        how long a single commitment may run stops a Runner from baiting an
        indefinite orbit around one piece of cover.
        """
        now=r.stamp
        if self.flank_cooldown_until is not None and now<self.flank_cooldown_until:
            return None
        raw=self.cfg.get('shield_obstacles') or ()
        obstacles=self.detected_obstacles or [(raw[i],raw[i+1]) for i in range(0,len(raw)-1,2)]
        if not obstacles:
            self._reset_flank(); return None
        obstacle=min(obstacles,key=lambda o:math.hypot(r.x-o[0],r.y-o[1]))
        trigger=self.cfg.get('anti_shield_trigger',1.65)
        if math.hypot(r.x-obstacle[0],r.y-obstacle[1])>trigger:
            self._reset_flank(); return None
        same_obstacle=(self.flank_obstacle is not None and
                       math.hypot(obstacle[0]-self.flank_obstacle[0],obstacle[1]-self.flank_obstacle[1])<self.cfg.get('flank_association_distance',.75))
        if not same_obstacle:
            self.flank_obstacle=obstacle; self.flank_dwell_start=now; self.flank_committed_since=None
            ox,oy=obstacle
            catcher_angle=math.atan2(c.y-oy,c.x-ox)
            runner_angle=math.atan2(r.y-oy,r.x-ox)
            angular_velocity=((r.x-ox)*rv.vy-(r.y-oy)*rv.vx)/max(.1,(r.x-ox)**2+(r.y-oy)**2)
            delta=normalize_angle(runner_angle-catcher_angle)
            threshold=self.cfg.get('flank_angular_velocity_threshold',.08)
            self.flank_direction=math.copysign(1.0,angular_velocity if abs(angular_velocity)>threshold else (delta or 1.0))
        else:
            self.flank_obstacle=obstacle
        if self.flank_dwell_start is None:
            self.flank_dwell_start=now
        dwell=self.cfg.get('anti_shield_dwell',.35)
        if self.flank_committed_since is None and now-self.flank_dwell_start<dwell:
            return None
        if self.flank_committed_since is None:
            self.flank_committed_since=now
        if now-self.flank_committed_since>self.cfg.get('anti_shield_max_duration',3.0):
            self.flank_cooldown_until=now+self.cfg.get('anti_shield_cooldown',1.5)
            self._reset_flank(); return None
        ox,oy=obstacle
        catcher_angle=math.atan2(c.y-oy,c.x-ox)
        radius=self.cfg.get('anti_shield_radius',1.12)
        step=self.cfg.get('anti_shield_step',.72)
        angle=catcher_angle+self.flank_direction*step
        margin=self.cfg.get('boundary_margin',.55); limit=self.cfg.get('arena_half',5.)-margin
        return Pose2D(max(-limit,min(limit,ox+radius*math.cos(angle))),
                      max(-limit,min(limit,oy+radius*math.sin(angle))))
    def command(self,c,r,rv,strategy='predictive'):
        d=distance(c,r)
        adaptive_speed=self.speed_for_distance(d)
        if d<=self.cfg['capture_control_distance']:
            self.mode='CAPTURE'
            runner_speed=math.hypot(rv.vx,rv.vy)
            target=interception_point(c,r,rv,self.cfg['max_linear'],.6,.1)
            hold_correction=max(0.0,d-self.cfg['capture_radius']*.75)*.8
            speed=min(self.cfg['max_linear'],max(self.cfg['capture_speed'],runner_speed+hold_correction))
        elif strategy=='baseline':
            self.mode='CHASE'; target=r; speed=adaptive_speed
        else:
            target=self.shield_flank_target(c,r,rv)
            if target is not None:
                self.mode='FLANK'; speed=max(adaptive_speed,self.cfg.get('cruise_linear',.44))
            else:
                aggressive=strategy=='aggressive'
                # A constant-turn-rate forecast is only trustworthy while the
                # turn rate is actually persisting in one direction. Discount
                # the horizon by how consistent it has recently been so a
                # Runner reacting tick-to-tick (juking) gets a short, cautious
                # forecast instead of a full curved extrapolation flung far
                # past where it will actually be -- measured to cost several
                # extra seconds against strategic/adversarial evasion.
                floor=self.cfg.get('prediction_confidence_floor',.25)
                confidence=floor+(1.-floor)*getattr(rv,'consistency',1.0)
                horizon=self.cfg['prediction_horizon']*(1.25 if aggressive else 1.)*confidence
                step=self.cfg['prediction_step']*(.5 if aggressive else 1.)
                intercept=interception_point(c,r,rv,adaptive_speed,horizon,step)
                # Self-arbitration: only steer at the forecast point if the
                # search actually found a time-consistent convergence within
                # the horizon. An infeasible result is just the horizon
                # endpoint -- an extrapolation guess that can be worse than
                # simply closing on where the Runner already is, which is
                # exactly what let the plain baseline chase win several
                # benchmark scenarios the forecast-trusting Catcher lost.
                # Debounce the trust switch itself (a Schmitt trigger on a
                # smoothed feasibility signal): flipping the aim point between
                # the forecast and the raw current position every single tick
                # against an oscillating Runner (e.g. `adversarial`) made the
                # Catcher's own path chatter and cost time, exactly the same
                # class of bug the FLANK dwell-gate fixed above.
                self.intercept_trust += .4*((1.0 if intercept.feasible else 0.0)-self.intercept_trust)
                if self.intercept_trust>=self.cfg.get('intercept_trust_high',.6): self.trusting_intercept=True
                elif self.intercept_trust<=self.cfg.get('intercept_trust_low',.3): self.trusting_intercept=False
                if self.trusting_intercept:
                    target=intercept; self.mode='PRESSURE' if aggressive else 'INTERCEPT'
                else:
                    target=r; self.mode='CHASE'
                speed=adaptive_speed
        self.target=target; return drive_to(c,target,speed,self.cfg['turn_gain'])
