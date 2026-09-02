import math, random
from turtle_pursuit.common.geometry import Pose2D, normalize_angle
from turtle_pursuit.control.motion import drive_to, drive_to_bidirectional, dynamic_speed

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
        self.shield_obstacle=None; self.detected_obstacles=[]
        # Fixed organizer-style Round 1 course, clear of the four arena obstacles.
        self.standard_course=[Pose2D(3.6,0.0),Pose2D(3.6,3.6),Pose2D(0.0,3.6),Pose2D(-3.6,3.6),Pose2D(-3.6,0.0),Pose2D(-3.6,-3.6),Pose2D(0.0,-3.6),Pose2D(3.6,-3.6)]
    def set_obstacles(self,obstacles):
        self.detected_obstacles=list(obstacles)
    def speed_for_separation(self,separation):
        """Use full boost under direct threat, then taper smoothly to cruise."""
        full=self.cfg.get('runner_full_boost_distance',3.2)
        cruise=max(full+.05,self.cfg.get('runner_boost_distance',5.0))
        demand=(cruise-separation)/(cruise-full)
        return dynamic_speed(self.cfg.get('cruise_linear',.44),self.cfg['max_linear'],demand)
    def shield_target(self,c,r):
        raw=self.cfg.get('shield_obstacles') or ()
        obstacles=self.detected_obstacles or [(raw[i],raw[i+1]) for i in range(0,len(raw)-1,2)]
        if not obstacles:return None
        # Prefer reachable cover that is not already controlled by the Catcher.
        # Keep that choice until it is genuinely lost: symmetric obstacles otherwise
        # make a noisy pose estimate flip the target and waste the Runner's speed.
        radius=self.cfg.get('shield_radius',1.05)
        arena_limit=self.cfg.get('arena_half',5.)-self.cfg.get('boundary_margin',.55)
        def score(o):
            orbit_clearance=min(arena_limit-abs(o[0]),arena_limit-abs(o[1]))-radius
            return (math.hypot(o[0]-c.x,o[1]-c.y)
                    -self.cfg.get('shield_reach_weight',1.6)*math.hypot(o[0]-r.x,o[1]-r.y)
                    +self.cfg.get('shield_open_weight',3.0)*max(-.5,orbit_clearance))
        best=max(obstacles,key=score)
        if self.shield_obstacle is not None:
            nearest=min(obstacles,key=lambda o:math.hypot(o[0]-self.shield_obstacle[0],o[1]-self.shield_obstacle[1]))
            association=self.cfg.get('shield_association_distance',.75)
            self.shield_obstacle=nearest if math.hypot(nearest[0]-self.shield_obstacle[0],nearest[1]-self.shield_obstacle[1])<association else None
        if self.shield_obstacle is None:
            self.shield_obstacle=best
        else:
            current=self.shield_obstacle
            current_lost=(math.hypot(current[0]-c.x,current[1]-c.y)+self.cfg.get('shield_lost_margin',.35) <
                          math.hypot(current[0]-r.x,current[1]-r.y))
            if current_lost and score(best)>score(current)+self.cfg.get('shield_switch_hysteresis',.75):
                self.shield_obstacle=best
        ox,oy=self.shield_obstacle
        desired=math.atan2(oy-c.y,ox-c.x)
        runner_angle=math.atan2(r.y-oy,r.x-ox)
        runner_radius=math.hypot(r.x-ox,r.y-oy)
        if runner_radius>radius*self.cfg.get('shield_join_ratio',1.45):
            # Join the protective orbit on the near tangent instead of crossing the obstacle.
            delta=normalize_angle(desired-runner_angle)
            limit=self.cfg.get('shield_join_step',.65); angle=runner_angle+max(-limit,min(limit,delta))
        else:
            delta=normalize_angle(desired-runner_angle)
            limit=self.cfg.get('shield_orbit_step',.48); angle=runner_angle+max(-limit,min(limit,delta))
        margin=self.cfg.get('boundary_margin',.55); limit=self.cfg.get('arena_half',5.)-margin
        return Pose2D(max(-limit,min(limit,ox+radius*math.cos(angle))),
                      max(-limit,min(limit,oy+radius*math.sin(angle))))
    def command(self,c,r,strategy='strategic'):
        away=math.atan2(r.y-c.y,r.x-c.x)
        if strategy=='stationary': self.mode='STATIONARY'; return drive_to(r,r,0.0)
        if strategy=='standardized':
            self.target=self.standard_course[self.waypoint]
            if math.hypot(self.target.x-r.x,self.target.y-r.y)<.35:
                self.waypoint=(self.waypoint+1)%len(self.standard_course); self.target=self.standard_course[self.waypoint]
            self.mode='STANDARDIZED'; return drive_to(r,self.target,min(self.cfg.get('cruise_linear',.44),self.cfg['max_linear']),self.cfg['turn_gain'])
        separation=math.hypot(r.x-c.x,r.y-c.y)
        speed=self.speed_for_separation(separation)
        if strategy=='competitive' and separation>=self.cfg.get('shield_commit_distance',3.2):
            self.target=self.shield_target(c,r)
            if self.target is not None:
                self.mode='SHIELD'
                return drive_to_bidirectional(r,self.target,speed,self.cfg['turn_gain'])
        advanced=strategy in ('strategic','adversarial','competitive')
        emergency=advanced and separation<self.cfg.get('emergency_escape_distance',1.15)
        fine=strategy in ('adversarial','competitive')
        headings=[away] if strategy=='baseline' else ([away-math.pi/3,away+math.pi/3,away-math.pi/4,away+math.pi/4,away] if emergency else [away+i*math.pi/(12 if fine else 6) for i in range(-11 if fine else -5,12 if fine else 6)])
        interval=self.cfg.get('adversarial_interval',.8)
        phase=int(r.stamp/max(.1,interval))
        break_heading=away+(math.pi/2 if phase%2 else -math.pi/2)
        candidates=[]
        for h in headings:
            jitter=self.rng.uniform(-.035,.035) if advanced else 0.0
            x=max(-self.cfg['arena_half']+self.cfg['boundary_margin'],min(self.cfg['arena_half']-self.cfg['boundary_margin'],r.x+self.cfg['lookahead']*math.cos(h+jitter)))
            y=max(-self.cfg['arena_half']+self.cfg['boundary_margin'],min(self.cfg['arena_half']-self.cfg['boundary_margin'],r.y+self.cfg['lookahead']*math.sin(h+jitter)))
            t=Pose2D(x,y); score=candidate_score(c,r,t,self.heading,self.cfg)
            if strategy=='adversarial': score+=self.cfg.get('adversarial_break_weight',.9)*(1. if separation<2.2 else .3)*math.cos(normalize_angle(h-break_heading))
            if strategy=='competitive':
                threat=max(0.,self.cfg.get('safe_feint_distance',2.6)-separation)
                score+=self.cfg.get('survival_radial_weight',3.5)*(1.+threat)*math.cos(normalize_angle(h-away))
                if separation>=self.cfg.get('safe_feint_distance',2.6): score+=.45*math.cos(normalize_angle(h-break_heading))
            candidates.append((score,h,t))
        _,self.heading,self.target=max(candidates,key=lambda x:x[0]); self.mode='BREAKAWAY' if (emergency or (strategy=='competitive' and separation<self.cfg.get('shield_commit_distance',3.2))) else ('COMPETITIVE' if strategy=='competitive' else ('ADVERSARIAL' if strategy=='adversarial' else ('STRATEGIC' if strategy=='strategic' else 'BASELINE'))); return drive_to_bidirectional(r,self.target,speed,self.cfg['turn_gain'])
