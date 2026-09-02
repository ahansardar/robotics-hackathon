import argparse, csv, math, random
from pathlib import Path
from turtle_pursuit.catcher.strategy import CatcherStrategy
from turtle_pursuit.runner.strategy import RunnerStrategy
from turtle_pursuit.common.geometry import Pose2D, Velocity2D, distance, normalize_angle
from turtle_pursuit.tracking.velocity import VelocityEstimator
from turtle_pursuit.evaluation.match import CaptureDetector

C_CFG={'max_linear':.70,'cruise_linear':.44,'catcher_cruise_distance':1.,'catcher_boost_distance':3.5,'prediction_horizon':4.,'prediction_step':.2,'chase_distance':1.6,'capture_control_distance':.55,'capture_radius':.5,'capture_speed':.16,'turn_gain':2.2}
R_CFG={'max_linear':.70,'cruise_linear':.44,'runner_full_boost_distance':3.2,'runner_boost_distance':5.,'shield_commit_distance':3.2,'emergency_escape_distance':1.15,'arena_half':5.,'boundary_margin':.55,'lookahead':1.25,'turn_gain':2.2,'distance_weight':1.,'clearance_weight':1.7,'open_weight':.6,'smooth_weight':.35}

def step_robot(p,cmd,dt): p.yaw=normalize_angle(p.yaw+max(-1.8,min(1.8,cmd.angular))*dt); p.x+=cmd.linear*math.cos(p.yaw)*dt; p.y+=cmd.linear*math.sin(p.yaw)*dt; return p
# Two TurtleBot 4 Lite / Create 3 bases (~0.34 m outer diameter) would make
# physical contact around this separation -- well inside the 0.5 m capture
# radius, so a clean capture does not require it. This benchmark has no
# obstacle geometry, so it can only ever measure Catcher/Runner body contact,
# never a collision with an obstacle; that is still rulebook-dependent on the
# official arena and can only be checked in the full Gazebo trial.
BODY_CONTACT_DISTANCE=.34
def run(catcher_strategy,behavior,seed,duration=60.,collision_distance=BODY_CONTACT_DISTANCE):
    random.seed(seed); c=Pose2D(-1.5,0,0,0); r=Pose2D(1.5,0,math.pi,0); cs=CatcherStrategy(C_CFG); rs=RunnerStrategy(R_CFG,seed); est=VelocityEstimator(.35); det=CaptureDetector(.5,1.); dt=.05; t=0.; cp=rp=0.; minimum=99.; collisions=0; touching=False
    while t<duration:
        oldc=Pose2D(c.x,c.y); oldr=Pose2D(r.x,r.y); vel=est.update(Pose2D(r.x,r.y,r.yaw,t)); cc=cs.command(c,r,vel,catcher_strategy)
        if behavior=='straight': rc=rs.command(Pose2D(-20,r.y),r,'baseline')
        elif behavior=='circular':
            from turtle_pursuit.common.geometry import Command
            rc=Command(.28,.42)
        elif behavior=='random_walk':
            from turtle_pursuit.common.geometry import Command
            rc=Command(.26,random.uniform(-1.2,1.2) if int(t*2)!=int((t-dt)*2) else .2)
        elif behavior=='stationary': rc=rs.command(c,r,'stationary')
        else: rc=rs.command(c,r,'competitive' if behavior=='competitive' else ('adversarial' if behavior=='adversarial' else ('strategic' if behavior in ('strategic','wall_aware','obstacle_weaving') else 'baseline')))
        step_robot(c,cc,dt); step_robot(r,rc,dt); c.stamp=r.stamp=t; cp+=distance(c,oldc); rp+=distance(r,oldr); sep=distance(c,r); minimum=min(minimum,sep); t+=dt
        contact=sep<=collision_distance
        if contact and not touching: collisions+=1
        touching=contact
        if det.update(sep,t): break
    return {'catcher_strategy':catcher_strategy,'runner_behavior':behavior,'seed':seed,'capture_success':t<duration,'capture_time':round(t,3) if t<duration else '','survival_time':round(t,3),'minimum_separation':round(minimum,3),'collisions':collisions,'catcher_path_length':round(cp,3),'runner_path_length':round(rp,3)}
def main(args=None):
    ap=argparse.ArgumentParser(); ap.add_argument('--seeds',type=int,default=3); ap.add_argument('--duration',type=float,default=60.); ap.add_argument('--output',default='benchmark_results.csv'); ap.add_argument('--require-all-captures',action='store_true'); ns=ap.parse_args(args); rows=[]
    for cs in ('baseline','predictive'):
        for b in ('stationary','straight','random_walk','circular','wall_aware','obstacle_weaving','strategic','adversarial'):
            for seed in range(ns.seeds): rows.append(run(cs,b,seed,ns.duration))
    out=Path(ns.output); out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('w',newline='',encoding='utf-8') as f: w=csv.DictWriter(f,fieldnames=rows[0],lineterminator='\n'); w.writeheader(); w.writerows(rows)
    for cs in ('baseline','predictive'):
        subset=[r for r in rows if r['catcher_strategy']==cs]; wins=sum(r['capture_success'] for r in subset); times=[float(r['capture_time']) for r in subset if r['capture_time']!='']; print(f'{cs}: {wins}/{len(subset)} captures, mean capture {sum(times)/len(times):.2f}s' if times else f'{cs}: 0 captures')
    print(out.resolve())
    failures=[r for r in rows if not r['capture_success']]
    if ns.require_all_captures and failures:
        failed_cases=sorted({(r['catcher_strategy'],r['runner_behavior']) for r in failures})
        raise SystemExit(f'benchmark gate failed: {len(failures)} missed captures in {failed_cases}')
