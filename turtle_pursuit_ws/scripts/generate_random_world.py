#!/usr/bin/env python3
import argparse
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path


def sample_layout(rng, count, extent, minimum_spacing, start_clearance):
    starts=((-1.5,0.0),(1.5,0.0)); points=[]
    for _ in range(count):
        for _ in range(2000):
            point=(rng.uniform(-extent,extent),rng.uniform(-extent,extent))
            if all(math.hypot(point[0]-x,point[1]-y)>=start_clearance for x,y in starts) and all(math.hypot(point[0]-x,point[1]-y)>=minimum_spacing for x,y,_ in points):
                points.append((point[0],point[1],rng.uniform(-math.pi,math.pi))); break
        else:raise RuntimeError('unable to generate a collision-free obstacle layout')
    return points


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--template',required=True); parser.add_argument('--output',required=True)
    parser.add_argument('--seed',required=True,type=int); parser.add_argument('--world-name',default='pursuit_random')
    parser.add_argument('--extent',type=float,default=3.45); parser.add_argument('--minimum-spacing',type=float,default=2.15)
    parser.add_argument('--start-clearance',type=float,default=1.45)
    args=parser.parse_args(); tree=ET.parse(args.template); root=tree.getroot(); world=root.find('world'); world.set('name',args.world_name)
    model=next(model for model in world.findall('model') if model.get('name')=='obstacles'); link=model.find('link')
    collisions={element.get('name'):element for element in link.findall('collision')}; visuals={element.get('name')[:-1]:element for element in link.findall('visual')}
    layout=sample_layout(random.Random(args.seed),len(collisions),args.extent,args.minimum_spacing,args.start_clearance)
    for name,(x,y,yaw) in zip(sorted(collisions),layout):
        pose=f'{x:.4f} {y:.4f} 0.5 0 0 {yaw:.4f}'
        collisions[name].find('pose').text=pose; visuals[name].find('pose').text=pose
    output=Path(args.output); output.parent.mkdir(parents=True,exist_ok=True); tree.write(output,encoding='unicode',xml_declaration=True)
    print(f'ARENA_SEED={args.seed}')
    for name,(x,y,yaw) in zip(sorted(collisions),layout):print(f'  {name}: x={x:.3f} y={y:.3f} yaw={yaw:.3f}')


if __name__=='__main__':main()
