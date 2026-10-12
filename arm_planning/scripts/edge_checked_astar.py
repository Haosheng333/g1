#!/usr/bin/env python3
"""A* that validates each 4-degree edge at dense resolution before expansion."""
from __future__ import annotations
import heapq, itertools, math
import numpy as np
from joint_space_astar import weighted_manhattan, reconstruct


def dense_edge_is_blocked(grid, first, second, wrist_rad, probe, max_step_rad=math.radians(1.0)):
    q0=np.r_[grid.state_to_joints(first),wrist_rad]
    q1=np.r_[grid.state_to_joints(second),wrist_rad]
    steps=max(1,int(math.ceil(np.max(np.abs(q1-q0))/max_step_rad)))
    for i in range(1,steps+1):
        q=q0+(q1-q0)*(i/steps)
        if any(item['blocking'] for item in probe.report(q)):
            return True
    return False


def astar_edge_checked(grid,start,goal,is_blocked,probe,wrist_rad,movable_axes=(0,1,2,3),move_weights=(1.,1.,1.,1.),max_expansions=1_000_000):
    if is_blocked(start): return None,0,'start state is blocked'
    if is_blocked(goal): return None,0,'goal state is blocked'
    if start==goal: return [start],0,'success'
    counter=itertools.count(); heap=[]; g={start:0.}; came={}; closed=set(); expanded=0; edge_cache={}
    h=weighted_manhattan(start,goal,move_weights); heapq.heappush(heap,(h,h,next(counter),start))
    while heap:
        _,_,_,current=heapq.heappop(heap)
        if current in closed: continue
        if current==goal: return reconstruct(came,current),expanded,'success'
        closed.add(current); expanded+=1
        if expanded>max_expansions: return None,expanded,'maximum A* expansions exceeded'
        for neighbor,axis in grid.neighbors(current,movable_axes):
            if neighbor in closed or is_blocked(neighbor): continue
            edge_key=(current,neighbor) if current<neighbor else (neighbor,current)
            blocked=edge_cache.get(edge_key)
            if blocked is None:
                blocked=dense_edge_is_blocked(grid,current,neighbor,wrist_rad,probe)
                edge_cache[edge_key]=blocked
            if blocked: continue
            tentative=g[current]+float(move_weights[axis])
            if tentative<g.get(neighbor,math.inf):
                came[neighbor]=current; g[neighbor]=tentative; h=weighted_manhattan(neighbor,goal,move_weights)
                heapq.heappush(heap,(tentative+h,h,next(counter),neighbor))
    return None,expanded,'no path found'
