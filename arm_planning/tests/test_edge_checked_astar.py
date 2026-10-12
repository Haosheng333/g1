from pathlib import Path
import sys, numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from edge_checked_astar import dense_edge_is_blocked
class Grid:
    def state_to_joints(self,s): return np.asarray(s,dtype=float)
class Probe:
    def report(self,q): return [{'blocking':True}] if 0.4<q[0]<0.6 else []
def test_edge_checks_intermediate_points():
    assert dense_edge_is_blocked(Grid(),(0,0,0,0),(1,0,0,0),0.0,Probe(),max_step_rad=.1)
