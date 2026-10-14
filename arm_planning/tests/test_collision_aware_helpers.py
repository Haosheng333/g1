from pathlib import Path
import sys, numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from collision_aware_planner import nearby_states

def test_nearby_states_starts_at_goal_then_one_cell():
    states=nearby_states((2,2,2,2),(5,5,5,5),1)
    assert states[0]==(2,2,2,2)
    assert sum(abs(np.array(states[1])-2))==1
