from pathlib import Path
import sys,numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from joint_space_astar import JointGrid,states_to_five_joint_path

def test_single_state_preserves_exact_start_and_goal():
    grid=JointGrid([-1]*5,[1]*5,4.0); start=np.array([0,0,0,0,0.]); goal=np.array([.01,.01,.01,.01,.01]); state=grid.joints_to_state(start)
    path=states_to_five_joint_path(grid,[state],start,goal)
    assert np.allclose(path[0],start); assert np.allclose(path[-1],goal); assert len(path)>=2
