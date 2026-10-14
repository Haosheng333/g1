from pathlib import Path
import sys,numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from module2_grasp_planner import Module2GraspPlanner
class Result:
    success=True; message='success'; joint_path=[[0]*5,[.1]*5]; verified_goal_joints=[.1]*5; position_error_m=.001; rx_error_rad=.01; rz_error_rad=.02
class Stub:
    calls=0
    def plan_to_pose(self,arm,current,pose): self.calls+=1; return Result()
def test_cartesian_segment_routes_through_plan_to_pose():
    p=Module2GraspPlanner.__new__(Module2GraspPlanner); p.planner=Stub(); segment,error=p._cartesian_segment('left',np.zeros(5),np.zeros(5),np.array([.01,0,0,0,0]),'approach'); assert error is None; assert p.planner.calls==1; assert segment.joint_path[-1]==[.1]*5
