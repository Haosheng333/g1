from pathlib import Path
import sys, numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'scripts'))
from collision_mask import EmptyCollisionMask, BooleanGridCollisionMask, CombinedCollisionMask

def test_empty_mask_never_blocks():
    assert not EmptyCollisionMask()((1,2,3,4))

def test_boolean_and_combined_masks(tmp_path):
    data=np.zeros((2,2,2,2),dtype=bool); data[1,0,1,0]=True; path=tmp_path/'mask.npy'; np.save(path,data); mask=BooleanGridCollisionMask(path,data.shape); assert mask((1,0,1,0)); assert not mask((0,0,0,0)); combined=CombinedCollisionMask(EmptyCollisionMask(),mask); assert combined((1,0,1,0))
