# perception

Module 1 of the Grasp-and-Deliver pipeline: detects a target object from
RGB-D and reports its position in the robot base frame. Standard ROS1
(catkin) package.

## Layout

```
perception/
├── package.xml, CMakeLists.txt, setup.py   # catkin package files
├── srv/GetObjectPosition.srv               # the service contract
├── src/perception/m1_perception.py         # detection + geometry algorithm, no ROS dependency
├── scripts/perception_node.py              # rospy service wrapper (the ROS node)
├── scripts/demo_m1.py                      # run the algorithm on any photo, no ROS needed
├── launch/perception.launch                # brings up the RealSense driver + this node
├── models/bottle_finetuned.pt              # fine-tuned detector (see "Detector" below)
├── test/test_m1_perception.py              # unit tests
├── test/mujoco_validation.py               # validation against simulated RGB-D (one worked example)
└── test/mujoco_accuracy_eval.py            # same, batched over a grid of positions (accuracy numbers)
```

## Interface (for other modules)

One service: **`/perception/get_object_position`** (`GetObjectPosition.srv`)

```
string  label      # e.g. "bottle"; empty = any class
int32   index      # which match to return (0-based, after sorting by base-frame z, highest first)
---
bool    success
float64 x
float64 y
float64 z
string  frame_id   # always "base_link"
string  message    # human-readable failure reason
```

```python
resp = rospy.ServiceProxy("/perception/get_object_position", GetObjectPosition)("bottle", 0)
if resp.success:
    x, y, z = resp.x, resp.y, resp.z   # meters, in base_link
else:
    print(resp.message)
```

```bash
rosservice call /perception/get_object_position "{label: 'bottle', index: 0}"
```

This node does not retry -- one call = one fresh camera frame = one
detection pass. Retry logic is the caller's responsibility.

Subscribes to (from the RealSense driver, `align_depth:=true`):
`/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`
(16UC1, mm), `/camera/color/camera_info`.

## Build & run

```bash
cd ~/catkin_ws/src
pip install -r perception/requirements.txt
cd ~/catkin_ws
catkin build perception
source devel/setup.bash
roslaunch perception perception.launch
```

## Testing

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

PYTHONPATH=src python3 test/test_m1_perception.py
PYTHONPATH=src python3 scripts/demo_m1.py --image test/data/sample_remote.png --label remote --depth 1.4
PYTHONPATH=src python3 test/mujoco_validation.py       # needs `pip install mujoco`
PYTHONPATH=src python3 test/mujoco_accuracy_eval.py    # same, batched -- accuracy numbers below
```

**Accuracy** (`mujoco_accuracy_eval.py`, simulated RGB-D with known ground
truth, current camera mount, spec's own bottle-radius approximation): across
a grid of table positions 0.7-1.0 m in front of the robot and ±0.15 m
side-to-side (12 trials, all fully in frame) -- **mean error 1.46 cm, std
0.28 cm, range 1.07-1.90 cm, 12/12 under the spec's 2 cm/1 m bar**. Beyond
~1.15 m forward the object starts touching the top edge of the frame with
this camera's fixed mount (error jumps to 1.1-3.9 cm, biased by the clipped
silhouette) -- treat ~1.15 m as this setup's practical forward range, not a
hard limit of the algorithm.

## Detector

`models/bottle_finetuned.pt` is fine-tuned from `yolov8n.pt` on frames
auto-extracted from `data/bottle_capture.mp4` (real video from the robot's
camera): the actual target is a standard drink can, not a bottle, but kept
under the label `"bottle"` for interface/spec consistency (see
`CLASS_RADIUS_M`'s comment in `m1_perception.py`). Validation mAP50 0.959,
mAP50-95 0.916, a clear improvement over the stock COCO detector (missed or
low-confidence detections on the same footage, since a can isn't a real COCO
"bottle").

`perception_node.py` and `demo_m1.py` load this checkpoint automatically
when present, falling back to stock `yolov8n.pt` if it's missing.

To retrain on new footage:

```bash
python3 tools/video_to_dataset.py path/to/video.mp4 --out data/video_dataset --n-frames 200 --labels bottle
# review data/video_dataset/preview/, correct data/video_dataset/labels/ if needed
python3 tools/build_bottle_dataset.py     # -> data/bottle_dataset (train/val split, class remapped to 0)
python3 tools/train_bottle.py             # -> runs/bottle_detect/weights/best.pt
cp runs/bottle_detect/weights/best.pt perception/models/bottle_finetuned.pt
```

## Known limitations

- Not tested with ROS or a real RealSense (no ROS support on this dev
  machine) -- `perception_node.py` only checked for syntax errors.
- Camera extrinsics (`Extrinsics()` in `m1_perception.py`) are a real value
  from a different G1+D435i mount, not this team's own -- measure and
  replace `R_base_opt`/`t_base_opt` before trusting position output.
- `CLASS_RADIUS_M`'s `"bottle"` entry is measured (a real can, see
  "Detector"); `"cup"`/`"remote"`/the default are still unverified
  placeholders for the stock detector.
- If the target object changes again, retrain per "Detector" above.
