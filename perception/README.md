# perception

Module 1 of the Grasp-and-Deliver pipeline: detects a target object from
RGB-D and reports its position in `torso_link` (x forward, y left, z up),
the frame the arm planners take targets in. Standard ROS1
(catkin) package.

## Layout

```
perception/
├── package.xml, CMakeLists.txt, setup.py   # catkin package files
├── srv/GetObjectPosition.srv               # the service contract
├── src/perception/m1_perception.py         # detection + geometry algorithm, no ROS dependency
├── src/perception/ros_image.py             # sensor_msgs/Image -> numpy (replaces cv_bridge)
├── scripts/perception_node.py              # rospy service wrapper (the ROS node)
├── scripts/perception_viewer.py            # live "what the robot sees" view, /perception/annotated_image
├── scripts/start_remote_camera.sh          # start the RealSense driver on PC2 over SSH
├── scripts/stop_perception.sh              # kill everything perception.launch started
├── src/perception/overlay.py               # draws boxes / depth / torso_link xyz (viewer + demo)
├── scripts/demo_m1.py                      # run the algorithm on any photo, no ROS needed
├── launch/perception.launch                # brings up the RealSense driver + this node
├── config/camera_mount.yaml                # torso_link -> camera_link mount the node loads by default
├── config/d455_extrinsics_*.yaml           # ChArUco candidate(s) from ~/Workspace/d455_calib_ws
├── models/bottle_finetuned.pt              # fine-tuned detector (see "Detector" below)
├── test/test_m1_perception.py              # unit tests (geometry + end-to-end on a photo)
├── test/test_node_helpers.py               # image decoding, calibration loader
├── test/test_lazy_import.py                # `import perception` must not pull in torch
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
string  frame_id   # always "torso_link"
string  message    # human-readable failure reason
```

```python
resp = rospy.ServiceProxy("/perception/get_object_position", GetObjectPosition)("bottle", 0)
if resp.success:
    x, y, z = resp.x, resp.y, resp.z   # meters, in torso_link
else:
    print(resp.message)
```

```bash
rosservice call /perception/get_object_position "{label: 'bottle', index: 0}"
```

The point is the object's **geometric centre**. For `bottle` (the 14 cm
can, an upright cylinder) that is exact by construction: the radius is pushed
horizontally and z is the table height read from the depth just below the
box (fallback: solved from the box's bottom row) plus half the can's height (`CLASS_HEIGHT_M` in `m1_perception.py`, so
update it if the can changes). Other classes use the spec's plain push along
the view ray, which from a camera looking down is biased by a few cm.

A constant correction is applied to every reported point via the launch
args `x_offset` / `y_offset` / `z_offset` (m, torso_link; param
`~offset_xyz`). The offset is the measured error `reported - real`, and the
node subtracts it: `real = reported - offset`. Launch defaults, set by the
operator on 2026-10-21 from the grasp tests: x -0.005, y +0.005, z -0.035
(so x is reported 5 mm further, y 5 mm less, z 35 mm higher than the raw
estimate). `x_offset:=0 y_offset:=0 z_offset:=0` disables it. The viewer applies the same offset, so the window
always shows what the service returns. Prefer fixing the calibration file
for errors that grow with distance; the offset is for a constant residual.

Depth is the median over the central 30% x 50% of the box (a single
7x7 patch on the shiny can flickered by 2 cm between frames), and one call runs
`frames` (default 3) consecutive synced frames and returns the per-axis median
of the successful ones, so a call takes ~0.2 s at 15 fps. `frames:=1` restores
one frame per call. This node does not retry a failed call; that is the caller's responsibility. `index` must be
>= 0. Any internal error comes back as `success=false` with the exception in
`message`, never as a ROS service exception.

Subscribes to (from the RealSense driver, `align_depth:=true`):
`/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`
(16UC1, mm), `/camera/color/camera_info`. RGB and depth are paired by an
`ApproximateTimeSynchronizer` (100 ms slop); a call waits up to 1 s for the
first pair newer than the latest one it had when the call arrived. Freshness
is judged by frame stamps relative to each other, never against this
machine's clock: with the camera on PC2 the stamps follow PC2's clock, which
was 7 minutes off on 2026-10-21. The node logs a warning when stamps are more
than 0.5 s from local time, and keeps working. Images are decoded with numpy
(`ros_image.py`), not cv_bridge, because Noetic's cv_bridge is built against
numpy 1 and crashes under the container's numpy 2.

The message is byte-identical to g1_intellect's `g1_camera/GetObjectPosition`
(same md5), so `arm_grasp`'s `grasp_bottle` and g1_intellect's `main_process`
can call this node unchanged.

### Camera extrinsics

Default: `config/d455_extrinsics_20261017_fitted_20261021.yaml`: the ChArUco
calibration of the **D455** done on this robot on 2026-10-17 with
`~/Workspace/d455_calib_ws` (`torso_link -> camera_color_optical_frame`, used
directly), refitted on 2026-10-21 to three facts on the standing robot: the
desk in view must be level in `torso_link` (pitch +1.9°), the desk is at
-0.050 m by tape (camera 0.7 cm lower), and a can's centre is at x = 0.390 m
by tape (camera 1.0 cm further forward). With it the desk is level to 0.04°
and reads -0.051 to -0.052 m from 0.3 to 0.6 m, and the can centre comes out
at (0.392, 0.028, 0.022). The raw ChArUco file read the same can 2 cm high
and 3 cm far. **Caveat:** the robot's lean relative to the desk differed by
about 2° between two sessions, which is 1 to 1.5 cm at 0.45 m; a fixed
calibration cannot follow the stance, so re-run the levelling check below when
accuracy matters, and treat the orchestrator's `perception_extrinsics` blocker
as cleared only for the standing pose it was fitted in.

| Set | Camera | torso_link -> camera | pitch down | Select with |
|---|---|---|---|---|
| `config/d455_extrinsics_20261017_fitted_20261021.yaml` (**default**) | D455, fitted | (0.118, -0.015, 0.406) m to the optical frame | 67.7° | `calibration_file:=<file>` |
| `config/d455_extrinsics_20261017T074044_928552Z.yaml` | D455, raw ChArUco | same | 65.8° | `calibration_file:=<file>` |
| `config/camera_mount.yaml` | old D435i (g1_intellect `g1_camera/config/params.yaml`) | (0.058, 0.028, 0.445) m to camera_link | 47.6° | `use_calibration_file:=false` |

The D435i set differs by about 30 cm at 1 m and must not be used with the
D455.

**Levelling check.** Back-project the depth image into `torso_link` with the
candidate extrinsics and fit a plane to the desk: it must be level (the robot
standing, waist at zero). A tilt about y is a pitch error; the desk height at
the can must match a tape measure. `rosrun perception desk_level_check.py` does this against the running camera
with the node's calibration (or `--calibration-file F`). A new calibration from the ChArUco tool is dropped in by pointing
`calibration_file:=` at it (the loader checks `parent_frame: torso_link` and a
proper rotation). arm_grasp's TF stub in `arm_node_abs.launch` mirrors the
D455 file's `parent_to_camera_link` block; update it when the file changes.
The camera is rigidly mounted on `torso_link`, so points are correct at every
waist angle. A constant offset in detected positions is a translation error; a
distance-dependent one is a rotation error.

## Seeing what the robot sees

`perception.launch` also starts `perception_viewer.py` (`viewer:=true`,
default) and opens its output in an `image_view` window on this host
(`viewer_window:=true`, default; needs X11, which `g1.sh start` allows).
The window shows the camera image with every detection drawn on it, and the
depth image with the same boxes on the right:

- green box: the label being looked for (`viewer_target_label`, default
  `bottle`), with confidence, sampled depth and the object centre in
  `torso_link`
- yellow box: another class the detector knows
- red box: a detection whose depth sample was missing or out of range, i.e.
  one the service would NOT return
- header line: how many targets were found, how many boxes had depth, and the
  frame age (large = the camera stream is stale)
- the xyz shown is the median of the last 5 frames of that object
  (`~smooth_frames`), like the service's own multi-frame median; the raw
  per-frame value moves by about 1 cm on a shiny can because of depth noise

```bash
roslaunch perception perception.launch                               # service + viewer + window
roslaunch perception perception.launch viewer_target_label:=cup model:=yolov8n.pt
roslaunch perception perception.launch viewer_window:=false          # publish only, no window
rosrun image_view image_view image:=/perception/annotated_image      # open the window later
rosrun rqt_image_view rqt_image_view /perception/annotated_image     # or this; or an RViz Image display
```

The viewer runs its own detector pass at `viewer_rate` Hz (default 5) and
never touches the service node, so it cannot delay
`/perception/get_object_position`. Without the camera, the bench stand-in
`test/fake_camera_publisher.py --image photo.jpg` feeds it a photo.

## Build & run

The package is part of `g1_ws` (symlink `g1_ws/src/perception -> ../../perception`)
and builds with the rest of DIP inside the g1_intellect container; see the
repo-level `CLAUDE.md` for the `catkin_make` line. ultralytics, torch and
numpy are already in the container's conda env.

```bash
source ~/Workspace/DIP/g1_ws/devel/setup.bash --extend
roslaunch perception perception.launch                    # camera driver on PC2
roslaunch perception perception.launch local_camera:=true # camera driver on this host
rosservice call /perception/get_object_position "{label: 'bottle', index: 0}"
```

Stop it with Ctrl+C in the launch shell. If nodes are left behind (ROS's
Python nodes are named `python3`, so `killall roslaunch` does NOT find them):

```bash
rosrun perception stop_perception.sh          # perception nodes, window, local driver only
rosrun perception stop_perception.sh --all    # also roscore and the PC2 camera session
```

It never touches other packages' launches, and without `--all` (or
`--with-master` / `--with-remote-camera`) it leaves `roscore` and the PC2
camera alone, since other terminals usually own them.

On the robot the RealSense is wired to PC2. With `roscore` already running
here in a shell where the robot cable was up (so `ROS_IP` points at the
robot interface), start the driver on PC2 over SSH with:

```bash
rosrun perception start_remote_camera.sh        # Ctrl+C stops it
```

It publishes the same topics as `local_camera:=true` (1280x720 colour,
aligned depth, 15 fps). PC2 details come from `G1_PC2_IP` / `G1_PC2_USER` /
`G1_PC2_PASS` (defaults: the lab robot). Sync PC2's clock first
(g1_intellect `scripts/setup_remote_timesync.sh`, once per boot) or the
viewer's frame age drifts and the RGB/depth synchronizer drops pairs.

## Testing

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

PYTHONPATH=src python3 test/test_m1_perception.py
PYTHONPATH=src python3 -m unittest discover -s test -p 'test_*.py' -v
PYTHONPATH=src python3 scripts/demo_m1.py --image test/data/sample_remote.png --label remote --depth 1.4
# (the fine-tuned model only knows "bottle"; for any other label the demo
#  switches to the stock COCO yolov8n.pt by itself)
PYTHONPATH=src python3 test/mujoco_validation.py       # needs `pip install mujoco`
PYTHONPATH=src python3 test/mujoco_accuracy_eval.py    # same, batched -- accuracy numbers below
```

**Accuracy** (`mujoco_accuracy_eval.py`, simulated RGB-D with known ground
truth, the VisualMimic head-camera mount -- 46° pitch, close to the D435i default -- and the spec's own bottle-radius approximation): across
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

- `perception_node.py` has not yet run against the real RealSense; its
  helpers (`ros_image.py`, calibration loader) are unit-tested and it imports
  cleanly inside the container.
- The default D455 extrinsics are a ChArUco result not yet checked against a
  tape-measured object from this node; see "Camera extrinsics". The
  orchestrator keeps `perception_extrinsics` as a grasp blocker until then.
- `CLASS_RADIUS_M`'s `"bottle"` entry is measured (a real can, see
  "Detector"); `"cup"`/`"remote"`/the default are still unverified
  placeholders for the stock detector.
- If the target object changes again, retrain per "Detector" above.
