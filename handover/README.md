# Tactile Handover for Unitree G1

This ROS1 catkin package adds a handover coordinator, a tactile release monitor, and mock dependencies to the supplied G1 project. The original Desktop checkout is unchanged.

**Validation:** five pure Python detector tests pass. ROS compilation, ROS integration, trained-model loading, and physical robot tests have not been performed.

## What the module does

After navigation reaches the recipient and the robot stops, this module extends the arm, waits for someone to take the bottle, opens the hand, and retracts the arm.

```text
Navigation confirms arrival and standstill
    -> Orchestrator confirms a verified grasp
    -> Handover requests all motion paths
    -> Arm executes staging and reaching paths
    -> Wait 1 second
    -> Tactile monitor establishes a baseline and detects a pull
    -> Coordinator disarms monitoring and opens the hand
    -> Wait 1.5 seconds
    -> Arm executes retraction and home paths
    -> Publish DONE
```

The monitor only requests release. It never commands the hand or arm directly.

## Beginner's guide to ROS terms

| Term | Meaning in this project |
|---|---|
| Node | A running program, such as the tactile monitor |
| Topic | A stream of messages, such as fingertip readings |
| Service | A request and response, such as asking the hand to open |
| Package | A folder of related ROS code and its build configuration |
| Launch file | A configuration that starts several nodes together |
| Mock | A software substitute for a real device or service |

## File structure and responsibilities

```text
handover/
├── README.md
├── package.xml
├── CMakeLists.txt
├── setup.py
├── src/handover/
│   ├── __init__.py
│   └── detector.py
├── scripts/
│   ├── auto_release_monitor.py
│   ├── handover_node.py
│   └── mock_dependencies.py
├── srv/
│   ├── ArmMonitor.srv
│   ├── SetHeldObject.srv
│   └── PlanHandover.srv
├── launch/
│   ├── demo.launch
│   └── handover.launch
├── config/example.yaml
├── tests/test_detector.py
└── docs/GITHUB_GUIDE.md
```

| File | Responsibility | Relationship to other files |
|---|---|---|
| `src/handover/detector.py` | Detect sustained tactile changes | Imported by the monitor; tested without ROS |
| `scripts/auto_release_monitor.py` | Receive tactile data and request release once | Runs `Detector`; calls the coordinator's release service |
| `scripts/handover_node.py` | Coordinate planning, reaching, release, and retraction | Calls arm, hand, and monitor services |
| `scripts/mock_dependencies.py` | Simulate planning, movement, opening, and tactile readings | Used by `demo.launch`; never starts hardware SDKs |
| `srv/ArmMonitor.srv` | Define arm and object-label fields for enabling monitoring | Used by coordinator and monitor |
| `srv/SetHeldObject.srv` | Define how a verified grasp is registered | Called by the main orchestrator |
| `srv/PlanHandover.srv` | Define a complete four-leg handover plan | Must be implemented by the arm-planning adapter |
| `launch/demo.launch` | Start coordinator, monitor, and mocks | For software integration demonstrations only |
| `launch/handover.launch` | Start coordinator and monitor with an explicit configuration | Requires real dependencies to be running separately |
| `config/example.yaml` | Provide example target and detector settings | Must be reviewed and calibrated before physical use |
| `tests/test_detector.py` | Exercise detector behavior with synthetic readings | Imports the detector directly |
| `package.xml` | Declare ROS and runtime dependencies | Read by catkin and dependency tooling |
| `CMakeLists.txt` | Generate service types and install scripts | Read during the catkin build |
| `setup.py` | Make the internal Python package importable | Allows nodes to import `handover.detector` |
| `__init__.py` | Mark the internal Python package | Intentionally empty |

## Detection behavior

The detector transitions through `IDLE -> BASELINING -> MONITORING -> TRIGGERED`. Invalid or interrupted data enters `FAULT`.

- Collect 10 stable frames before establishing a baseline.
- Average the latest 10 frames to reduce noise.
- Slowly update the baseline for changes below the threshold.
- Require a sustained change on the same unmasked channel before triggering.
- Suppress release triggers for the first second after enabling the monitor.
- Send only one automatic release notification per arming operation.
- Enter FAULT on invalid values, changing channel count, or interrupted input.

A failed notification is logged without an automatic retry. Operator intervention is required.

## Software requirements

Use the team's existing **Ubuntu 20.04, ROS1 Noetic, and Python 3** environment for compatibility with this ROS1 project. This delivery does not include a complete ROS environment for macOS. macOS can be used to inspect files and run the pure Python detector tests.

Required components: catkin, rospy, std_msgs, std_srvs, message_generation, message_runtime, catkin_pkg, and python3-yaml.

The mock demonstration does not require a camera, LiDAR, PyTorch, MuJoCo, or a physical robot. Physical operation additionally requires the team's G1 SDK/DDS environment, Revo2 SDK/Modbus connection, and working arm-planning and execution nodes. The tactile detector does not require neural-network training.

## Build and run the mock demonstration

In an Ubuntu terminal, replace PROJECT with the absolute path to the delivered `g1-handover` folder:

```bash
source /opt/ros/noetic/setup.bash
export PROJECT=/absolute/path/to/g1-handover
mkdir -p ~/handover_ws/src
ln -s "$PROJECT/handover" ~/handover_ws/src/handover
ln -s "$PROJECT/arm_control" ~/handover_ws/src/arm_control
cd ~/handover_ws
catkin_make
source devel/setup.bash
roslaunch handover demo.launch
```

Use a new workspace or check that these links do not already exist. Only these two packages are needed for the mock demonstration. Other supplied folders are not necessarily complete catkin packages.

In a second terminal, source the same workspace:

```bash
source ~/handover_ws/devel/setup.bash
rosservice call /handover/set_held_object "{arm: right, label: bottle}"
rosservice call /orchestrator/run_handover "{}"
rostopic echo /handover/status
```

In a third terminal, simulate someone pulling the bottle:

```bash
source ~/handover_ws/devel/setup.bash
rosparam set /handover_mock_dependencies/pull true
```

Expected states include `RELEASING`, `RETRACTING`, and `DONE`. The mock publishes right-hand data only, so use `right`. Before repeating the demonstration, set `pull` back to `false` and register the held object again.

**The mock validates software sequencing only. It does not validate reachability, collision avoidance, actual grasping, or physical release latency.**

## Interfaces and team prerequisites

### Arm planning

The arm team must implement `/arm/plan_handover` using `handover/PlanHandover`. This is a new integration contract, not an existing implemented service in the supplied project.

The request contains the arm and target x/y/z in `torso_link`. The response must contain four flattened N-by-5 joint paths, in radians: staging, reaching, retraction, and home.

Before any movement, the adapter must validate IK, joint limits, and collisions for every leg. The staging path must begin at the actual current joint state; consecutive legs must share matching endpoints. Reject unsupported arms, unreachable targets, and stale robot state. Account for the held bottle when checking relevant paths.

The supplied model code defines a right-arm model with input `[x, y, z, vx, vy]`. Do not pass the specification's `[x, y, z, rx, rz]` values or NaN orientations directly into this model. The adapter must choose a valid orientation and validate forward-kinematics error.

The specification's target `(0.35, 0, 0.15)` is outside the training workspace configured in `arm_config.py`. The target in `example.yaml` is also only an example and requires validation. This implementation executes a preplanned home path instead of assuming a separate `go_home` service exists.

### Arm execution

Reuse `/arm/move_arm_joints`, type `arm_control/MoveArmJoints`. Confirm arm support, low-level speed limits, actual arrival detection, and finite execution timeouts.

The coordinator subdivides consecutive points within a path to at most 5 degrees per joint. This does not replace the controller's velocity limit. The planner must generate the initial point from current robot state.

### Hand and tactile data

Provide `/hand_left/release` or `/hand_right/release`, type `std_srvs/Trigger`. Success must reflect the actual release outcome.

Provide a tactile bridge publishing `/hand_{side}/tactile`, type `std_msgs/Float64MultiArray`, at approximately 30 Hz. Suggested data order:

```text
thumb normal, thumb tangential,
index normal, index tangential,
middle normal, middle tangential,
ring normal, ring tangential,
pinky normal, pinky tangential
```

This is the new package's adapter contract, not a claim that the vendor SDK already publishes this ROS message. Confirm channel meanings and units with the hand team.

On device disconnection, stop publishing rather than repeatedly sending cached values. This message has no device timestamp, so the monitor detects reception gaps but cannot identify cached sensor readings. Do not include cyclic direction angles or proximity values under a shared threshold without calibration.

Record stationary holding and pulls from multiple directions, then calibrate `threshold`, `stable_span`, `debounce`, `stale`, and `masked`.

### Navigation and overall orchestration

Navigation must confirm arrival, final orientation, and standstill. This package does not send navigation goals.

After a verified successful grasp, the main orchestrator calls `/handover/set_held_object` with arm and label. After confirmed standstill, call `/orchestrator/run_handover`.

The start service returning success means **waiting for the recipient**, not completed delivery. Completion is published as `/handover/status = DONE`.

The independent coordinator locks only its own handover and release operations. The main orchestrator must enforce a global execution lock that blocks navigation, grasping, and other arm tasks throughout handover. Do not run another provider of the same service names simultaneously. If moving this logic into the main orchestrator, share its lock and held-object state rather than maintaining duplicate state.

FAULT, motion failures, and prolonged waiting require operator handling. This version does not automatically open the hand or retract on a waiting timeout.

## Physical operation

First validate and start the real planning adapter, arm controller, hand node, and tactile bridge separately. Do not start the mock node alongside them.

Use an explicitly reviewed configuration:

```bash
roslaunch handover handover.launch config:=/absolute/path/reviewed.yaml
```

Validate posture, workspace, collision avoidance while holding the bottle, and global task exclusion before recipient trials.

If opening succeeds but retraction fails, the coordinator clears the held-object label and enters FAULT rather than reporting success. Manual recovery is required. Service discovery has a timeout; a service response does not have a separate forced cancellation mechanism. Service implementations must enforce finite execution timeouts.

## Recommended development order

1. Upload this package to a separate GitHub branch for team review.
2. Build it in the team's ROS environment.
3. Run the mock demonstration and observe DONE after a simulated pull.
4. Connect real tactile readings and test detection without executing release.
5. Verify hand release independently.
6. Validate planned arm paths and execution independently.
7. Test the complete handover.
8. Connect navigation and the main orchestrator.

## Tests and acceptance criteria

From the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=handover/src python3 -m unittest discover -s handover/tests -v
```

Tests cover 60 seconds of synthetic stationary data without release, positive and negative step changes triggering once within one second, unstable baseline rejection, invalid or interrupted input, and masked channels.

Synthetic tests do not replace physical acceptance tests: release within one second of a recipient taking the bottle from different directions, and zero false releases during 60 seconds of stationary holding.

For first-time GitHub users, see [the upload guide](docs/GITHUB_GUIDE.md).
