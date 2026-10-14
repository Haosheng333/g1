# g1_ws - catkin workspace for this repo's SLAM + navigation stack

Builds `slam/` (package `g1_slam`, FAST-LIO2 based), `navigation/` (`g1_navigation`) and
`system_bringup/` together with the two external dependencies. Kept separate from `~/g1_intellect/catkin_ws`, which
contains a *different* package also named `g1_slam`.

## Layout

    src/g1_slam            -> ../../slam                     (relative symlink, tracked)
    src/g1_navigation      -> ../../navigation               (relative symlink, tracked)
    src/system_bringup     -> ../../system_bringup           (relative symlink, tracked)
    src/perception         -> ../../perception               (relative symlink, tracked)
    src/arm_planning       -> ../../arm_planning             (relative symlink, tracked)
    src/arm_control        -> ../../arm_control              (relative symlink, tracked)
    src/hand_control       -> ../../hand_control             (relative symlink, tracked)
    src/handover           -> ../../handover                 (relative symlink, tracked)
    src/orchestrator       -> ../../orchestrator             (relative symlink, tracked)
    src/livox_ros_driver2  -> ../../../g1_intellect/external/livox_ros_driver2  (symlink, tracked)
    src/fast_lio           external clone, NOT tracked (ignored via .gitignore)

    src/local_*            untracked per-machine overlays (see "Two layouts" below)
    local.conf             untracked per-machine paths (see local.conf.example)
    CATKIN_IGNORE          untracked marker (see "Two layouts" below)

## Two layouts, one repo

This repo is built on machines whose directory layouts differ, and
`src/livox_ros_driver2` is a **tracked** symlink that encodes one of them. On a
machine where `../../../g1_intellect/external/livox_ros_driver2` does not exist
the link dangles and `roslaunch g1_slam sensors.launch` fails with
`Resource not found: livox_ros_driver2`.

**Do not repoint the tracked symlink.** It is correct for the machine it was
written for, and repointing it fixes one machine by breaking the other. Instead:

    tools/bootstrap_workspace.sh            # report what this machine has (read-only)
    tools/bootstrap_workspace.sh --apply    # create the untracked overlays it needs

The script discovers `livox_ros_driver2` and `fast_lio` from, in priority order:
`$G1_LIVOX_DIR` / `$G1_FAST_LIO_DIR`, then `g1_ws/local.conf` (untracked, copy
`local.conf.example`), then `rospack find`, then documented locations relative to
the repo. It never hard-codes a personal path, never writes a tracked file, and
never installs anything.

When the tracked link dangles it adds a **relative**, untracked overlay symlink
`src/local_<pkg>` beside it (relative so it resolves on both sides of the
container bind mount). The dangling tracked link contributes no package to
catkin, so the overlay leaves exactly one package per name — which the script
verifies and prints.

It also handles the case where this repo sits **inside a parent catkin source
space** (one machine builds that parent rather than this nested workspace). There,
every package reachable through `g1_ws/src` would be discovered twice, so the
script checks for the untracked `g1_ws/CATKIN_IGNORE` marker that stops the
parent descending into here, and creates it with `--apply`.

## System dependencies

Declared in each `package.xml`, so the standard command finds them:

    cd <workspace root>
    rosdep check   --from-paths src --ignore-src --rosdistro noetic   # report
    rosdep install --from-paths src --ignore-src --rosdistro noetic -y  # install

Python packages with no rosdep key (`torch`, `trimesh`, `python-fcl`,
`ultralytics`, `unitree_sdk2py`, `bc_stark_sdk`, `mujoco`) are listed in each
package's `requirements.txt` instead and are imported lazily, so a node that does
not need one still starts and reports `NOT_READY` rather than crashing.

## First-time setup

Run the bootstrap first: if this machine already has `livox_ros_driver2` and
`fast_lio` somewhere (another workspace, a sibling checkout), it will wire them
up and you can skip the clone below.

    tools/bootstrap_workspace.sh --apply

Otherwise, fetch and patch FAST-LIO2 into this workspace:

    cd g1_ws/src
    git clone --recursive https://github.com/hku-mars/FAST_LIO.git fast_lio
    # upstream targets the old livox_ros_driver messages; this repo uses livox_ros_driver2
    cd fast_lio && sed -i 's/livox_ros_driver/livox_ros_driver2/g' \
        CMakeLists.txt package.xml src/laserMapping.cpp src/preprocess.cpp src/preprocess.h
    cd ../..
    source /opt/ros/noetic/setup.bash
    catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release   # ROS_EDITION is required by livox_ros_driver2

`~/g1_env.sh` sources `devel/setup.bash --extend` from here, so a normal
interactive shell already resolves `g1_slam`, `fast_lio` and `livox_ros_driver2`.

## Run

    roslaunch g1_slam slam_bringup.launch rviz:=true
    rosrun g1_slam slam_health_check.py --component mid360
