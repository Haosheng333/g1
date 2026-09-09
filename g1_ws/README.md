# g1_ws - catkin workspace for this repo's SLAM stack

Builds `slam/` (package `g1_slam`, FAST-LIO2 based) together with its two
external dependencies. Kept separate from `~/g1_intellect/catkin_ws`, which
contains a *different* package also named `g1_slam`.

## Layout

    src/g1_slam            -> ../../slam                     (relative symlink, tracked)
    src/livox_ros_driver2  -> ~/g1_intellect/external/livox_ros_driver2   (symlink, tracked)
    src/fast_lio           external clone, NOT tracked (ignored via .gitignore)

## First-time setup

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
