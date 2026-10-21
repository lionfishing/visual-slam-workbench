#!/usr/bin/env bash
set -e

slam_project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
cd "$slam_project_root/ros2_ws"
colcon build --packages-select rgbd_dataset_player
source install/local_setup.bash

# Additional ROS arguments override the defaults below.
ros2 run rgbd_dataset_player player_node --ros-args \
  -p dataset_root:="$slam_project_root/data/rgbd_dataset_freiburg1_xyz" \
  -p association_path:="$slam_project_root/third_party/ORB_SLAM3/Examples/RGB-D/associations/fr1_xyz.txt" \
  -p max_frames:=0 \
  -p publish_period_ms:=33 \
  "$@"
