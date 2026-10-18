set -e
#! 获取脚本目录所在的上一级
slam_project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$slam_project_root/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --packages-select rgbd_dataset_player
source install/local_setup.bash
ros2 run rgbd_dataset_player player_node --ros-args \
  -p dataset_root:="$slam_project_root/data/rgbd_dataset_freiburg1_xyz" \
  -p association_path:="$slam_project_root/third_party/ORB_SLAM3/Examples/RGB-D/associations/fr1_xyz.txt" \
  -p max_frames:=0 \
  -p publish_period_ms:=33