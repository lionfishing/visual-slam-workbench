set -e
slam_project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
cd "$slam_project_root/ros2_ws"
colcon build --packages-select rgbd_slam
source install/local_setup.bash
ros2 run rgbd_slam rgbd_slam_node --ros-args \
-p vocabulary_path:="$slam_project_root/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
-p settings_path:="$slam_project_root/configs/tum_fr1_rgbd.yaml"
