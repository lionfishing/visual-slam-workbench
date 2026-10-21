cmake_minimum_required(VERSION 3.8)
project(rgbd_slam)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

#接入slam
get_filename_component(SLAM_PROJECT_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  ABSOLUTE
)
set(ORB_SLAM3_ROOT
  "${SLAM_PROJECT_ROOT}/third_party/ORB_SLAM3"
)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED
  PATHS "${SLAM_PROJECT_ROOT}/install/pangolin/lib/cmake/Pangolin"
  NO_DEFAULT_PATH
)
find_package(geometry_msgs REQUIRED)

add_executable(rgbd_slam_node src/rgbd_slam_node.cpp)
target_compile_features(rgbd_slam_node PRIVATE cxx_std_17)

ament_target_dependencies(rgbd_slam_node
  rclcpp
  sensor_msgs
  message_filters
  std_msgs
  cv_bridge
  geometry_msgs
)
# 编译
target_include_directories(rgbd_slam_node PRIVATE
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  ${Pangolin_INCLUDE_DIRS}
)
# 链接
target_link_libraries(rgbd_slam_node
  "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so"
  Eigen3::Eigen
  ${Pangolin_LIBRARIES}
)
# 为安装好的节点提供运行时库搜索路径
set_target_properties(rgbd_slam_node PROPERTIES
  INSTALL_RPATH
    "${ORB_SLAM3_ROOT}/lib;${SLAM_PROJECT_ROOT}/install/pangolin/lib"
)

install(TARGETS rgbd_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
