cmake_minimum_required(VERSION 3.8)
project(rgbd_dataset_player)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

add_executable(player_node src/player_node.cpp)
target_compile_features(player_node PRIVATE cxx_std_17)

target_include_directories(player_node PRIVATE ${OpenCV_INCLUDE_DIRS})
#!两者保持一种写法
target_link_libraries(player_node ${OpenCV_LIBS})

ament_target_dependencies(player_node rclcpp sensor_msgs)

install(TARGETS player_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()