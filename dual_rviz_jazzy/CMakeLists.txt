cmake_minimum_required(VERSION 3.16)
project(dual_rviz_jazzy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(dual_rviz src/main.cpp)
ament_target_dependencies(dual_rviz
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  rviz_ogre_vendor
  rtabmap_msgs
  pluginlib
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  ament_index_cpp
)

target_link_libraries(dual_rviz Qt5::Widgets)

install(TARGETS dual_rviz
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
