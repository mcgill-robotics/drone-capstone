cmake_minimum_required(VERSION 3.8)
project(precision_land_apriltag)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(precision_land_apriltag_node
  src/precision_land_apriltag.cpp
)

ament_target_dependencies(precision_land_apriltag_node
  rclcpp
  px4_msgs
  px4_ros2_cpp
  apriltag_msgs
  sensor_msgs
)

target_link_libraries(precision_land_apriltag_node
  Eigen3::Eigen
)

install(TARGETS
  precision_land_apriltag_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
