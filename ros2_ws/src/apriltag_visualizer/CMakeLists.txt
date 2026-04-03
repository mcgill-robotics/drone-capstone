cmake_minimum_required(VERSION 3.8)
project(apriltag_visualizer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(apriltag_visualizer src/apriltag_visualizer.cpp)

ament_target_dependencies(apriltag_visualizer
  rclcpp
  sensor_msgs
  apriltag_msgs
  cv_bridge
)

target_link_libraries(apriltag_visualizer
  ${OpenCV_LIBS}
)

install(TARGETS apriltag_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
