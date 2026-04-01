cmake_minimum_required(VERSION 3.8)
project(gz_image_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find Gazebo Transport
find_package(gz-transport13 QUIET)
if(gz-transport13_FOUND)
  set(GZ_TRANSPORT_VER 13)
else()
  find_package(gz-transport12 REQUIRED)
  set(GZ_TRANSPORT_VER 12)
endif()

find_package(gz-msgs10 QUIET)
if(NOT gz-msgs10_FOUND)
  find_package(gz-msgs9 REQUIRED)
  set(GZ_MSGS_VER 9)
else()
  set(GZ_MSGS_VER 10)
endif()

add_executable(gz_image_bridge src/gz_image_bridge.cpp)

target_include_directories(gz_image_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(gz_image_bridge
  rclcpp
  sensor_msgs
)

target_link_libraries(gz_image_bridge
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

install(TARGETS gz_image_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
