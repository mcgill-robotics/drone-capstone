# drone-capstone

## Utilized Repos
- https://github.com/ros-perception/image_common -humble
- https://github.com/ros-perception/image_pipeline -humble
- https://github.com/ros-perception/vision_opencv (not sure which version)
- https://github.com/idra-lab/realsense_t265_shelfino
- https://github.com/christianrauch/apriltag_ros -3.3.0
- https://github.com/christianrauch/apriltag_msgs -2.0.1
- https://github.com/AprilRobotics/apriltag -v3.4.5
- https://github.com/PX4/px4_msgs/commit/3a7c790eaee5284f4e48091dd77f697e4fd828d3
- https://github.com/PX4/px4_ros_com/commit/86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af

## Jetson Command
```bash
xhost +local:docker
```

## Usage
```bash
cd drone_capstone
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch bringup bringup.launch.py
ros2 topic echo /detections
```
