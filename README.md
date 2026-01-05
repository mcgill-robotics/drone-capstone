# drone-capstone

## Utilized repos
https://github.com/ros-perception/image_common -humble
https://github.com/ros-perception/image_pipeline -humble
https://github.com/ros-perception/vision_opencv (not sure which version)
https://github.com/idra-lab/realsense_t265_shelfino
https://github.com/christianrauch/apriltag_ros -3.3.0
https://github.com/christianrauch/apriltag_msgs -2.0.1
https://github.com/AprilRobotics/apriltag -v3.4.5

## Usage
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch bringup bringup.launch.py
ros2 run apriltag_ros apriltag_node --ros-args     -r image_rect:=/camera/fisheye1/image_rect     -r camera_info:=/camera/fisheye1/camera_info     --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml
ros2 topic echo /detections
