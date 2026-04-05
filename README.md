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
- https://github.com/Auterion/px4-ros2-interface-lib/commit/365dd8807869fd81813de0415ec99e85ea021d59

## Jetson Command
```bash
xhost +local:docker
cd drone_capstone
docker compose up -d
docker exec -it t265-jetson bash
```

Requirement for Q Ground Control is disabled in container, but it can be run in another terminal with 
```bash
flatpak run org.mavlink.qgroundcontrol
```

## Usage
### Terminal 1 in Container
```bash
cd PX4_Autopilot
```
To run with gui run:
```bash
PX4_GZ_WORLD=apriltag make px4_sitl gz_x500_mono_cam_down NAV_DLL_ACT=0
```
Otherwise, you can run the sim headless with
```bash
PX4_GZ_WORLD=apriltag make px4_sitl gz_x500_mono_cam_down NAV_DLL_ACT=0 HEADLESS=1
```

### Terminal 2 in Container
```bash
cd drone_capstone
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch bringup sim.launch
```

# Terminal 3 in Container
```bash
ros2 run precision_land_apriltag precision_land_apriltag_node
```

# Terminal 4 in Container
```bash
ros2 run rqt_image_view rqt_image_view --clear-config
```
