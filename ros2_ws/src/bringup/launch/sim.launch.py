from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # --- Gazebo → ROS 2 image bridge ---
    gz_image_bridge = Node(
        package='gz_image_bridge',
        executable='gz_image_bridge',
        name='gz_image_bridge',
        output='screen',
    )

    # --- AprilTag detection directly on raw sim image ---
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_sim',
        output='screen',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[
            os.path.join(
                get_package_share_directory('apriltag_ros'),
                'cfg',
                'tags_36h11.yaml'
            )
        ],
    )

    # --- Tag pose from detections node ---
    tag_pose_node = Node(
        package='apriltag_pose_from_detections',
        executable='tag_pose_node',
        name='tag_pose_node',
        output='screen',
        parameters=[{
            'detections_topic': '/detections',
            'camera_info_topic': '/camera/camera_info',
            'tag_size_m': 1.0,
            'tag_id': 9,
        }],
    )

    # --- MicroXRCE agent over UDP for PX4 SITL ---
    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    return LaunchDescription([
        gz_image_bridge,
        apriltag_node,
        tag_pose_node,
        microxrce_agent,
    ])
