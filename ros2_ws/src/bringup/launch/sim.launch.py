from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    tags_config = os.path.join(
        get_package_share_directory('apriltag_ros'),
        'cfg',
        'tags_36h11.yaml'
    )

    gz_image_bridge = Node(
        package='gz_image_bridge',
        executable='gz_image_bridge',
        name='gz_image_bridge',
        output='screen',
    )

    # AprilTag detector subscribes to /camera/image_rect and /camera/camera_info
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_detector',
        output='screen',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[tags_config],
    )

    # tag overlay on image feed
    apriltag_viz = Node(
        package='apriltag_visualizer',
        executable='apriltag_visualizer',
        name='apriltag_viz',
        output='screen',
    )

    # PX4 Micro XRCE agent
    microxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    return LaunchDescription([
        gz_image_bridge,
        apriltag_node,
        apriltag_viz,
        microxrce_agent,
    ])