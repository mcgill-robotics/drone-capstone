from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- RealSense T265 ---
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_name': 'camera',
            'device_type': 't265',
            'enable_fisheye1': 'true',
            'enable_fisheye2': 'true',
        }.items(),
    )

    # --- image_proc for fisheye2 ---
    fisheye2_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc_fisheye2',
        namespace='camera/fisheye2',
        output='screen',
        remappings=[
            ('image', 'image_raw'),
            ('camera_info', 'camera_info'),
            ('image_rect', 'image_rect'),
        ],
        parameters=[{
            'queue_size': 60,
        }],
    )

    # --- AprilTag detection node ---
    apriltag_node_2 = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_fisheye2',
        output='screen',
        remappings=[
            ('image_rect', '/camera/fisheye2/image_rect'),
            ('camera_info', '/camera/fisheye2/camera_info'),
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
            'camera_info_topic': '/camera/fisheye2/camera_info',
            'tag_size_m': 1.0,
            'tag_id': 9,
        }],
    )

    return LaunchDescription([
        realsense_launch,
        fisheye2_proc,
        apriltag_node_2,
        tag_pose_node,
    ])
