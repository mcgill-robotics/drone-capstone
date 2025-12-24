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

    # --- image_proc for fisheye1 ---
    fisheye1_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc_fisheye1',
        namespace='camera/fisheye1',
        output='screen',
        remappings=[
            ('image', 'image_raw'),        # relative to namespace
            ('camera_info', 'camera_info'),
            ('image_rect', 'image_rect'),
        ],
        parameters=[{
            'queue_size': 60,
        }],
    )

    # --- image_proc for fisheye2 ---
    fisheye2_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc_fisheye2',
        namespace='camera/fisheye2',
        output='screen',
        remappings=[
            ('image', 'image_raw'),        # relative to namespace
            ('camera_info', 'camera_info'),
            ('image_rect', 'image_rect'),
        ],
        parameters=[{
            'queue_size': 60,
        }],
    )


    return LaunchDescription([
        realsense_launch,
        fisheye1_proc,
        fisheye2_proc,
        # apriltag_node_1,
        # apriltag_node_2
    ])