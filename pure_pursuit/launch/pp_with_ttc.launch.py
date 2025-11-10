from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('pure_pursuit')
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='scan_crop_node',
            name='scan_crop_node',
            parameters=[os.path.join(pkg, 'params', 'scan_crop.yaml')],
            output='screen'
        ),
        Node(
            package='pure_pursuit',
            executable='pure_pursuit',
            name='pure_pursuit',
            parameters=[os.path.join(pkg, 'params', 'pure_pursuit.yaml')],
            output='screen'
        ),
    ])

