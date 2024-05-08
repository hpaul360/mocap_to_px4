import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_description = LaunchDescription()
    log_level = "warn"
    
    node = Node(
        package='mocap_to_px4',
        executable='converter',
        name='mocap_to_px4',
        parameters=[{'body_id': LaunchConfiguration('body_id', default=1)}],
        output='screen',
        #arguments=['--ros-args', '--log-level', log_level]
    )

    launch_description.add_action(node)
    return launch_description
