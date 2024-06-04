import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_description = LaunchDescription()
    log_level = "warn"
    
    config = os.path.join(
        get_package_share_directory('mocap_client'),
        'config',
        'params.yaml'
        )
        
    node = Node(
        package = 'mocap_client',
        name = 'mocap_client',
        executable = 'mocap_client',
        parameters = [config],
        #arguments=['--ros-args', '--log-level', log_level]
    )
    
    launch_description.add_action(node)
    return launch_description
