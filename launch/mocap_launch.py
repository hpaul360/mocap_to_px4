import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_description = LaunchDescription()
    log_level = "warn"

    config = os.path.join(
        get_package_share_directory('mocap_client'),
        'config',
        'params.yaml'
        )
    
    mocap_node = Node(
        package='mocap_client',
        executable='mocap_client',
        name='mocap_client',
        parameters=[config],
    )

    converter_node = Node(
        package='mocap_to_px4',
        executable='converter',
        name='mocap_to_px4',
        parameters=[{'body_id': LaunchConfiguration('body_id', default=1)}],
        output='screen',
    )

    launch_description.add_action(mocap_node)
    launch_description.add_action(converter_node)
    return launch_description
