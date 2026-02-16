import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('perception_server')
    config_file_path = os.path.join(pkg_share, 'config', 'perception_server_params.yaml')

    perception_server_node = Node(
        package='perception_server',
        executable='get_scene_objects_server',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([perception_server_node])