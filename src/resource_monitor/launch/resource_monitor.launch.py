from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generates a launch description for publisher and subscriber nodes."""
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('resource_monitor'),
        'config',
        'pub_parameters.yaml'
    )
    # Configure publisher node
    pub = Node(
        package='resource_monitor',
        executable='pub_app',
        name='pub',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        remappings=[
            ("/pub/cpu_usage", "/resource_monitor/cpu_usage"),
            ("/pub/memory_usage", "/resource_monitor/memory_usage"),
        ],
        parameters=[config])
    ld.add_action(pub)

    # Configure subscriber node
    sub = Node(
        package='resource_monitor',
        executable='sub_app',
        name='sub',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True)
    ld.add_action(sub)

    return ld