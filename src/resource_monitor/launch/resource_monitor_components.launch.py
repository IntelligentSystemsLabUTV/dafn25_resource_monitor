from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('resource_monitor'),
        'config',
        'pub_parameters.yaml'
    )

    #! Create a ComposableNodeContainer, then add ComposableNode instances to it.
    container = ComposableNodeContainer(
        name='resource_monitor',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        composable_node_descriptions=[
            ComposableNode(
                package='resource_monitor',
                plugin='resource_monitor::ResourceMonitorPublisher',
                name='pub',
                namespace='',
                extra_arguments=[{'use_intra_process_comms': True}],
                remappings=[
                    ("/pub/cpu_usage", "/resource_monitor/cpu_usage"),
                    ("/pub/memory_usage", "/resource_monitor/memory_usage"),
                ],
                parameters=[config]),
            ComposableNode(
                package='resource_monitor',
                plugin='resource_monitor::ResourceMonitorSubscriber',
                name='sub',
                namespace='',
                parameters=[],
                extra_arguments=[{'use_intra_process_comms': True}])
        ]
    )
    ld.add_action(container)

    return ld