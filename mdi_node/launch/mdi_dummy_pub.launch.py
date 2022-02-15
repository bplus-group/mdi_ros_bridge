"""Launch mdi_receive node in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='MDI_execution_context',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='mdi_node',
                    plugin='MdiReceiveNode',
                    name='mdi_dummy_pub')
                # Add here another component
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
