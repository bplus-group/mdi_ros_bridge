"""Launch mdi_receive node in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import os

camera_prefix = os.environ.get('CAMERA_PREFIX','')


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='MDI_execution_context',
            namespace=camera_prefix,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    namespace=camera_prefix,
                    package='mdi_daq_converter',
                    plugin='MdiConverterNode',
                    name='mdi_converter'),
                ComposableNode(
                    namespace=camera_prefix,
                    package='mdi_node',
                    plugin='MdiReceiveNode',
                    name='mdi_receiver')
            ],
            remappings=[
                ],
            output='screen',
    )

    description = [container]


    return launch.LaunchDescription(description)
