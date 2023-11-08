"""Launch an action server and an action client as components
   that execute in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

"""Generate launch description with multiple components."""
def generate_launch_description():
  container = ComposableNodeContainer(
      name='action_server_client_container',
      namespace='',
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=[
        ComposableNode(
          package='action_pkg_cpp',
          plugin='action_pkg_cpp::FibonacciActionServer',
          name='action_server'),
        ComposableNode(
          package='action_pkg_cpp',
          plugin='action_pkg_cpp::FibonacciActionClient',
          name='action_client')
        ],
      output='screen',
      )

  return launch.LaunchDescription([container])
