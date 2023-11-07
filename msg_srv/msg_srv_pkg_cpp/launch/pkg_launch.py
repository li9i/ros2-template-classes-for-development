from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package="ros2_templates_pkg",
      executable="template_class_node",
      name="template_class_node",
      output="screen",
      emulate_tty=True,
      parameters=[
        {"my_parameter": "earth"}
        ]
      )
    ])
