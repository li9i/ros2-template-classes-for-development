import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  config = os.path.join(
    get_package_share_directory('msg_srv_pkg_cpp'),
    'config',
    'test_yaml.yaml'
    )

  return LaunchDescription([
    Node(
      package="msg_srv_pkg_cpp",
      executable="template_class_node",
      name="template_class_node",
      output="screen",
      emulate_tty=True,
      parameters=[config])
    ])
