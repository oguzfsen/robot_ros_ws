from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()


    params = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "params.yaml",
    )

    rc_node = Node(
        package="robot_core",
        executable="rc_node",
        parameters=[params]
    )

    rc_listener_node = Node(
        package="robot_core",
        executable="rc_listener_node",
    )

    ld.add_action(rc_node)
    ld.add_action(rc_listener_node)

    return ld