from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()


    config_hardware = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "hardware.yaml",
    )

    encoder_node = Node(
        package="robot_core",
        executable="encoder_node",
        parameters=[config_hardware]
    )

    i2c_srv_node = Node(
        package="robot_core",
        executable="i2c_srv_node",
        parameters=[config_hardware]
    )

    encoder_listener_node = Node(
        package="robot_core",
        executable="encoder_listener_node"
    )

    ld.add_action(i2c_srv_node)
    ld.add_action(encoder_node)
    ld.add_action(encoder_listener_node)
    
    

    return ld