from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch steering zero calibration tools.

    Starts:
        - rc_node
        - database_node
        - steer_zero_node
    """
    ld = LaunchDescription()

    config_hardware = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "hardware.yaml",
    )

    config_database = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "database.yaml",
    )

    rc_node = Node(
            package="robot_core",
            executable="rc_node",
            parameters=[config_hardware]
    )

    database_node = Node(
            package="robot_core",
            executable="database_node",
            parameters=[config_database]
    )
    

    steer_zero_node = Node(
            package="robot_core",
            executable="steer_zero_node",
    )

    encoder_node =  Node(
        package="robot_core",
        executable="encoder_node",
        parameters=[config_hardware]
    )

    i2c_srv_node = Node(
        package="robot_core",
        executable="i2c_srv_node",
        parameters=[config_hardware]
    )
    

    ld.add_action(i2c_srv_node)
    ld.add_action(rc_node)
    ld.add_action(database_node)
    ld.add_action(encoder_node)
    ld.add_action(steer_zero_node)

    return ld