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

    config_database = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "database.yaml",
    )

    rc_node = Node(
            package="robot_core",
            executable="rc_node",
            parameters=[params]
    )

    database_node = Node(
            package="robot_core",
            executable="database_node",
            parameters=[config_database]
    )
    

    steer_zero_node = Node(
            package="robot_core",
            executable="steer_zero_node",
            parameters=[params]
    )

    
    ld.add_action(rc_node)
    ld.add_action(database_node)
    ld.add_action(steer_zero_node)

    return ld