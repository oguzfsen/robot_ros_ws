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

    
    core_node = Node(
        package="robot_core",
        executable="core_node",
        parameters=[params]
    )

    rc_node = Node(
        package="robot_core",
        executable="rc_node",
        parameters=[params]
    )

    relay_node = Node(
        package="robot_core",
        executable="relay_node",
        parameters=[params]
    )

    i2c_srv_node = Node(
        package="robot_core",
        executable="i2c_srv_node",
        parameters=[params]
    )

    database_node = Node(
        package="robot_core",
        executable="database_node",
        parameters=[config_database]
    )
    encoder_node =  Node(
        package="robot_core",
        executable="encoder_node",
        parameters=[params]
    )
    steer_node =  Node(
        package="robot_core",
        executable="steer_node",
        parameters=[params]
    )
    compass_node = Node(
        package="robot_core",
        executable="compass_node",
        parameters=[params]
    )

    hubmotor_node = Node(
        package="robot_core",
        executable="hubmotor_node",
        parameters=[params]
    )

    gnss_node = Node(
        package="robot_core",
        executable="gnss_node",
        parameters=[params]
    )

    telemetry_node = Node(
        package="robot_core",
        executable="telemetry_node",
        parameters=[params]
    )

    ld.add_action(i2c_srv_node)
    ld.add_action(database_node)
    ld.add_action(rc_node)
    ld.add_action(relay_node)
    ld.add_action(encoder_node)
    ld.add_action(steer_node)
    ld.add_action(hubmotor_node)
    ld.add_action(compass_node)
    ld.add_action(core_node)
    ld.add_action(gnss_node)
    ld.add_action(telemetry_node)

    return ld
