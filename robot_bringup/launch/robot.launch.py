from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # CONFIG PATHS
    config = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "params.yaml",
    )

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

    config_telemetry = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "telemetry.yaml",
    )

    config_telemetry_routes = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "telemetry_routes.yaml",
    )

    #EXECUTIBLE PATHS
    core_node = Node(
        package="robot_core",
        executable="core_node",
        parameters=[config]
    )

    rc_node = Node(
        package="robot_core",
        executable="rc_node",
        parameters=[config_hardware]
    )

    relay_node = Node(
        package="robot_core",
        executable="relay_node",
        parameters=[config_hardware]
    )

    i2c_srv_node = Node(
        package="robot_core",
        executable="i2c_srv_node",
        parameters=[config_hardware]
    )

    database_node = Node(
        package="robot_core",
        executable="database_node",
        parameters=[config_database]
    )
    encoder_node =  Node(
        package="robot_core",
        executable="encoder_node",
        parameters=[config_hardware]
    )
    steer_node =  Node(
        package="robot_core",
        executable="steer_node",
        parameters=[config_hardware]
    )
    compass_node = Node(
        package="robot_core",
        executable="compass_node",
        parameters=[config_hardware]
    )

    hubmotor_node = Node(
        package="robot_core",
        executable="hubmotor_node",
        parameters=[config_hardware]
    )

    gnss_node = Node(
        package="robot_core",
        executable="gnss_node",
        parameters=[config_hardware]
    )

    telemetry_bridge_node = Node(
        package="robot_core",
        executable="telemetry_bridge_node",
        parameters=[
            config_telemetry,
            {
                "routes_file": config_telemetry_routes,
                "endpoint_role": "robot",
            },
        ]
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
    ld.add_action(telemetry_bridge_node)

    return ld
