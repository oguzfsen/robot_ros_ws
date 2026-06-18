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

    ld.add_action(gnss_node)
    ld.add_action(telemetry_bridge_node)

    return ld
