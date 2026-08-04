from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    ros_domain_id = LaunchConfiguration("ros_domain_id")

    ld.add_action(
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="21",
            description="ROS domain id for robot communication stack",
        )
    )
    ld.add_action(SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id))

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

    database_node = Node(
        package="robot_core",
        executable="database_node",
        parameters=[config_database]
    )

    gnss_rover_node = Node(
        package="robot_core",
        executable="gnss_rover_node",
        parameters=[params]
    )

    signalr_node = Node(
        package="robot_core",
        executable="signalr_node",
        parameters=[params]
    )


    ld.add_action(database_node)
    ld.add_action(core_node)
    ld.add_action(gnss_rover_node)
    ld.add_action(signalr_node)

    return ld
