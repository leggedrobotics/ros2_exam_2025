import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the directory of the robot_navigation package
    pkg_robot_navigation_dir = get_package_share_directory("robot_navigation")

    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("robot_navigation"),
                        "launch",
                        "nav2_bringup.launch.py",
                    ]
                )
            ]
        )
    )

    # Run the waypoint_action_client node
    waypoint_action_client_node = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[
            Node(
                package="robot_navigation",
                executable="follow_waypoints_action_client",
                output="screen",
                parameters=[{"waypoints_file_path": LaunchConfiguration("waypoints_file_path")}],
            )
        ],
    )
    # Return the launch description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "waypoints_file_path",
                default_value=os.path.join(pkg_robot_navigation_dir, "config", "waypoints.yaml"),
                description="Path to the waypoints file",
            ),
            navigation_stack,
            waypoint_action_client_node,
        ]
    )
