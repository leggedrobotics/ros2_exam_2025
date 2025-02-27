import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    arg_use_sim_time = LaunchConfiguration("use_sim_time")

    # Get the package directory
    pkg_dir = get_package_share_directory("robot_description")

    # Include robot description launch file
    robot_description_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [pkg_dir, "launch", "robot_description.launch.py"],
        ),
        launch_arguments={"use_sim_time": arg_use_sim_time}.items(),
    )

    # Joint state publisher GUI node
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    # RViz2 node
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg_dir, "config", "visualize_robot.rviz")],
        output="screen",
    )

    # Launch description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
            ),
            robot_description_launch,
            joint_state_publisher,
            rviz2,
        ]
    )
