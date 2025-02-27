from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg_use_sim_time = LaunchConfiguration("use_sim_time")
    arg_file_name = LaunchConfiguration("file_name")

    # Get the package directory
    pkg_path = get_package_share_directory("robot_description")

    # Robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", PathJoinSubstitution([pkg_path, "urdf", arg_file_name])]), value_type=str
                ),
                "use_sim_time": arg_use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="true",
            ),
            DeclareLaunchArgument(
                name="file_name",
                default_value="robot_standalone.urdf.xacro",
                description="Name of the XACRO/URDF file to load",
            ),
            robot_state_publisher,
        ]
    )
