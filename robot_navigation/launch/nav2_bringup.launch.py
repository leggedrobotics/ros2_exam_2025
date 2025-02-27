import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the directory of the robot_navigation package
    pkg_robot_navigation_dir = get_package_share_directory("robot_navigation")

    # Initialize launch configurations
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level")
    rviz_config_file = LaunchConfiguration("rviz_config")

    # List of lifecycle nodes
    lifecycle_nodes = [
        "map_server",
        "custom_map_saver_server",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"autostart": autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Set environment variables for logging
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    # Group action to load nodes with parameters
    load_nodes = GroupAction(
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[configured_params, {"yaml_filename": map_yaml_file}],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="robot_navigation",
                executable="custom_map_saver_server",
                name="custom_map_saver_server",
                output="screen",
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[configured_params],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{"autostart": autostart}, {"node_names": lifecycle_nodes}],
            ),
        ],
    )

    # RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    # Return the launch description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg_robot_navigation_dir, "config", "nav2_params.yaml"),
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument("log_level", default_value="info", description="log level"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(pkg_robot_navigation_dir, "maps", "ros2_exam.yaml"),
                description="Full path to map yaml file to load",
            ),
            DeclareLaunchArgument(
                name="rviz_config",
                default_value=os.path.join(pkg_robot_navigation_dir, "config", "nav2_exam_view.rviz"),
                description="Path to RViz config file",
            ),
            stdout_linebuf_envvar,
            colorized_output_envvar,
            load_nodes,
            rviz_node,
        ]
    )
