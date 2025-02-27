import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ros_gz_bridge config
    ros_gz_bridge_config = os.path.join(
        get_package_share_directory("robot_gazebo"),
        "config",
        "robot_gz_bridge.yaml",
    )
    ros_gz_bridge_with_tf_config = os.path.join(
        get_package_share_directory("robot_gazebo"),
        "config",
        "robot_gz_bridge_with_tf.yaml",
    )

    # Publish robot description topic
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_description"),
                    "launch",
                    "robot_description.launch.py",
                ]
            )
        ),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )

    # gz-sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                IfElseSubstitution(LaunchConfiguration("paused"), if_value="", else_value="-r "),
                IfElseSubstitution(LaunchConfiguration("verbose"), if_value="-v4 ", else_value=""),
                LaunchConfiguration("world_file"),
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawn robot model
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_spawn_model.launch.py",
                ]
            )
        ),
        launch_arguments={
            "topic": "/robot_description",
            "entity_name": "robot",
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )

    # ros_gz_bridge config
    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_bridge"),
                    "launch",
                    "ros_gz_bridge.launch.py",
                ]
            )
        ),
        launch_arguments={
            "bridge_name": "ros_gz_bridge",
            "config_file": IfElseSubstitution(
                LaunchConfiguration("bridge_odom_tf"),
                if_value=ros_gz_bridge_with_tf_config,
                else_value=ros_gz_bridge_config,
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_file",
                default_value="empty.world",
                description="Path to the world file",
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.4"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="If true, Gazebo won't start the simulation after loading the world",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument("verbose", default_value="false"),
            DeclareLaunchArgument(
                "bridge_odom_tf",
                default_value="false",
                description="Set to true to if you want to publish the odom to base_link TF calculated estimated by the wheel odometry",
            ),
            robot_description,
            gz_sim,
            spawn_robot,
            ros_gz_bridge,
        ]
    )
