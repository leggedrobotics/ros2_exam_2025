from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ########################################################################
    # Don't modify below this line
    ########################################################################
    pub_env_info = Node(
        package="exam_launch",
        executable="pub_env_info.py",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"student_id": LaunchConfiguration("student_id")},
        ],
    )

    # Register event handler for pub_env_info exit
    pub_env_info_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=pub_env_info,
            on_exit=[EmitEvent(event=Shutdown(reason="Student ID validation failed"))],
            handle_once=True,
        )
    )
    ########################################################################
    # Don't modify above this line
    ########################################################################

    # Launch simulation
    robot_gazebo_launch = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("robot_gazebo"),
                                "launch",
                                "robot_gazebo.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "bridge_odom_tf": "false",  # Disable the ros_gz_bridge odom to base_link transform
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "x": LaunchConfiguration("x"),
                    "y": LaunchConfiguration("y"),
                    "yaw": LaunchConfiguration("yaw"),
                }.items(),
            )
        ],
    )

    # With bridge_odom_tf set to false, we need to publish odom to base_link transform manually. There is a precise sensor
    # that provides the transform from map to base_link. For simplicity, we assume the map frame is coincident with the odom frame.
    # We need to publish a static identity transform from map to odom.
    map_to_odom_tf = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
                output="screen",
            )
        ],
    )

    dummy_localization = TimerAction(
        period=1.0,  # Delay in seconds
        actions=[
            Node(
                package="robot_navigation",
                executable="dummy_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            )
        ],
    )

    # Launch the robot_navigation stack
    robot_navigation_launch = TimerAction(
        period=4.0,  # Delay in seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("robot_navigation"),
                                "launch",
                                "robot_navigation.launch.py",
                            ]
                        )
                    ]
                )
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_file",
                default_value="ros2_exam.world",
                description="Path to the world file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "student_id",
                default_value="",  # TODO: Add your student ID here
                description="Student ID",
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            pub_env_info,
            pub_env_info_exit_handler,  # Add the handler right after the node
            robot_gazebo_launch,
            map_to_odom_tf,
            dummy_localization,
            robot_navigation_launch,
        ]
    )
