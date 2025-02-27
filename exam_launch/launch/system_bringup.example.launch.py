from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch simulation
    robot_gazebo_launch = IncludeLaunchDescription(
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
            "bridge_odom_tf": "true",  # Publish odom to base_link transform. The source of the odom is wheel odometry and it drifts over distance traveled.
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )

    # With bridge_odom_tf set to True, the ros_gz_bridge will publish the odom to base_link transform.
    # The odom frame is intialized at the robot's initial pose in the gazebo world frame.
    # Assuming the world frame is coincident with the map frame, to get the robot's pose in the map frame,
    # we need to publish a static transform from map to odom.
    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            LaunchConfiguration("x"),
            "--y",
            LaunchConfiguration("y"),
            "--z",
            "0.21",  # The height of the base_link frame from the ground
            "--yaw",
            LaunchConfiguration("yaw"),
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        output="screen",
    )

    # Launch the robot_navigation stack
    robot_navigation_launch = TimerAction(
        period=3.0,  # Delay in seconds
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
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            robot_gazebo_launch,
            map_to_odom_tf,
            robot_navigation_launch,
        ]
    )
