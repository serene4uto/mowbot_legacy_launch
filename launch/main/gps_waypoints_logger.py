from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():

    mapviz_config = PathJoinSubstitution(
        [FindPackageShare('mowbot_navigation'), 'mapviz', 'gps_wp_logger.mvc']
    )

    # Declare launch arguments
    declare_lat = DeclareLaunchArgument(
        "lat",
        default_value="36.114016",
        description="Original Latitude"
    )

    declare_lon = DeclareLaunchArgument(
        "lon",
        default_value="128.418550",
        description="Original Longitude"
    )

    declare_rl = DeclareLaunchArgument(
        "rl",
        default_value="true",
        description="Use robot_localization"
    )

    # robot localization
    rl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_localization'), 'launch', 'rl_dual_ekf_navsat.launch.py']
            )
        ),
        condition=IfCondition(LaunchConfiguration("rl"))
    )

    # Include another launch file
    mapviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('mowbot_navigation'), 'launch', 'mapviz.launch.py']
            )
        ),
        launch_arguments={
            "fix_topic": "/combined_gps/filtered",
            "mvc_config": mapviz_config,
        }.items()
    )

    # Node that should start first
    gps_waypoints_node = Node(
        package='py_mowbot_utils',
        executable='gps_waypoints_logger',
        name='gps_waypoints_logger',
        output='screen',
        remappings=[
            ('/gps/fix', '/combined_gps/filtered'),
            ('/imu', '/imu_gps_heading/data')
        ]
    )

    return LaunchDescription([
        declare_lat,
        declare_lon,
        declare_rl,
        mapviz_launch,
        gps_waypoints_node,
        rl_launch,
    ])
