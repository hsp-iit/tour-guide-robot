from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    return LaunchDescription([

        # --- Launch arguments ---
        DeclareLaunchArgument(
            'world',
            default_value='/absolute/path/to/madama.world',
            description='Gazebo world file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        # --- Global sim time ---
        SetParameter(
            name='use_sim_time',
            value=LaunchConfiguration('use_sim_time')
        ),

        # --- Gazebo Harmonic (include upstream launch file) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    ros_gz_sim_share,
                    'launch',
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={
                'gz_args': [LaunchConfiguration('world'), ' -r']
            }.items(),
        ),

        # --- Clock bridge ---
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen'
        ),
    ])
