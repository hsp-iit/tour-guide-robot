from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory
import os


def _get_gz_args(context, *args, **kwargs):
    """Helper function to conditionally build gz_args based on auto_run setting"""
    world = context.launch_configurations['world']
    auto_run = context.launch_configurations['auto_run']

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    gz_args = [world]
    if auto_run.lower() == 'true':
        gz_args.append(' -r')
    custom_args={'gz_args': gz_args}

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments=custom_args.items(),
    )]


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

        DeclareLaunchArgument(
            'auto_run',
            default_value='true',
            description='Automatically run Gazebo simulation (adds -r flag)'
        ),

        # --- Global sim time ---
        SetParameter(
            name='use_sim_time',
            value=LaunchConfiguration('use_sim_time')
        ),

        # --- Gazebo Harmonic (include upstream launch file) ---
        OpaqueFunction(
            function=_get_gz_args
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
