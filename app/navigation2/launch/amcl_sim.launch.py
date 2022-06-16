import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    amcl_params = os.path.join(os.getcwd(), '..', 'conf', 'amcl_params.yaml')
    return LaunchDescription([
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[amcl_params]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='amcl_lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['amcl']}]
        )
    ])
