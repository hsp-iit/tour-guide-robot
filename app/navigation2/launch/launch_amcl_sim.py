import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    amcl_params = os.path.join(os.getcwd(), 'amcl_params.yaml')
    # par2 = [{'use_sim_time': True},
    #         {'autostart': True},
    #         {'node_names': ['amcl']}]
    return LaunchDescription([
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[amcl_params]
        )])
    # ),
    # Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_localization',
    #     output='screen',
    #     parameters= par2
    # )
    # ])
