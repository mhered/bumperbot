#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Create the launch description

    # Declare the launch arguments and set their default values
    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='true',
        description='Whether to use Python or C++ nodes'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.033',
        description='Wheel radius'
    )

    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.17',
        description='Wheel separation'
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        'use_simple_controller',
        default_value='True',
        description='Whether to use simple controller or the standard diff_drive ros2 control controller'
    )

    # read runtime values of the launch arguments
    use_python = LaunchConfiguration('use_python')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    use_simple_controller = LaunchConfiguration('use_simple_controller')

    # start joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster',
                    '--controller-manager',
                      '/controller_manager' # set namespace
        ]
    )

    # start bumperbor_controller
    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['bumperbot_controller',
                    '--controller-manager',
                    '/controller_manager' # set namespace
        ],
        condition=UnlessCondition(use_simple_controller)
    )

    # define a conditional group of nodes to start the simple controller

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager",  # set namespace
                ],
            ),
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius},
                    {"wheel_separation": wheel_separation},
                ],
                condition=IfCondition(use_python),
            ),
            Node(
                package="bumperbot_controller",
                executable="simple_controller",
                parameters=[
                    {"wheel_radius": wheel_radius},
                    {"wheel_separation": wheel_separation},
                ],
                condition=UnlessCondition(use_python),
            ),
        ],
    )

    # Return the launch description object
    return LaunchDescription(
        [
            use_python_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            use_simple_controller_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller,
        ]
    )
