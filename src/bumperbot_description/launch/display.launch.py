#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    # Declare launch argument 'model' 
    # as the absolute path to the file containing 
    # the xacro description of the robot

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value= os.path.join(
            get_package_share_directory('bumperbot_description'),
            'urdf',
            'bumperbot.urdf.xacro'),
        description='Absolute path to robot xacro file'
    )

    # Define 'robot_description' as a parameter of type string  
    # containing the output of running through xacro 
    # the file defined by the 'model' launch parameter 
    # (i.e. a string containing the full URDF robot description)

    robot_description = ParameterValue( 
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )


    # Declare the 'robot_state_publisher' node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description}
        ]
    )

    # Declare the 'joint_state_publisher' node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Declare the 'rviz2' node with config file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('bumperbot_description'),
                 'rviz', 
                 'display.rviz'
                 )],
    )

    # Return the launch description object
    return LaunchDescription(
        [
            model_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node

        ]
    )
