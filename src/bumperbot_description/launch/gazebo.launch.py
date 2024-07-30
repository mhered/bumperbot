#!/usr/bin/env python

import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Set up GAZEBO_MODEL_PATH environmental variable
    # to store path to models and share directories 

    bumperbot_description = get_package_share_directory('bumperbot_description')
    bumperbot_description_prefix = get_package_prefix('bumperbot_description')

    model_path = os.path.join(bumperbot_description, "models")
    model_path += pathsep + os.path.join(bumperbot_description_prefix, "share")
    
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)


    # Declare launch argument 'model' 
    # as the absolute path to the file containing 
    # the xacro description of the robot

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value= os.path.join(
            bumperbot_description,
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

    # launch the gazebo server
    start_gazebo_server = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 
                         'gzserver.launch.py')
        )
    )

    # launch the gazebo client
    start_gazebo_client = IncludeLaunchDescription( 
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 
                         'launch', 
                         'gzclient.launch.py')
        )
    )

    # spawn the robot in gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'bumperbot', '-topic', 'robot_description'],
        output='screen'
    )

    # Return the launch description object
    return LaunchDescription(
        [
            env_variable, 
            model_arg,
            robot_state_publisher_node,
            start_gazebo_server,
            start_gazebo_client,
            spawn_robot

        ]
    )
