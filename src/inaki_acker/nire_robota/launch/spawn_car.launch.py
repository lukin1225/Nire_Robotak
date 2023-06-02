#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():


    xacro_file = os.path.join(get_package_share_directory('inaki_acker'), 'robot/', 'junki_acker.urdf.xacro')
    assert os.path.exists(xacro_file), "The xacro doesnt exist in "+str(xacro_file)

    install_dir = get_package_prefix('inaki_acker')
    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.00'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    timeout = '3000.0'

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'


    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()


    spawn_car =  Node(package="gazebo_ros", executable='spawn_entity.py', arguments=
          ["-topic", "/robot_description", "-entity", "car", '-timeout', timeout, '-x', pose['x'], '-y', pose['y'], '-z', pose['z'], '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])
    robot_state_publisher =   Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_desc}], remappings=remappings,
                output="screen")

    ld =  LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        TimerAction(
            period=0.0,
            actions=[spawn_car]),
        TimerAction(
            period=0.0,
            actions=[robot_state_publisher]
        )
,

    ])

    return ld

