#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression



def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_summit = get_package_share_directory('summit_deskribapena')
    world = LaunchConfiguration("world")
    headless = "False"

    bringup_dir = get_package_share_directory('summit_deskribapena')
    launch_dir = os.path.join(bringup_dir, 'launch')
    world_config = DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_summit, 'worlds', 'empty.world'), ''],
          description='SDF world file')

    use_simulator = "True"


    # Specify the actions
    rviz_config_path = os.path.join(pkg_summit, 'rviz', 'urdf_config.rviz')
    start_rviz2_cmd = ExecuteProcess(
        cmd=['rviz2','--display-config',rviz_config_path], 
        cwd=[launch_dir], 
        output='screen')

    # # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    summit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_summit, 'launch', 'spawn_car.launch.py'),
        )
    )

    ld = LaunchDescription()
    ld.add_action(world_config)
    ld.add_action(gazebo)
    ld.add_action(TimerAction(
            period=5.0,
            actions=[start_rviz2_cmd]))
    ld.add_action(summit)
    return ld
