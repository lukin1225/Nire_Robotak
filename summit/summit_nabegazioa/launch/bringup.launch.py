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

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_summit = get_package_share_directory('summit_nabegazioa')
    world = LaunchConfiguration("world")
    map_yaml_file = LaunchConfiguration('map')
    headless = "False"



    bringup_dir = get_package_share_directory('summit_nabegazioa')
    launch_dir = os.path.join(bringup_dir, 'launch')
    world_config = DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_summit, 'worlds', 'etxea.world'), ''],
          description='SDF world file')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 'maps', 'etxea_mapa.yaml'),
        description='Full path to map file to load')

    use_simulator = "True" # LaunchConfiguration('use_simulator')


    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',  '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    rviz_config_path = os.path.join(pkg_summit, 'rviz', 'nav2_default_view.rviz')
    start_rviz2_cmd = ExecuteProcess(
        cmd=['rviz2','--display-config',rviz_config_path], 
        cwd=[launch_dir], 
        output='screen')


    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')
    nav2_params_path = os.path.join(get_package_share_directory('summit_nabegazioa'), 'params/', 'nav2_params.yaml')

    behavior_tree_path = os.path.join(get_package_share_directory('summit_nabegazioa'), 'behavior_trees', 'navigate_w_replanning_time2.xml')
    configured_nav2_params = RewrittenYaml(
        source_file = nav2_params_path,
        param_rewrites = {'default_nav_to_pose_bt_xml': behavior_tree_path},
        convert_types=True)

    nav_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'bringup_launch.py')),
        launch_arguments={
                          #'namespace': namespace,
                          #'use_namespace': use_namespace,
                          'slam': "False",
                          #'map': 'map.yaml',
                          'map': map_yaml_file,
                          #'use_sim_time': use_sim_time,
                          'params_file':configured_nav2_params,
                          #'params_file': nav2_params_path,
                          #'autostart': autostart}.items()
                        }.items()
        )

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

    joy = Node(
            package='joy', executable='joy_node', output='screen')
    teleop_twist = Node(
            package='teleop_twist_joy', executable='teleop_node', output='screen', parameters=[os.path.join(pkg_summit, "config", "teleop_twist_joy.yaml")])

    ld = LaunchDescription()
    ld.add_action(world_config)
    ld.add_action(gazebo)
    ld.add_action(declare_map_yaml_cmd)
    # ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    ld.add_action(TimerAction(
            period=5.0,
            actions=[start_rviz2_cmd]))
    ld.add_action(summit)
    ld.add_action(joy)
    ld.add_action(teleop_twist)
    ld.add_action(
        TimerAction(
            period=0.0,
            actions=[nav_bringup_cmd]
        ))
    return ld


