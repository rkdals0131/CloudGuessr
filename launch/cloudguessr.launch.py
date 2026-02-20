#!/usr/bin/env python3
"""
CloudGuessr Launch File

Usage:
    ros2 launch cloudguessr cloudguessr.launch.py
    ros2 launch cloudguessr cloudguessr.launch.py map_vis:=/path/to/map.pcd
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('cloudguessr')
    ws_dir = os.path.abspath(os.path.join(pkg_dir, '..', '..', '..', '..'))
    venv_python = os.path.join(ws_dir, '.venv', 'bin', 'python3')
    py_exec = venv_python if os.path.exists(venv_python) else 'python3'

    data_dir = os.path.join(pkg_dir, 'data')
    scripts_dir = os.path.join(pkg_dir, 'scripts')
    config_dir = os.path.join(pkg_dir, 'config')

    # Default paths
    default_map_vis = os.path.join(data_dir, 'campus_q16.pcd')
    default_map_score = os.path.join(data_dir, 'campus_q8.pcd')
    default_rounds_dir = os.path.join(data_dir, 'rounds')
    query_viewer_script = os.path.join(scripts_dir, 'query_viewer.py')
    hmi_display_script = os.path.join(scripts_dir, 'hmi_display.py')
    desktop_gui_script = os.path.join(scripts_dir, 'cloudguessr_gui.py')
    rviz_config = os.path.join(config_dir, 'cloudguessr.rviz')

    ui_mode_arg = DeclareLaunchArgument(
        'ui_mode',
        default_value='desktop_gui',
        description="UI mode: rviz | desktop_gui | hybrid"
    )

    # Launch arguments
    map_vis_arg = DeclareLaunchArgument(
        'map_vis',
        default_value=default_map_vis,
        description='Map file for visualization (lower resolution)'
    )

    map_score_arg = DeclareLaunchArgument(
        'map_score',
        default_value=default_map_score,
        description='Map file for scoring (higher resolution)'
    )

    rounds_dir_arg = DeclareLaunchArgument(
        'rounds_dir',
        default_value=default_rounds_dir,
        description='Directory containing round data'
    )

    # Map Server Node
    map_server_node = Node(
        package='cloudguessr',
        executable='map_server_node',
        name='map_server',
        output='screen',
        parameters=[{
            'map_file': LaunchConfiguration('map_vis'),
            'map_frame': 'map',
            'voxel_size': 0.0,
            'publish_rate': 0.5,
        }]
    )

    # Round Manager Node
    round_manager_node = Node(
        package='cloudguessr',
        executable='round_manager_node',
        name='round_manager',
        output='screen',
        parameters=[{
            'map_file': LaunchConfiguration('map_score'),
            'map_frame': 'map',
            'rounds_dir': LaunchConfiguration('rounds_dir'),
            'roi_radius': 20.0,
            'voxel_size': 0.5,
            'icp_max_iter': 50,
            'icp_max_corr_dist': 2.0,
            'fail_min_fitness': 0.3,
            'fail_max_rmse': 2.0,
            'yaw_candidates_deg': [0, 45, 90, 135, 180, 225, 270, 315],
            'use_xy_distance': True,
            'click_debounce_ms': 300,
            'auto_advance': False,
            'result_display_sec': 5.0,
        }]
    )

    # Query Viewer (Python script with Open3D)
    query_viewer = ExecuteProcess(
        cmd=[py_exec, query_viewer_script],
        name='query_viewer',
        output='screen',
    )

    # HMI Display
    hmi_display = ExecuteProcess(
        cmd=[py_exec, hmi_display_script],
        name='hmi_display',
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ui_mode'), "' in ['rviz', 'hybrid']"])),
    )

    desktop_gui = ExecuteProcess(
        cmd=[py_exec, desktop_gui_script],
        name='desktop_gui',
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ui_mode'), "' in ['desktop_gui', 'hybrid']"])),
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ui_mode'), "' in ['rviz', 'hybrid']"])),
    )

    return LaunchDescription([
        ui_mode_arg,
        map_vis_arg,
        map_score_arg,
        rounds_dir_arg,
        map_server_node,
        round_manager_node,
        query_viewer,
        hmi_display,
        desktop_gui,
        rviz_node,
    ])
