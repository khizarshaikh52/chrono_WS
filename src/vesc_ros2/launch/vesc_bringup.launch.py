#!/usr/bin/env python3
"""
Launch file to start VESC command node and (optionally) the Tk GUI with a duty cap.

Usage examples:
  # Use default config path and show GUI capped at 0.25 duty
  ros2 launch vesc_ros2 vesc_bringup.launch.py

  # Provide a different params file and override duty cap
  ros2 launch vesc_ros2 vesc_bringup.launch.py \
      params_file:=/home/<you>/chrono_WS/src/vesc_ros2/config/vesc.yaml \
      max_abs_duty:=0.20 start_gui:=false

Notes:
- start_gui requires a display ($DISPLAY set). On headless machines, set start_gui:=false.
- The max_abs_duty applies only to the GUI node (as per provided CLI example).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch-time arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace for all VESC nodes.'
    )

    # Default to the in-tree config; user can override with an absolute path
    default_params = PathJoinSubstitution([
        FindPackageShare('vesc_ros2'), 'config', 'vesc.yaml'
    ])

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to VESC YAML parameters.'
    )

    start_gui_arg = DeclareLaunchArgument(
        'start_gui', default_value='true',
        description='Whether to launch the Tkinter-based vesc_gui.'
    )

    max_abs_duty_arg = DeclareLaunchArgument(
        'max_abs_duty', default_value='0.25',
        description='Max absolute duty cycle allowed by the GUI (0.0–1.0).'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='rclcpp log level (debug, info, warn, error, fatal).'
    )

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    start_gui = LaunchConfiguration('start_gui')
    max_abs_duty = LaunchConfiguration('max_abs_duty')
    log_level = LaunchConfiguration('log_level')

    # Core command node (headless, uses params from YAML)
    vesc_cmd = Node(
        package='vesc_ros2',
        executable='vesc_command',
        name='vesc_command',
        namespace=namespace,
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Optional GUI node with a duty cap parameter
    vesc_gui = Node(
        package='vesc_ros2',
        executable='vesc_gui',
        name='vesc_gui',
        namespace=namespace,
        output='screen',
        parameters=[{'max_abs_duty': max_abs_duty}],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(start_gui)
    )

    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        start_gui_arg,
        max_abs_duty_arg,
        log_level_arg,
        vesc_cmd,
        vesc_gui,
    ])
