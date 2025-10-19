#!/usr/bin/env python3
"""
Launch a full VESC telemetry pipeline:
- vesc_command (configured by params YAML)
- optional vesc_gui (with duty cap)
- vesc_csv_logger writing CSVs (focus on current & voltage, plus a few fields)

Usage:
  ros2 launch vesc_ros2 vesc_pipeline.launch.py \
      params_file:=/home/$USER/chrono_WS/src/vesc_ros2/config/vesc.yaml \
      output_dir:=/home/$USER/chrono_WS/data/telemetry \
      filename_prefix:=drive \
      start_gui:=false max_abs_duty:=0.25
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Arguments ---
    default_params = PathJoinSubstitution([
        FindPackageShare('vesc_ros2'), 'config', 'vesc.yaml'
    ])

    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to VESC params YAML.'
    )

    start_gui_arg = DeclareLaunchArgument(
        'start_gui', default_value='false',
        description='Start the Tk GUI.'
    )

    max_abs_duty_arg = DeclareLaunchArgument(
        'max_abs_duty', default_value='0.25',
        description='Duty cap for GUI (0-1).'
    )

    # Logger args
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='/home/'+ '${USER}' +'/chrono_WS/data/telemetry',
        description='Directory to store CSV logs.'
    )
    filename_prefix_arg = DeclareLaunchArgument(
        'filename_prefix', default_value='vesc_run',
        description='Prefix for CSV filenames.'
    )
    topic_name_arg = DeclareLaunchArgument(
        'topic_name', default_value='/vesc/state',
        description='Topic with vesc_msgs/VescStateStamped.'
    )
    flush_every_n_arg = DeclareLaunchArgument(
        'flush_every_n', default_value='20',
        description='Flush CSV every N messages.'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='rclcpp log level.'
    )

    # --- LaunchConfigurations ---
    params_file = LaunchConfiguration('params_file')
    start_gui = LaunchConfiguration('start_gui')
    max_abs_duty = LaunchConfiguration('max_abs_duty')

    output_dir = LaunchConfiguration('output_dir')
    filename_prefix = LaunchConfiguration('filename_prefix')
    topic_name = LaunchConfiguration('topic_name')
    flush_every_n = LaunchConfiguration('flush_every_n')
    log_level = LaunchConfiguration('log_level')

    # --- Nodes ---
    vesc_cmd = Node(
        package='vesc_ros2',
        executable='vesc_command',
        name='vesc_command',
        parameters=[params_file],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )

    vesc_gui = Node(
        package='vesc_ros2',
        executable='vesc_gui',
        name='vesc_gui',
        parameters=[{'max_abs_duty': max_abs_duty}],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(start_gui)
    )

    csv_logger = Node(
        package='vesc_ros2',
        executable='vesc_csv_logger',
        name='vesc_csv_logger',
        output='screen',
        parameters=[{
            'output_dir': output_dir,
            'filename_prefix': filename_prefix,
            'topic_name': topic_name,
            'flush_every_n': flush_every_n,
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        params_file_arg,
        start_gui_arg,
        max_abs_duty_arg,
        output_dir_arg,
        filename_prefix_arg,
        topic_name_arg,
        flush_every_n_arg,
        log_level_arg,
        vesc_cmd,
        vesc_gui,
        csv_logger,
    ])
