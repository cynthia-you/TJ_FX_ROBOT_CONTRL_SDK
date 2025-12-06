#!/usr/bin/env python3
"""
启动机器人控制节点
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取参数文件路径
    params_file = os.path.join(
        get_package_share_directory('your_package_name'),
        'config',
        'realtime_robot_control.yaml'
    )

    robot_control_node = Node(
        package='your_package_name',
        executable='realtime_robot_control',
        name='realtime_robot_control',
        output='screen',
        parameters=[params_file],
        remappings=[],
        emulate_tty=True
    )

    return LaunchDescription([
        robot_control_node,
    ])