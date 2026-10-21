# ROS2 (Humble) port of launch/rviz.launch (ROS1).
#
# rviz1 的 Goal3DTool（topic "goal"）已删除，由 rviz2 内置 2D Goal Pose 取代：
# 默认话题 /goal_pose 经节点 remap 到 /goal（FSM 忽略 z 并强制 1.0，功能等价）。
# 原 required="true" 用 OnProcessExit -> Shutdown 事件等价实现。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'default.rviz')],
        remappings=[
            ('/goal_pose', '/goal'),
        ],
    )

    # rviz 退出时终止整个 launch（对应 ROS1 required="true"）
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([rviz_node, shutdown_on_exit])
