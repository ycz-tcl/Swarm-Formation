# ROS2 (Humble) port of launch/rviz.launch (ROS1).
#
# rviz1 的 Goal3DTool（topic "goal"）已删除，由 rviz2 内置 2D Goal Pose 取代。
# 目标话题按 FSM 的 flight_type 分支选择（ego_replan_fsm.cpp）：
#   flight_type=3（SWARM_MANUAL_TARGET，demo 默认）-> /move_base_simple/goal
#                                                  （formationWaypointCallback，z 强制 0.5）
#   flight_type=1（MANUAL_TARGET）               -> /goal（waypointCallback，z 强制 1.0）
# 故默认 remap 到 /move_base_simple/goal；若改跑 flight_type=1，请改回 /goal。
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
            # flight_type=3（默认）FSM 订阅 /move_base_simple/goal；flight_type=1 才是 /goal
            ('/goal_pose', '/move_base_simple/goal'),
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
