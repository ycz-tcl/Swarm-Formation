# ROS2 (Humble) port of swarm_bridge/launch/bridge.launch (ROS1).
#
# 参数类型契约：broadcast_ip 为 string，drone_id 为 int；odom_max_freq 原 XML
# value="70"（ROS1 自动推导 int），但 ROS2 端 bridge_node 声明为 double，故传 70.0。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    drone_id = int(LaunchConfiguration('drone_id').perform(context))
    broadcast_ip = LaunchConfiguration('broadcast_ip').perform(context)

    bridge_node = Node(
        package='swarm_bridge',
        executable='bridge_node',
        name='drone_{}_bridge_node'.format(drone_id),
        output='screen',
        remappings=[
            ('my_odom', '/vins_estimator/imu_propagate'),
        ],
        parameters=[{
            'broadcast_ip': broadcast_ip,
            'drone_id': drone_id,
            'odom_max_freq': 70.0,
        }],
    )

    return [bridge_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id'),
        DeclareLaunchArgument('broadcast_ip'),
        OpaqueFunction(function=_launch_setup),
    ])
