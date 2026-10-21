# ROS2 (Humble) port of launch/normal_hexagon.launch (ROS1).
#
# 7 机编队主入口：swarm_bridge + random_forest 地图 + 7 组 run_in_sim.launch.py。
# 无人机循环用 Python for 展开，初值/目标值逐一取自原 XML。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (drone_id, init_x, init_y, init_z, target_x, target_y, target_z)
_DRONE_SPECS = [
    (0, -26.0, 0.0, 0.5, 26.0, 0.0, 0.5),
    (1, -23.4, -1.5, 0.5, 28.6, -1.5, 0.5),
    (2, -26.0, -3.0, 0.5, 26.0, -3.0, 0.5),
    (3, -28.6, -1.5, 0.5, 23.4, -1.5, 0.5),
    (4, -28.6, 1.5, 0.5, 23.4, 1.5, 0.5),
    (5, -26.0, 3.0, 0.5, 26.0, 3.0, 0.5),
    (6, -23.4, 1.5, 0.5, 28.6, 1.5, 0.5),
]


def generate_launch_description():
    ego_planner_share = get_package_share_directory('ego_planner')
    swarm_bridge_share = get_package_share_directory('swarm_bridge')

    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    odom_topic = LaunchConfiguration('odom_topic')

    declare_launch_arguments = [
        DeclareLaunchArgument('map_size_x', default_value='70.0'),
        DeclareLaunchArgument('map_size_y', default_value='30.0'),
        DeclareLaunchArgument('map_size_z', default_value='3.0'),
        DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom'),
        # formation_type 在原 XML 中声明但未被引用（引用处已注释），保留以完整对应
        DeclareLaunchArgument('formation_type', default_value='1'),
    ]

    # swarm topic transmitter bridge
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(swarm_bridge_share, 'launch', 'bridge.launch.py')),
        launch_arguments={
            'drone_id': '999',
            'broadcast_ip': '127.0.0.255',
        }.items(),
    )

    # map（类型契约：尺寸/距离为 double，计数为 int，seed 为 int）
    random_forest = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[{
            'map.x_size': 34.0,
            'map.y_size': 15.0,
            'map.z_size': 3.0,
            'map.resolution': 0.1,
            'ObstacleShape.seed': 1,  # int（ROS2 版 random_forest 改用随机种子，不再读取）
            'map.obs_num': 60,
            'ObstacleShape.lower_rad': 0.5,
            'ObstacleShape.upper_rad': 0.5,
            'ObstacleShape.lower_hei': 2.0,
            'ObstacleShape.upper_hei': 3.0,
            'map.circle_num': 20,
            'ObstacleShape.radius_l': 1.0,
            'ObstacleShape.radius_h': 1.2,
            'ObstacleShape.z_l': 0.7,
            'ObstacleShape.z_h': 3.0,
            'ObstacleShape.theta': 0.5,
            'pub_rate': 1.0,
            'min_distance': 0.8,
        }],
    )

    # swarm robots
    run_in_sim_launch = PythonLaunchDescriptionSource(
        os.path.join(ego_planner_share, 'launch', 'run_in_sim.launch.py'))
    drones = []
    for drone_id, init_x, init_y, init_z, target_x, target_y, target_z in _DRONE_SPECS:
        drones.append(IncludeLaunchDescription(
            run_in_sim_launch,
            launch_arguments={
                'drone_id': str(drone_id),
                'init_x': str(init_x),
                'init_y': str(init_y),
                'init_z': str(init_z),
                'target_x': str(target_x),
                'target_y': str(target_y),
                'target_z': str(target_z),
                'map_size_x': map_size_x,
                'map_size_y': map_size_y,
                'map_size_z': map_size_z,
                'odom_topic': odom_topic,
            }.items(),
        ))

    return LaunchDescription([
        *declare_launch_arguments,
        bridge,
        random_forest,
        *drones,
    ])
