# ROS2 (Humble) port of launch/run_in_sim.launch (ROS1).
#
# 组合 advanced_param.py（ego_planner_node）+ traj_server + simulator.py。
# LaunchConfiguration 值经 OpaqueFunction 转成 Python 类型后再传给辅助模块，
# 以满足 int/float/bool 类型契约（flight_type=3 -> int，use_distinctive_trajs -> bool）。

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from advanced_param import generate_advanced_param
from simulator import generate_simulator


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    drone_id = int(arg('drone_id'))
    map_size_x = float(arg('map_size_x'))
    map_size_y = float(arg('map_size_y'))
    map_size_z = float(arg('map_size_z'))
    init_x = float(arg('init_x'))
    init_y = float(arg('init_y'))
    init_z = float(arg('init_z'))
    odom_topic = arg('odom_topic')

    # ---- include advanced_param.xml：值取自原 XML 中的 include 参数 ----
    advanced = generate_advanced_param(
        drone_id=drone_id,
        map_size_x_=map_size_x,
        map_size_y_=map_size_y,
        map_size_z_=map_size_z,
        odometry_topic=odom_topic,
        camera_pose_topic='pcl_render_node/camera_pose',
        depth_topic='pcl_render_node/depth',
        cloud_topic='pcl_render_node/cloud',
        cx=321.04638671875,
        cy=243.44969177246094,
        fx=387.229248046875,
        fy=387.229248046875,
        max_vel=1.5,
        max_acc=8.0,
        planning_horizon=7.5,  # always 1.5x greater than sensing horizon
        use_distinctive_trajs=arg('use_distinctive_trajs').lower() in ('true', '1'),
        flight_type=int(arg('flight_type')),  # 3: use 2D Nav Goal to select goal
        point_num=int(arg('point_num')),
        point0_x=float(arg('target_x')),
        point0_y=float(arg('target_y')),
        point0_z=float(arg('target_z')),
        point1_x=0.0,
        point1_y=0.0,
        point1_z=0.0,
        point2_x=0.0,
        point2_y=0.0,
        point2_z=0.0,
        point3_x=0.0,
        point3_y=0.0,
        point3_z=0.0,
    )
    ego_planner_node = Node(**advanced)

    # ---- trajectory server ----
    traj_server = Node(
        package='ego_planner',
        executable='traj_server',
        name='drone_{}_traj_server'.format(drone_id),
        output='screen',
        remappings=[
            ('position_cmd', 'drone_{}_planning/pos_cmd'.format(drone_id)),
            ('planning/trajectory', 'drone_{}_planning/trajectory'.format(drone_id)),
            ('planning/start', 'drone_{}_planning/start'.format(drone_id)),
            ('planning/finish', 'drone_{}_planning/finish'.format(drone_id)),
        ],
        parameters=[{
            'traj_server.time_forward': 1.0,
        }],
    )

    # ---- include simulator.xml ----
    simulator_nodes = [Node(**spec) for spec in generate_simulator(
        drone_id=drone_id,
        init_x_=init_x,
        init_y_=init_y,
        init_z_=init_z,
        map_size_x_=map_size_x,
        map_size_y_=map_size_y,
        map_size_z_=map_size_z,
        odometry_topic=odom_topic,
    )]

    return [ego_planner_node, traj_server, *simulator_nodes]


def generate_launch_description():
    declare_launch_arguments = [
        # size of map, change the size inflate x, y, z according to your application
        DeclareLaunchArgument('map_size_x', default_value='70.0'),
        DeclareLaunchArgument('map_size_y', default_value='30.0'),
        DeclareLaunchArgument('map_size_z', default_value='3.0'),
        DeclareLaunchArgument('init_x', default_value='-26.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='0.5'),
        DeclareLaunchArgument('target_x', default_value='26.0'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='0.5'),
        # topic of your odometry such as VIO or LIO
        DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom'),
        DeclareLaunchArgument('drone_id', default_value='0'),
        # advanced_param include 的固定值（原 XML 中为字面量）
        DeclareLaunchArgument('flight_type', default_value='3'),
        DeclareLaunchArgument('use_distinctive_trajs', default_value='false'),
        DeclareLaunchArgument('point_num', default_value='1'),
        # 以下参数在原 XML 中声明、但所有引用处均已被注释掉（normal_hexagon 亦
        # 不传入），仅为保持原参数面完整；默认值取自 config/normal_hexagon.yaml
        DeclareLaunchArgument('point0_x', default_value='0.0'),
        DeclareLaunchArgument('point0_y', default_value='0.0'),
        DeclareLaunchArgument('point0_z', default_value='0.0'),
        DeclareLaunchArgument('point1_x', default_value='0.0'),
        DeclareLaunchArgument('point1_y', default_value='0.0'),
        DeclareLaunchArgument('point1_z', default_value='0.0'),
        DeclareLaunchArgument('point2_x', default_value='0.0'),
        DeclareLaunchArgument('point2_y', default_value='0.0'),
        DeclareLaunchArgument('point2_z', default_value='0.0'),
        DeclareLaunchArgument('formation_type', default_value='1'),
        DeclareLaunchArgument('weight_obstacle', default_value='50000.0'),
        DeclareLaunchArgument('weight_swarm', default_value='50000.0'),
        DeclareLaunchArgument('weight_feasibility', default_value='10000.0'),
        DeclareLaunchArgument('weight_sqrvariance', default_value='10000.0'),
        DeclareLaunchArgument('weight_time', default_value='80.0'),
        DeclareLaunchArgument('weight_formation', default_value='15000.0'),
        DeclareLaunchArgument('obstacle_clearance', default_value='0.5'),
        DeclareLaunchArgument('swarm_clearance', default_value='0.5'),
        DeclareLaunchArgument('replan_trajectory_time', default_value='0.1'),
    ]

    return LaunchDescription([
        *declare_launch_arguments,
        OpaqueFunction(function=_launch_setup),
    ])
