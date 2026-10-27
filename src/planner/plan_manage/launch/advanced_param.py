# ROS2 (Humble) port of launch/advanced_param.xml (ROS1).
#
# 提供 generate_advanced_param(...)：返回 ego_planner_node 的 Node 构造 kwargs
# （package/executable/name/output/remappings/parameters）。由 run_in_sim.launch.py
# 调用，参数已按 ROS2 类型契约转换：
#   - 参数名  '~a/b'  ->  'a.b'（'/' -> '.'）
#   - int 参数保持 int（flight_type/waypoint_num/pose_type/local_map_margin/
#     depth_filter_margin/skip_pixel/constrain_points_perPiece/drone_id）
#   - double 参数保持浮点（如 planning_horizen_time 原 XML value="3" 但 type="double" -> 3.0）
#   - 原 <rosparam file="config/normal_hexagon.yaml"/> 在此加载并展平为 '.' 分隔键

import os

import yaml

_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))
_NORMAL_HEXAGON_YAML = os.path.join(_LAUNCH_DIR, os.pardir, 'config', 'normal_hexagon.yaml')


def _flatten_params(value, prefix=''):
    """ROS1 rosparam 嵌套 YAML -> ROS2 '.' 分隔参数名（a/b/c -> a.b.c）。"""
    flat = {}
    for key, val in value.items():
        name = '{}.{}'.format(prefix, key) if prefix else str(key)
        if isinstance(val, dict):
            flat.update(_flatten_params(val, name))
        else:
            flat[name] = val
    return flat


def _load_normal_hexagon_yaml():
    """<rosparam file="$(find ego_planner)/config/normal_hexagon.yaml"/> 的等价物。"""
    with open(_NORMAL_HEXAGON_YAML, 'r') as f:
        return _flatten_params(yaml.safe_load(f))


def generate_advanced_param(
        drone_id,
        map_size_x_, map_size_y_, map_size_z_,
        odometry_topic, camera_pose_topic, depth_topic, cloud_topic,
        cx, cy, fx, fy,
        max_vel, max_acc, planning_horizon,
        use_distinctive_trajs, flight_type, point_num,
        point0_x, point0_y, point0_z,
        point1_x, point1_y, point1_z,
        point2_x, point2_y, point2_z,
        point3_x, point3_y, point3_z):
    """对应 advanced_param.xml 的 ego_planner_node；返回 Node(...) 的 kwargs dict。

    类型：drone_id/flight_type/point_num 为 int；尺寸/速度/内参等为 float；
    use_distinctive_trajs 为 bool；话题为 str。
    """

    parameters = _load_normal_hexagon_yaml()

    # ------------------------------ planning fsm ------------------------------
    parameters.update({
        'fsm.flight_type': int(flight_type),
        'fsm.thresh_replan_time': 1.0,
        'fsm.thresh_no_replan_meter': 1.0,
        'fsm.planning_horizon': float(planning_horizon),  # always 1.5x sensing horizon
        'fsm.planning_horizen_time': 3.0,  # 原 XML value="3" type="double" -> 3.0
        'fsm.emergency_time': 1.0,
        'fsm.realworld_experiment': False,
        'fsm.fail_safe': True,
        'fsm.result_file': '/home/zuzu/Documents/report/ego_swarm_1.txt',
        'fsm.waypoint_num': int(point_num),
        'fsm.waypoint0_x': float(point0_x),
        'fsm.waypoint0_y': float(point0_y),
        'fsm.waypoint0_z': float(point0_z),
        'fsm.waypoint1_x': float(point1_x),
        'fsm.waypoint1_y': float(point1_y),
        'fsm.waypoint1_z': float(point1_z),
        'fsm.waypoint2_x': float(point2_x),
        'fsm.waypoint2_y': float(point2_y),
        'fsm.waypoint2_z': float(point2_z),
        'fsm.waypoint3_x': float(point3_x),
        'fsm.waypoint3_y': float(point3_y),
        'fsm.waypoint3_z': float(point3_z),
    })

    # ------------------------------- grid map ---------------------------------
    parameters.update({
        'grid_map.resolution': 0.1,
        'grid_map.map_size_x': float(map_size_x_),
        'grid_map.map_size_y': float(map_size_y_),
        'grid_map.map_size_z': float(map_size_z_),
        'grid_map.local_update_range_x': 5.5,
        'grid_map.local_update_range_y': 5.5,
        'grid_map.local_update_range_z': 4.5,
        'grid_map.obstacles_inflation': 0.1,
        'grid_map.local_map_margin': 10,  # int
        'grid_map.ground_height': -0.01,
        # camera parameter
        'grid_map.cx': float(cx),
        'grid_map.cy': float(cy),
        'grid_map.fx': float(fx),
        'grid_map.fy': float(fy),
        # depth filter
        'grid_map.use_depth_filter': True,
        'grid_map.depth_filter_tolerance': 0.15,
        'grid_map.depth_filter_maxdist': 5.0,
        'grid_map.depth_filter_mindist': 0.2,
        'grid_map.depth_filter_margin': 2,  # int
        'grid_map.k_depth_scaling_factor': 1000.0,
        'grid_map.skip_pixel': 2,  # int
        # local fusion
        'grid_map.p_hit': 0.65,
        'grid_map.p_miss': 0.35,
        'grid_map.p_min': 0.12,
        'grid_map.p_max': 0.90,
        'grid_map.p_occ': 0.80,
        'grid_map.min_ray_length': 0.1,
        'grid_map.max_ray_length': 4.5,
        'grid_map.virtual_ceil_height': 3.0,
        'grid_map.visualization_truncate_height': 2.8,
        'grid_map.show_occ_time': False,
        'grid_map.pose_type': 1,  # int（不是 1.0）
        'grid_map.frame_id': 'world',
        # sdf map
        'grid_map.local_bound_inflate': 0.0,
        'grid_map.show_esdf_time': False,
        'grid_map.esdf_slice_height': 0.3,
    })

    # ---------------------------- planner manager -----------------------------
    parameters.update({
        'manager.max_vel': float(max_vel),
        'manager.max_acc': float(max_acc),
        'manager.control_points_distance': 0.4,
        'manager.polyTraj_piece_length': 2.0,
        'manager.feasibility_tolerance': 0.05,
        'manager.planning_horizon': float(planning_horizon),
        'manager.use_distinctive_trajs': bool(use_distinctive_trajs),
        'manager.drone_id': int(drone_id),
    })

    # -------------------------- trajectory optimization -----------------------
    parameters.update({
        'optimization.constrain_points_perPiece': 3,  # int
        'optimization.max_vel': float(max_vel),
        'optimization.max_acc': float(max_acc),
        'optimization.record_opt': True,
    })

    # ROS1 '~topic' 私有名 -> ROS2 相对名；目标话题名保持不变
    remappings = [
        ('odom_world', '/drone_{}_{}'.format(drone_id, odometry_topic)),
        ('planning/trajectory', '/drone_{}_planning/trajectory'.format(drone_id)),
        ('planning/start', '/drone_{}_planning/start'.format(drone_id)),
        ('planning/finish', '/drone_{}_planning/finish'.format(drone_id)),
        ('planning/data_display', '/drone_{}_planning/data_display'.format(drone_id)),
        ('planning/broadcast_traj_send', '/broadcast_traj_from_planner'),
        ('planning/broadcast_traj_recv', '/broadcast_traj_to_planner'),
        ('grid_map/odom', '/drone_{}_{}'.format(drone_id, odometry_topic)),
        ('grid_map/cloud', '/drone_{}_{}'.format(drone_id, cloud_topic)),
        ('grid_map/pose', '/drone_{}_{}'.format(drone_id, camera_pose_topic)),
        ('grid_map/depth', '/drone_{}_{}'.format(drone_id, depth_topic)),
    ]

    return {
        'package': 'ego_planner',
        'executable': 'ego_planner_node',
        'name': 'drone_{}_ego_planner_node'.format(drone_id),
        # ROS1 相对话题（goal_point/optimal_list/grid_map/esdf...）经节点名得到
        # /drone_X_ego_planner_node/ 前缀；ROS2 __node 重命名不改命名空间，
        # 必须显式 namespace 还原（否则 7 机可视化话题全部落在根下互相覆盖）。
        'namespace': 'drone_{}_ego_planner_node'.format(drone_id),
        'output': 'screen',
        'remappings': remappings,
        'parameters': [parameters],
    }
