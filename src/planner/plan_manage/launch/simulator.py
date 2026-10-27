# ROS2 (Humble) port of launch/simulator.xml (ROS1).
#
# 提供 generate_simulator(...)：返回仿真外围节点（poscmd_2_odom /
# odom_visualization / pcl_render_node）的 Node 构造 kwargs 列表。
# 由 run_in_sim.launch.py 调用。参数名 '~a/b' -> 'a.b'，类型契约保持
# （浮点写浮点，计数类保持 int）。
#
# 原 XML 中另有两个以 CDATA 形式注释掉的备选地图节点（random_forest 传感版 /
# mockamap），以及注释掉的 so3_quadrotor_simulator + so3_control nodelet 组合，
# 保留在文件末尾的注释中备查。

# $(find local_sensing_node)/params/camera.yaml 的 ROS1 <rosparam load> 内容内联如下
# （避免依赖 local_sensing_node 是否安装 params/ 目录；ROS2 版 pcl_render_node
# 已不再读取 cam_* 内参，作为未声明参数静默忽略、无害）。
_CAMERA_PARAMS = {
    'cam_width': 640,      # int
    'cam_height': 480,     # int
    'cam_fx': 387.229248046875,
    'cam_fy': 387.229248046875,
    'cam_cx': 321.04638671875,
    'cam_cy': 243.44969177246094,
}


def generate_simulator(drone_id,
                       init_x_, init_y_, init_z_,
                       map_size_x_, map_size_y_, map_size_z_,
                       odometry_topic,
                       obj_num=1, c_num=30, p_num=30, min_dist=1.0):
    """对应 simulator.xml；返回 Node kwargs dict 的列表。

    obj_num 为原 XML 默认值 1；c_num/p_num/min_dist 在原 XML 中声明但仅在
    注释掉的备选地图节点中引用，默认值任意（此处取 map_generator 的默认量级）。
    """

    # <node pkg="poscmd_2_odom" name="drone_X_poscmd_2_odom" type="poscmd_2_odom">
    poscmd_2_odom = {
        'package': 'poscmd_2_odom',
        'executable': 'poscmd_2_odom',
        'name': 'drone_{}_poscmd_2_odom'.format(drone_id),
        'output': 'screen',
        'parameters': [{
            'init_x': float(init_x_),
            'init_y': float(init_y_),
            'init_z': float(init_z_),
        }],
        'remappings': [
            ('command', 'drone_{}_planning/pos_cmd'.format(drone_id)),
            ('odometry', 'drone_{}_{}'.format(drone_id, odometry_topic)),
        ],
    }

    # <node pkg="odom_visualization" name="drone_X_odom_visualization">
    # 注 1：原 XML 'drone_id' 参数为字符串字面量 "drone_id"，ROS1 因类型不匹配
    # 静默回退默认值 -1；ROS2 端声明为 int，故传等效值 -1。
    # 注 2：ROS1 相对话题（robot/path/...）经节点名自动得到 /drone_X_odom_visualization/
    # 前缀；ROS2 的 __node 重命名不改命名空间，必须显式设 namespace 才能还原该布局，
    # 否则 7 个实例的 robot/path 全部落在根命名空间互相覆盖（rviz 无模型/路径）。
    odom_visualization = {
        'package': 'odom_visualization',
        'executable': 'odom_visualization',
        'name': 'drone_{}_odom_visualization'.format(drone_id),
        'namespace': 'drone_{}_odom_visualization'.format(drone_id),
        'output': 'screen',
        'parameters': [{
            'color.a': 1.0,
            'color.r': 0.0,
            'color.g': 0.0,
            'color.b': 0.0,
            'covariance_scale': 100.0,
            'robot_scale': 1.0,
            'tf45': False,
            'drone_id': -1,
        }],
        'remappings': [
            # 原 XML 硬编码 visual_slam/odom（未用 $(arg odometry_topic)），保持原样。
            # 目标名必须带前导 '/'：namespace 下相对目标会被解析进本节点命名空间。
            ('odom', '/drone_{}_visual_slam/odom'.format(drone_id)),
        ],
    }

    # <node pkg="local_sensing_node" type="pcl_render_node" name="drone_X_pcl_render_node">
    pcl_render_node = {
        'package': 'local_sensing_node',
        'executable': 'pcl_render_node',
        'name': 'drone_{}_pcl_render_node'.format(drone_id),
        'output': 'screen',
        'parameters': [{
            **_CAMERA_PARAMS,
            'sensing_horizon': 5.0,
            'sensing_rate': 30.0,
            'estimation_rate': 30.0,
            'map.x_size': float(map_size_x_),
            'map.y_size': float(map_size_y_),
            'map.z_size': float(map_size_z_),
        }],
        'remappings': [
            ('global_map', '/map_generator/global_cloud'),
            ('odometry', '/drone_{}_{}'.format(drone_id, odometry_topic)),
            ('pcl_render_node/cloud', '/drone_{}_pcl_render_node/cloud'.format(drone_id)),
        ],
    }

    return [poscmd_2_odom, odom_visualization, pcl_render_node]


# ---------------------------------------------------------------------------
# 原 simulator.xml 中注释掉（CDATA）的备选一：random_forest 传感版地图节点
#
#   node pkg="map_generator" name="random_forest" type="random_forest" output="screen"
#     remap ~odometry -> $(arg odometry_topic)
#     map.x_size=$(arg map_size_x_), map.y_size=$(arg map_size_y_),
#     map.z_size=$(arg map_size_z_), map.resolution=0.1,
#     ObstacleShape.seed=1, map.obs_num=$(arg p_num),
#     ObstacleShape.lower_rad=0.5, upper_rad=0.7, lower_hei=0.0, upper_hei=3.0,
#     map.circle_num=$(arg c_num), ObstacleShape.radius_l=0.7, radius_h=0.5,
#     z_l=0.7, z_h=0.8, theta=0.5, sensing.radius=5.0, sensing.rate=10.0,
#     min_distance=$(arg min_dist)
#
# 备选二：mockamap 节点
#
#   node pkg="mockamap" type="mockamap_node" name="mockamap_node" output="screen"
#     remap /mock_map -> /map_generator/global_cloud
#     seed=127(int), update_freq=0.5, resolution=0.1,
#     x_length=$(arg map_size_x_), y_length=$(arg map_size_y_), z_length=$(arg map_size_z_),
#     type=1(int), complexity=0.05, fill=0.12, fractal=1(int), attenuation=0.1
#
# 另有整段注释掉的 so3_quadrotor_simulator / so3_control(nodelet) 组合，
# 详见 git 历史中的 simulator.xml。
# ---------------------------------------------------------------------------
