# Swarm-Formation ROS1 → ROS2 (Humble) 迁移设计

日期：2026-10-20
目标：功能不变，只变架构——将 ROS1 Noetic catkin 工程迁移到 ROS2 Humble colcon 工程。

## 目标环境

- Ubuntu 22.04 + ROS2 Humble（本机已装，无 ROS1）
- 仓库根目录即 colcon workspace：删除 `.catkin_workspace`、`src/CMakeLists.txt`（catkin toplevel）
- 构建命令：`colcon build`（取代 `catkin_make`）

## 包清单（14 保留 / 4 移除）

### 保留

| 包 | 类型 | 说明 |
|---|---|---|
| `quadrotor_msgs` | rosidl 接口包 | 15 个 msg 原样迁移；demo 链路用到 `PositionCommand` |
| `uav_utils` | header-only ament 包 | 仅 `odom_visualization` 使用 |
| `pose_utils` | header-only ament 包 | tf 转换工具，适配 tf2 |
| `odom_visualization` | ament_cmake | 里程计可视化 + traj 显示 |
| `map_generator` | ament_cmake | 随机森林全局点云 |
| `fake_drone` | ament_cmake | `poscmd_2_odom` 理想执行器 |
| `local_sensing_node` | ament_cmake | `pcl_render_node`（PCL 路径）；CUDA 分支保留结构、默认关闭（与上游 `ENABLE_CUDA false` 一致） |
| `swarm_bridge` | ament_cmake | UDP 广播桥；`ros::serialization` → `rclcpp::Serialization`（两端同为 ROS2，格式自洽） |
| `plan_env` | ament_cmake | grid map |
| `path_searching` | ament_cmake | A* / 拓扑搜索 |
| `traj_opt` | ament_cmake | 轨迹优化（含 swarm_graph 代价） |
| `traj_utils` | ament_cmake + msg | `Assignment`/`DataDisp`/`PolyTraj` 3 个 msg + planning_visualization |
| `swarm_graph` | ament_cmake (INTERFACE) | 形成图论代价（header-only 库） |
| `plan_manage` (ego_planner) | ament_cmake | ego_planner_node / traj_server / FSM |

### 移除（及其理由）

- `so3_quadrotor_simulator`、`so3_control`、`quadrotor_msgs 之外的 so3 栈依赖`：上游所有 launch 已注释弃用（so3_quadrotor_simulator、so3_control）
- `mockamap`：demo 未使用（上游 simulator.xml 中已注释）
- `rviz_plugins`：`Goal3DTool` 的 z 值被 FSM 丢弃（`waypointCallback` 强制 `end_pt_.z=1.0`），RViz2 内置 **2D Goal Pose** 发布到 `/goal` 功能完全等价

## 架构映射规则

| ROS1 | ROS2 |
|---|---|
| `ros::NodeHandle` / `rosinit` | `rclcpp::Node` / `rclcpp::init` |
| `ros::Rate` while 循环 | `rclcpp::Rate`（保持线程结构不变） |
| `nh.param("a/b", var, def)` | 声明 + get，参数名不变（launch 同名传入） |
| `ros::Publisher::publish`（latch） | `transient_local` QoS（/goal 相关） |
| `tf::` / `tf::TransformBroadcaster` | `tf2` / `tf2_ros::TransformBroadcaster`，转换用 `tf2_eigen` |
| XML launch（include/arg/remap） | Python launch，文件一一对应：`normal_hexagon.launch.py`、`run_in_sim.launch.py`、`rviz.launch.py`、`bridge.launch.py`；`advanced_param.xml`/`simulator.xml` 变为可复用的 launch 函数片段 |
| `default.rviz` | rviz2 配置格式（Display 类型名映射，Goal3DTool → 2D Goal Pose，话题 `/goal`） |
| msg 头文件 `<pkg/Msg.h>` | `<pkg/msg/msg__type_support>` / C++ 生成头 `<pkg/msg/msg.hpp>` |
| `std_msgs/Header` | `std_msgs/msg/header.hpp`（字段名 `stamp` 不变） |

## 不变项（功能等价性保证）

- 所有话题名、节点名、参数名、参数默认值
- 7 机正六边形 demo 拓扑：bridge(999) + random_forest + 每机 5 节点（poscmd_2_odom / odom_visualization / pcl_render_node / ego_planner_node / traj_server）
- UDP 端口 8081、广播地址参数
- 算法代码本体（优化、搜索、图代价）不动，仅替换 ROS API 胶水

## 验证标准

1. `colcon build` 14 包全部编译通过（无 warning-as-error 之外的错误）
2. `ros2 launch ego_planner rviz.launch.py` + `ros2 launch ego_planner normal_hexagon.launch.py`
3. RViz2 中点击 2D Goal Pose → 7 机正六边形编队穿越随机森林到达目标（与 README demo 行为一致）
