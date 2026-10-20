# Swarm-Formation ROS1 → ROS2 Humble 迁移实施计划

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 将 ROS1 Noetic catkin 工程完整迁移到 ROS2 Humble colcon 工程，demo 功能（7 机正六边形编队）行为不变。

**Architecture:** 仓库根目录变为 colcon workspace；14 个包转 ament_cmake/rosidl；包名、节点名、话题名、参数默认值全部保持不变（唯一例外：参数名中的 `/` 改为 `.`，因 ROS2 参数名不允许斜杠）。设计文档：`docs/plans/2026-10-20-ros2-migration-design.md`。

**Tech Stack:** ROS2 Humble, rclcpp, ament_cmake, rosidl, PCL, Eigen3, cv_bridge, message_filters, image_transport, tf2, armadillo, Python launch。

**验证方式说明:** 本项目为研究代码库，无单元测试框架。本迁移是机械式 API 替换，每个任务的验证 = 该包 `colcon build` 通过 + 最终运行时冒烟测试（Task 15）。不引入新的测试框架（YAGNI）。

---

## 通用转换规则表（每个任务都适用，DRY）

### 头文件映射

| ROS1 | ROS2 |
|---|---|
| `ros/ros.h` | `rclcpp/rclcpp.hpp` |
| `nav_msgs/Odometry.h` | `nav_msgs/msg/odometry.hpp` |
| `geometry_msgs/PoseStamped.h` | `geometry_msgs/msg/pose_stamped.hpp` |
| `sensor_msgs/Image.h`、`PointCloud2.h` | `sensor_msgs/msg/image.hpp`、`point_cloud2.hpp` |
| `visualization_msgs/Marker.h`、`MarkerArray.h` | `visualization_msgs/msg/marker.hpp`、`marker_array.hpp` |
| `quadrotor_msgs/PositionCommand.h` | `quadrotor_msgs/msg/position_command.hpp` |
| `traj_utils/PolyTraj.h` | `traj_utils/msg/poly_traj.hpp` |
| `tf/transform_broadcaster.h` | `tf2_ros/transform_broadcaster.h` |
| `tf/transform_listener.h` | `tf2_ros/transform_listener.h` |
| `cv_bridge/cv_bridge.h` | `cv_bridge/cv_bridge.hpp` |

### API 映射

| ROS1 | ROS2 |
|---|---|
| `ros::init(argc, argv, "name")` | `rclcpp::init(argc, argv)` |
| `ros::NodeHandle nh("~")` | 节点类内 `auto node = rclcpp::Node::make_shared("name");`（原私有句柄语义 → 节点本身；原全局句柄 `nh` 话题 → `node` + 前导 `/` 的话题名不变） |
| `ros::Publisher pub = nh.advertise<T>(topic, n)` | `auto pub = node->create_publisher<T>(topic, n);` |
| `ros::Subscriber sub = nh.subscribe<T>(topic, n, &Cb, this)` | `auto sub = node->create_subscription<T>(topic, n, std::bind(&Cb, this, _1));` 或 lambda。**回调签名**：`const T::ConstPtr&` → `const T::SharedPtr` 或 `const T::SharedPtr&`；ROS1 的非 const `T::Ptr`（如 `PoseStampedPtr`）→ `const T::ConstSharedPtr` |
| `nh.param<double>("a/b", v, def)` | `v = node->declare_parameter<double>("a.b", def); node->get_parameter("a.b", v);`——**参数名 `/`→`.`**。用下方 helper 片段 |
| `ros::Rate rate(hz)` / `rate.sleep()` | `rclcpp::Rate rate(hz)` / `rate.sleep()` |
| `ros::spinOnce()` | `rclcpp::spin_some(node);` |
| `ros::ok()` | `rclcpp::ok()` |
| `ros::Time::now()` | `node->now()` |
| `t.toSec()` | `t.seconds()`（builtin_interfaces::Time 上需 `rclcpp::Time(t).seconds()`） |
| `ros::Duration(d).toSec()`/`.sleep()` | 直接用 double / `rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(d)))` |
| `ros::WallDuration(x).sleep()` | `rclcpp::sleep_for(...)` 同上 |
| `msg->header.stamp = ros::Time::now()` | `msg->header.stamp = node->now();` |
| `ROS_INFO/WARN/ERROR(...)` | `RCLCPP_INFO(node->get_logger(), ...)`, printf 不动 |
| `ros::shutdown()` | `rclcpp::shutdown()` |
| `tf::Quaternion/Matrix3x3/Vector3` | `tf2::Quaternion/Matrix3x3/Vector3`（API 基本同名） |
| `tf::poseTFToMsg / poseToTFMsg` | `tf2::toMsg(tf2_pose)` / 反向手写（见 pose_utils 任务） |
| `cv_bridge::toCvCopy(img, enc)` | 同名（Humble cv_bridge），类型换 `sensor_msgs::msg::Image::ConstSharedPtr` |
| `cv_bridge::CvImage` + `toImageMsg()` | 同名 API 存在；`out_msg.toImageMsg(msg_ptr)` |
| latch publisher | `rclcpp::QoS(n).transient_local()` |

### 参数 helper（每个节点 cpp 顶部内联一份，不改共享库结构）

```cpp
template <typename T>
T get_param(rclcpp::Node::SharedPtr node, const std::string& name, const T& def) {
  T val = def;
  if (!node->has_parameter(name)) node->declare_parameter<T>(name, def);
  node->get_parameter(name, val);
  return val;
}
```

### message_filters（grid_map； pcl_render_node 的 CUDA 变体不改）

```cpp
// ROS1: message_filters::Subscriber<Image> sub_(nh, topic, 50);
message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_(node, "grid_map/depth", rmw_qos_profile_sensor_data);
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry> SyncPolicyImageOdom;
sync_image_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImageOdom>>(
    SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_);
sync_image_odom_->registerCallback(std::bind(&GridMap::depthOdomCallback, this, _1, _2));
```
注意 ROS2 版 `message_filters::Subscriber` 构造第一参为 node，`ros::TransportHints().tcpNoDelay()` 无对应（QoS 已覆盖）。

### CMake / package.xml 模板

CMakeLists.txt 骨架（ament_cmake）：

```cmake
cmake_minimum_required(VERSION 3.8)
project(<pkg>)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# ... 其余 find_package(ros dep REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_executable(<node> src/<node>.cpp)
ament_target_dependencies(<node> rclcpp <ros deps...>)
target_include_directories(<node> PRIVATE include ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(<node> ${PCL_LIBRARIES})
install(TARGETS <node> DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
```

package.xml 骨架（format 3）：`<buildtool_depend>ament_cmake</buildtool_depend>` + `<depend>rclcpp</depend>` 等原 catkin 依赖一一替换，注意加 `<member_of_group>rosidl_interface_packages</member_of_group>`（仅 msg 包）。

---

### Task 0: 工作区重构 + 移除弃用包

**Files:**
- Delete: `.catkin_workspace`, `src/CMakeLists.txt`
- Delete: `src/uav_simulator/so3_quadrotor_simulator/`, `src/uav_simulator/so3_control/`, `src/uav_simulator/mockamap/`, `src/Utils/rviz_plugins/`
- Delete: `src/uav_simulator/local_sensing/src/euroc.cpp`, `src/uav_simulator/local_sensing/src/empty.cpp`（demo 未用节点）。保留 `src/uav_simulator/local_sensing/src/pointcloud_render_node.cpp`——`ENABLE_CUDA=false` 默认分支的 pcl_render_node 源（`pcl_render_node.cpp` 为 CUDA 变体）

**Step 1:** `git rm -r` 上述文件/目录
**Step 2:** 确认 `ls src/planner src/uav_simulator src/Utils` 只剩 14 包
**Step 3:** Commit: `chore: remove catkin toplevel and deprecated packages for ROS2 migration`

### Task 1: quadrotor_msgs（rosidl 接口包）

**Files:**
- Rewrite: `src/Utils/quadrotor_msgs/package.xml`, `src/Utils/quadrotor_msgs/CMakeLists.txt`
- Keep: `src/Utils/quadrotor_msgs/msg/*.msg`（实际 13 个。已完成（6014c9e）：9 个文件的 `Header`→`std_msgs/Header`，Gains.msg `Kp/Kd/Kp_yaw/Kd_yaw`→小写，SO3Command.msg `kR/kOm`→`kr/kom`——rosidl 强制，无消费者受影响；其余内容不动）
- Delete: `src/Utils/quadrotor_msgs/src/`（ROS1 生成物胶水）

**Step 1:** package.xml → format 3, buildtool=ament_cmake, depend=rosidl_default_generators/std_msgs/geometry_msgs, exec_depend=rosidl_default_runtime, member_of_group rosidl_interface_packages
**Step 2:** CMakeLists.txt → `rosidl_generate_interfaces(${PROJECT_NAME} "msg/AuxCommand.msg" ... DEPENDENCIES std_msgs geometry_msgs nav_msgs)` + `ament_export_dependencies(rosidl_default_runtime)`
**Step 3:** 验证：`source /opt/ros/humble/setup.bash && colcon build --packages-select quadrotor_msgs` → `Summary: 1 package finished`
**Step 4:** Commit: `feat(ros2): convert quadrotor_msgs to rosidl interface package`

### Task 2: uav_utils + pose_utils（header-only）

**Files:**
- Rewrite: 两个包的 `package.xml`/`CMakeLists.txt`（ament `INTERFACE` 库：`add_library(${PROJECT_NAME} INTERFACE); target_include_directories(${PROJECT_NAME} INTERFACE include); ament_export_targets(); ament_export_dependencies(...)`）
- Modify: `src/Utils/pose_utils/include/pose_utils.h`——`tf/...` 头 → `tf2/LinearMath/Quaternion.hpp`、`tf2/LinearMath/Matrix3x3.hpp`、`tf2/LinearMath/Transform.hpp`，`tf::` → `tf2::`；`geometry_msgs/... .h` → `.hpp`；`tf::poseTFToMsg/tf2::toMsg` 适配
- Modify: `src/Utils/uav_utils/include/uav_utils/{utils,converters,geometry_utils}.h`——同理替换 tf/cv_bridge/msg 头

**Step 1-3:** 转换 → `colcon build --packages-select pose_utils uav_utils` → Commit `feat(ros2): port pose_utils and uav_utils headers to tf2`

### Task 3: traj_utils（msgs + planning_visualization）

**Files:**
- Rewrite: `package.xml`、`CMakeLists.txt`（rosidl_generate_interfaces 3 个 msg：Assignment/DataDisp/PolyTraj。**已完成的 rosidl 强制偏差**：DataDisp.msg `Header header`→`std_msgs/Header header`；PolyTraj.msg `time start_time`→`builtin_interfaces/Time start_time`——Humble rosidl_adapter **无** 裸 `time` 原语（后续发行版才加入），字段名不变，下游 C++ 为 `builtin_interfaces::msg::Time`（`.sec/.nanosec`，或 `rclcpp::Time(msg.start_time)`）；Assignment.msg 不动。C++ 库目标由 catkin 的 `traj_utils` 改名 `planning_visualization`——rosidl_generate_interfaces 占用了裸包名 target（`add_custom_target(traj_utils ALL)`），下游 plan_manage 链接 `planning_visualization`。同包消费生成消息用 Humble 等价写法：`rosidl_get_typesupport_target(var ${PROJECT_NAME} rosidl_typesupport_cpp)` + `target_link_libraries`（新发行版的 `${rosidl_generate_interfaces_targets}` 变量在 Humble 不存在））
- Modify: `src/planner/traj_utils/src/planning_visualization.cpp` + 对应头——按通用规则表转换（rviz Marker、Amos? 无；`ros::waitForMessage`? 实际不存在，无需适配）

**Step:** 转换 → `colcon build --packages-select traj_utils` → Commit `feat(ros2): port traj_utils messages and planning_visualization`

### Task 4: swarm_graph

**Files:** `package.xml`、`CMakeLists.txt`、`include/swarm_graph/*`、`src/swarm_graph.cpp`——大概率纯 Eigen 无 ROS API；仅需构建系统转换。
**Step:** 转换 → build → Commit `feat(ros2): port swarm_graph build system`

### Task 5: plan_env（grid_map，风险最高）

**Files:**
- Rewrite: `package.xml`、`CMakeLists.txt`（+message_filters, cv_bridge, pcl_conversions, visualization_msgs, tf2 依赖）
- Modify: `include/plan_env/grid_map.h`、`src/grid_map.cpp`

**要点（已扫描确认）:**
- `grid_map.cpp:120-137`：三组 message_filters 同步（image+pose exact / image+odom approx / cloud+odom）→ 按通用规则表模板改写
- `cv_bridge::toCvCopy`（759、1066 行）→ Humble 同名 API
- `grid_map.cpp` 内 `node_` 原为 `ros::NodeHandle`，改为 `rclcpp::Node::SharedPtr` 成员；`GridMap::setParam` 里的 `node_.param` 全部走 helper（参数名 `/`→`.`）
- `origin_;` 等 Eigen 逻辑不动；` visualization` 发布 Marker 不动
- 若有 `ros::console` 或 `ROS_` 宏 → RCLCPP

**Step:** 转换 → `colcon build --packages-select plan_env` → 修编译错至通过 → Commit `feat(ros2): port plan_env grid_map to rclcpp + message_filters2`

### Task 6: path_searching

**Files:** `package.xml`、`CMakeLists.txt`、`include/path_searching/*`（astar.h、topo_prm.h 等）、`src/*.cpp`
**要点:** 头文件若含 `ros/ros.h` 仅为日志/时间，按规则替换；算法不动。
**Step:** 转换 → build → Commit `feat(ros2): port path_searching`

### Task 7: traj_opt

**Files:** `package.xml`、`CMakeLists.txt`、`include/traj_opt/*`（disp_traj_opt 等）、`src/*.cpp`、捆绑的 LBFGS 头不动
**要点:** 大概率纯 Eigen + 日志宏替换；`ros::Time` 计时若用 `<chrono>` 更简（允许，日志/计时非功能）。
**Step:** 转换 → build → Commit `feat(ros2): port traj_opt`

### Task 8: plan_manage（ego_planner 主包）

**Files:**
- Rewrite: `package.xml`、`CMakeLists.txt`
- Modify: `src/ego_planner_node.cpp`（22 行主入口）、`src/ego_replan_fsm.cpp`（837 行）、`src/planner_manager.cpp`、`src/traj_server.cpp`（343 行）

**要点（已扫描确认）:**
- `ego_replan_fsm.cpp:90` `waypoint_sub_ = nh.subscribe("/goal", ...)`——话题名 `/goal` 保持（rviz2 2D Goal Pose 将 remap 到它）
- `:385 waypointCallback(PoseStampedPtr)` 非 const Ptr → `PoseStamped::ConstSharedPtr`
- FSM 中的 `ros::Duration/Time`、timer、`nh.param`（`fsm/waypoint*_x`、`global_goal/relative_pos_*` → `.` 形式）按规则表
- `traj_server.cpp:318` `pos_cmd_pub` 等 pub/sub 转换；`quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY` → `quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY`（常量生成在 C++ 类里，名字不变）
- `ego_planner_node.cpp` 的 main：`ros::init/spin` → `rclcpp::init/spin(node)`

**Step:** 转换 → `colcon build --packages-select plan_manage` → Commit `feat(ros2): port plan_manage (ego_planner nodes) to rclcpp`

### Task 9: map_generator

**Files:** `package.xml`、`CMakeLists.txt`、`src/random_forest_sensing.cpp`
**要点:** pub/sub + params（`map/x_size` → `map.x_size` 等）+ `ros::Rate` 循环。pcl_conversions 发布 PointCloud2。
**Step:** 转换 → build → Commit `feat(ros2): port map_generator`

### Task 10: fake_drone（poscmd_2_odom）

**Files:** `package.xml`、`CMakeLists.txt`、`src/poscmd_2_odom.cpp`（105 行）
**要点:** `quadrotor_msgs::PositionCommand` → `quadrotor_msgs::msg::PositionCommand`；订阅 `~command` → 私有话题 `command`（ROS2 无 `~`，节点名已含 drone_id，话题相对名一致）；nav_msgs Odometry 发布。
**Step:** 转换 → build → Commit `feat(ros2): port fake_drone poscmd_2_odom`

### Task 11: odom_visualization

**Files:** `package.xml`、`CMakeLists.txt`、`src/odom_visualization.cpp`（483 行）
**要点:** `tf/transform_broadcaster.h` → `tf2_ros`；armadillo 系统依赖不动；`pose_utils.h`（Task 2 已转）继续用；Marker/Path 发布转换；`~odom` 等私有话题处理同上。
**Step:** 转换 → build → Commit `feat(ros2): port odom_visualization`

### Task 12: local_sensing_node（pcl_render_node）

**Files:**
- Rewrite: `package.xml`、`CMakeLists.txt`
- Modify: `src/pointcloud_render_node.cpp`（默认非 CUDA 路径的 pcl_render_node 源；CUDA 分支的 pcl_render_node.cpp 保留不动）

**要点（已扫描确认）:**
- 迁移对象 `pointcloud_render_node.cpp` **不含** message_filters/image_transport/dynamic_reconfigure/cv_bridge——这些均为 CUDA 变体 `pcl_render_node.cpp` 的内容（`:6-13`/`:34`/`:300/:312`），该文件保留不动、默认不编译；`cfg/local_sensing_node.cfg` 与 camera.yaml（cam_fx 等）仅 CUDA 路径使用，非 CUDA 节点不读取
- `:159-201` main：`nh.getParam`（sensing_horizon/sensing_rate/estimation_rate、`map/x_size` 等 → `.` 形式）走 helper；PointCloud2/Odometry pub/sub、`ros::Timer`、`ros::Rate` 循环按规则表
- CUDA 分支（CMake `ENABLE_CUDA false` 默认）结构保留：else 分支（PCL 渲染）为本次迁移对象；CUDA 分支仅保持文件存在，不修改其内部（默认不编译）

**Step:** 转换 → `colcon build --packages-select local_sensing_node` → Commit `feat(ros2): port local_sensing pcl_render_node (pointcloud_render_node.cpp, non-CUDA default)`

### Task 13: swarm_bridge

**Files:** `package.xml`、`CMakeLists.txt`、`src/bridge_node.cpp`（270 行）
**要点:**
- UDP socket 代码（POSIX）完全不动：端口 8081、`BUF_LEN`、broadcast 逻辑
- `ros::serialization::serialize/deserialize` → `rclcpp::Serialization<traj_utils::msg::PolyTraj>`：
  ```cpp
  rclcpp::Serialization<traj_utils::msg::PolyTraj> serializer;
  rclcpp::SerializedMessage smsg;
  serializer.serialize_message(msg, &smsg);
  memcpy(udp_send_buf_, smsg.get_rcl_serialized_message().buffer, smsg.size());
  // 反向：SerializedMessage smsg(...); memcpy(...); serializer.deserialize_message(&smsg, msg_out);
  ```
- odom 广播 sub/pub 转换；`boost::thread` 可保留（或 std::thread，允许）

**Step:** 转换 → build → Commit `feat(ros2): port swarm_bridge UDP bridge`

### Task 14: launch 文件 + rviz 配置

**Files:**
- Create: `src/planner/plan_manage/launch/normal_hexagon.launch.py`、`run_in_sim.launch.py`、`rviz.launch.py`、`advanced_param.py`（提供 `generate_advanced_param(...)` 函数返回节点+参数 dict，替代 advanced_param.xml）、`simulator.py`（替代 simulator.xml）；`src/planner/swarm_bridge/launch/bridge.launch.py`
- Modify: `src/planner/plan_manage/launch/default.rviz` → rviz2 格式：`Class: rviz/Marker` → `rviz_default_plugins/Marker`、`rviz/MarkerArray`→`rviz_default_plugins/MarkerArray`、`rviz/Path`→`rviz_default_plugins/Path`、`rviz/PointStamped`→`rviz_default_plugins/PointStamped`、`rviz/Grid`→`rviz_default_plugins/Grid`、`rviz/Axes`→`rviz_default_plugins/Axes`、Panel/Toolbars 段结构更新；Goal3DTool 条目删除，由 rviz2 内置 2D Goal Pose 取代（默认话题 `/goal_pose` remap 到 `/goal`，z 被 FSM 忽略强制 1.0，功能等价）
- Delete: 旧 XML launch 文件

**参数传递规则:** `<param name="map/x_size" value="34"/>` → `"map.x_size": 34.0` 进 `parameters=[...]`；`<remap from="~command" to="drone_0_planning/pos_cmd"/>` → launch 中 `remappings=[("command", "drone_0_planning/pos_cmd")]`（ROS1 私有 `~x` → ROS2 相对名 `x`，配合 remap 保持最终话题名不变）。

**Step:** 编写 → `colcon build`（触发 launch 安装）→ `ros2 launch ego_planner rviz.launch.py --show-args` 干跑校验语法 → Commit `feat(ros2): python launch files and rviz2 config`

### Task 15: 全量构建 + 运行时冒烟验证

**Step 1:** `colcon build` → 14 包全部 `Finished`
**Step 2:** `source install/setup.bash && ros2 launch ego_planner rviz.launch.py`（后台）→ `ros2 node list` 应含 rviz2
**Step 3:** `ros2 launch ego_planner normal_hexagon.launch.py` → `ros2 node list` 应含 7×5+2 个节点（drone_0..6 的 5 节点 + bridge + random_forest）；`ros2 topic list` 应含 `/broadcast_traj_from_planner`、`/map_generator/global_cloud`、`/drone_0_visual_slam/odom` 等
**Step 4:** 模拟点击目标：`ros2 topic pub -1 /goal geometry_msgs/msg/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 20.0, y: 0.0, z: 1.0}}}"` → 观察 `ros2 topic echo /drone_0_visual_slam/odom --once` 多次采样，position.x 应从 -26 递增（无人机在飞）
**Step 5:** 若异常 → 用 superpowers:systematic-debugging 技能定位修复后重跑
**Step 6:** Commit（如有修复）+ 汇报结果

### Task 16: 文档与 Docker 更新

**Files:** Modify `README.md`（Quick Start 改 colcon/ros2 launch 指令）、`Dockerfile`（`osrf/ros:humble-desktop` + colcon）、`Makefile`（catkin_make → colcon build）
**Step:** 更新 → Commit `docs: update build instructions for ROS2 Humble`

---

## 完成标准（最终核对）

- [ ] `colcon build` 14 包全部成功
- [ ] `normal_hexagon.launch.py` 启动后节点/话题齐全
- [ ] 发布 `/goal` 后 7 机编队起飞并向目标移动
- [ ] git log 每任务一提交，无大杂烩提交
