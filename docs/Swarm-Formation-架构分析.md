# Swarm-Formation 技术架构分析

> 分析对象：ZJU-FAST-Lab/Swarm-Formation（ICRA2022 开源版 v1.1）
> 论文：*Distributed Swarm Trajectory Optimization for Formation Flight in Dense Environments*
> 生成日期：2026-10-21 · v2（全面重组版）

## 目录

1. [项目定位与总体架构](#1-项目定位与总体架构)
2. [感知：传感器与地图](#2-感知传感器与地图)
3. [定位：每机如何知道自己在哪](#3-定位每机如何知道自己在哪)
4. [导航：没有全局地图如何到达目标](#4-导航没有全局地图如何到达目标)
5. [路径规划与轨迹优化](#5-路径规划与轨迹优化)
6. [编队（队列）保持机制](#6-编队队列保持机制)
7. [机间通信与协同](#7-机间通信与协同)
8. [执行与控制链](#8-执行与控制链)
9. [关键参数速查](#9-关键参数速查)
10. [实现细节与陷阱](#10-实现细节与陷阱)
11. [总结](#11-总结)

---

## 1. 项目定位与总体架构

### 1.1 这是什么项目

**分布式集群编队轨迹优化框架**：让一群无人机在稠密障碍环境中**保持编队队形飞向目标**。

技术血统：

- **EGO-Swarm**：局部感知 + 滚动重规划的系统框架
- **GCOPTER/MINCO**：稀疏多项式轨迹参数化 + 解析梯度
- **LBFGS-Lite**：无约束 L-BFGS 求解器

核心创新点：把**图论队形相似性**（对称归一化拉普拉斯矩阵的谱距离）做成可微代价项，嵌入时空联合优化，使"编队保持"与"避障、互避、动力学可行"在同一个优化问题里权衡。

**完全分布式**：每架机只依赖自身局部感知 + 邻居广播的轨迹，没有中央节点。

系统全貌（一句话版）：**没有全局地图**——每机靠局部滑窗地图 + 滚动 MINCO 优化导航到目标坐标；**机间不测相对位姿**——靠互播轨迹按时间外推；**编队**靠"队形模板 + 终点摆阵 + 谱相似性软约束 + 顺序启动"多层合成。

### 1.2 整体数据流

```
                        ┌─────────────────────────────────────────────┐
                        │  全局: map_generator (随机森林点云)            │
                        │        swarm_bridge (UDP 8081 轨迹广播)       │
                        │        rviz                                  │
                        └──────┬──────────────────────────▲────────────┘
                    /map_generator/global_cloud           │ /broadcast_traj_from/to_planner
                               ↓ (每机一份)                │
  ┌────────────────── 每架无人机 drone_N 内部 ─────────────┴──────────────┐
  │                                                                     │
  │  pcl_render_node ──cloud(局部点云/深度)──▶ GridMap(占据+膨胀+ESDF)     │
  │        ▲ odom                                                       │
  │        │                                                            │
  │  ego_planner_node: FSM(100Hz) → A*初值 → MINCO+LBFGS优化 ──PolyTraj──┼──▶ 邻居
  │        │                          (代价: 平滑+障碍+互避+编队+可行性)    │
  │        ▼ PolyTraj                                                  │
  │  traj_server(100Hz采样) ──PositionCommand──▶ poscmd_2_odom(fake_drone)│
  │                                              或 so3_control→四旋翼ODE │
  │        └──────────── odom 闭环 ◀──────────────────────────┘           │
  └──────────────────────────────────────────────────────────────────────┘
```

### 1.3 进程/节点组织：每机一份 vs 全局一份

不使用 ROS namespace，全部靠 `drone_N_` 节点名与话题名前缀隔离（`normal_hexagon.launch` 以 drone_id=0~6 七次 include `run_in_sim.launch`）。

**每机 5 个节点**：

| 节点 | 职责 |
|---|---|
| **pcl_render_node** | 传感器模拟：里程计 + FOV → 从全局点云裁出局部观测 |
| **ego_planner_node** | **大脑，单进程内含四件事**：① GridMap 建图（点云→占据/膨胀/ESDF）；② FSM 状态机（100Hz 决策何时重规划）；③ EGOPlannerManager（局部目标、A\*、rebound 重规划）；④ PolyTrajOptimizer（MINCO+LBFGS，含编队图代价）。同时维护邻居轨迹表 `swarm_traj[]` 供互避/编队/顺序启动用 |
| **traj_server** | 轨迹执行器：多项式 → 100Hz PositionCommand（解析微分前馈 + yaw 生成），详见 §8.1 |
| **poscmd_2_odom**（fake_drone） | 理想机体：把位置指令直接当真实状态回吐，详见 §8.2 |
| **odom_visualization** | mesh 机体/路径/速度箭头等 rviz 显示，详见 §8.3 |

**全局仅一份**：map_generator（全局点云）、swarm_bridge（UDP 桥）、rviz。

### 1.4 每机对外接口（输入/输出速查）

**输入**（`plan_manage/src/ego_replan_fsm.cpp:72-119`、`plan_env/src/grid_map.cpp:120-146`）：

| 输入 | 类型 | 来源 |
|---|---|---|
| `odom_world` / `grid_map/odom` | `nav_msgs/Odometry` | poscmd_2_odom（仿真）或 VINS（实机） |
| `grid_map/cloud`（或 depth+pose） | `PointCloud2` / `Image` | pcl_render_node（模拟机载传感器） |
| `planning/broadcast_traj_recv` | `traj_utils/PolyTraj` | swarm_bridge UDP 广播的邻居轨迹 |
| `/move_base_simple/goal`（flight_type=3） | `PoseStamped` | RViz "2D Nav Goal"，编队中心目标点 |

**输出**：

| 输出 | 类型 | 去向 |
|---|---|---|
| `planning/trajectory` | `PolyTraj`（5 阶多项式系数 + 时长） | 本机 traj_server |
| `planning/broadcast_traj_send` | `PolyTraj` | swarm_bridge → UDP → 所有邻居 |
| `planning/pos_cmd` | `quadrotor_msgs/PositionCommand`（pos/vel/acc/jerk + yaw） | 控制器/仿真机体 |
| `grid_map/occupancy(_inflate)`、marker | `PointCloud2` / `Marker` | rviz 可视化 |
| `planning/start` / `finish`、`data_display` | `Bool` / `DataDisp` | 计时与调试 |

`PolyTraj.msg` 是多机协同核心载体：`drone_id + traj_id + start_time + order==5 + coef_x/y/z + duration`。跨机时钟校验硬约束 `|now − start_time| ≤ 0.25s`（`ego_replan_fsm.cpp:471`）。

### 1.5 坐标系约定：只有一层 world 系

**右手系，z 轴向上（重力反向），地面 z=0，原点在配置地图区域的几何中心**：

```
     z ↑ (虚拟天花板 3.0m)
       │
       │   ✈ 飞行高度 0.5m（init_z = target_z = 编队中心 z，全部统一）
       │
       └───────────────────────────→ x（主飞行方向：西 → 东）
  (-35,-15) 0,0 (35,-15)
   ┌───────────────────────────────┐
   │ 起飞区(西)         目标区(东)   │
   │ x≈-28.6~-23.4     x≈23~29     │ y（南北 ±15）
   └───────────────────────────────┘
   地面 z = ground_height = -0.01，规划地图桶 70×30×3m
```

- 原点计算：`map_origin_ = (-x_size/2, -y_size/2, ground_height)`（`grid_map.cpp:62`）；所有消息 `frame_id="world"`；规划、建图、轨迹、目标、编队图全部直接用 world 坐标，规划器内部几乎不做坐标变换。
- **与经典导航栈最大的不同：没有 map→odom→base 三层 TF**。经典 move_base 系有 map frame（全局定位）+ odom frame（局部漂移系）+ base_link 三层 TF 树；本系统仿真中 odom 直接就是 world 系真值（§3.1），"odom = world"，规划器眼里只有一层坐标。
- **机体系/相机系只出现在边缘位置**：动力学仿真内部（so3_control 推力方向 b3、`cam02body` 深度相机外参，CUDA 通路）、可视化 TF（`world → base(drone_id)`，`odom_visualization.cpp:362-393`）。默认点云通路连相机系都没有——点云从全局点云裁剪，本来就是 world 系。
- **两个"隐藏坐标系"**：① 栅格索引系——world 连续坐标 ↔ 体素 id 的仿射映射 `(pos − origin)/resolution`，A\* 在此离散栅格上跑；② 队形没有坐标系——谱代价定义在"成对距离空间"里（平移旋转不变的根源，§6.2）。
- **朝向细节**：终点派生的六边形模板顶点是**固定朝向**的（边平行 x/y 轴，`poly_traj_optimizer.h:205-219`）——到达目标时队形朝向固定（边始终平行东西-南北），不会随飞行方向旋转；但飞行途中谱约束允许队形整体旋转绕障——"飞着转、到点摆正"。yaw 约定从 x 轴起算。
- **实机提醒**：以上一切建立在"所有机共享同一个 world 原点"上——仿真天然成立；实机上各机 VINS 的 odom 系原点是各自起飞点，需要外部对齐手段重建这层前提（§3.3）。

---

## 2. 感知：传感器与地图

### 2.1 传感器形态

| 层面 | 传感器 | 说明 |
|---|---|---|
| **当前默认编译** | **局部 3D 点云**（CPU 版 pcl_render_node） | `src/uav_simulator/local_sensing/CMakeLists.txt:9` 设 `ENABLE_CUDA false`，编译 `pointcloud_render_node.cpp`：全局点云 0.1m 体素降采样建 KdTree → `radiusSearch(5m)` + FOV 过滤（垂直 ±30°、水平 ±60° 半角），约 12Hz（`pointcloud_render_node.cpp:98-152`） |
| CUDA 开启时 | **深度相机**（640×480, fx=fy=387.2） | GPU z-buffer 渲染深度图 + camera_pose（`params/camera.yaml`）；grid_map 侧深度通路（反投影 + DDA raycasting + log-odds 融合）代码完整但默认无数据源 |
| grid_map 备用 | 激光点云直融 | `grid_map/cloud` 话题（实机可接激光），每帧 `resetBuffer` 清局部窗后直接写膨胀占据层，**无 raycast free-space 清理**（`plan_env/src/grid_map.cpp:803-913`） |
| 里程计 | 话题名 `visual_slam/odom` | 实机对应 VINS 类视觉里程计（见 §3） |

关键设计：仿真中"传感器"是从上帝视角全局点云渲染出的**机载局部感知数据**，刻意限制视场角和感知半径（5m），让规划器只看得见局部——这是滚动重规划的前提。

### 2.2 全局地图：是素材，不是地图

`map_generator` 发布的 `/map_generator/global_cloud`（随机柱状森林 + 圆环，`random_forest_sensing.cpp:184-306`）**对规划零参与**：

- 唯二的消费者是每机的 `pcl_render_node`（拿它渲染局部传感器数据）和 rviz 显示；
- 规划器不订阅它（`advanced_param.xml:63` 的直连 remap 被注释）；
- 存在意义：把全局环境知识"降解"为每机的局部观测。实机部署时不存在这个节点，传感器数据直接来自真实的深度相机/激光。

备选全局地图源 `mockamap`（Perlin 3D 噪声 / 随机方块 / 2D 迷宫 / 3D 迷宫四种类型，输出同为点云，`mockamap/src/maps.cpp:696-712`）代码在仓库内，是 map_generator 的替代品，但主仿真中同样被禁用（见 §10 陷阱 4）。

### 2.3 每机局部地图：唯一参与规划的地图

GridMap 嵌在每机 ego_planner_node 进程内，三层结构：

1. **占据层**：点云直融（默认）或深度图 raycasting + log-odds（CUDA 路径）
2. **膨胀层**：立方体膨胀 0.1m，安全裕度，给 A\*/急停检查用（`grid_map.cpp:553-683`）
3. **ESDF**：Felzenszwalb 距离变换（轴分离 1D），给优化器提供连续可微的障碍距离和梯度（`grid_map.cpp:1114-1214`）

要点：

- 分配整张 70×30×3m / 0.1m 的"桶"，但只在 `odom ± 5.5m` 滑窗内更新——"全局尺寸的桶、局部滑窗的内容"
- 虚拟天花板限高 3m（`grid_map.cpp:673-682`）
- **机间不共享地图**：详见下述"机间地图关系"

**机间地图关系：各管各的，永不共享**

每机地图物理隔离：GridMap 活在各自 `drone_N_ego_planner_node` 进程内（`planner_manager.cpp:31-36`），发布的 `grid_map/occupancy(_inflate)`、`esdf` 全是私有话题（只给本机 rviz 看）。全仓库没有任何跨机地图话题——唯二的全局共享话题是 `/broadcast_traj_from/to_planner`，交换的是**轨迹**（几百字节多项式系数），不是地图。

不共享的理由：

- **带宽**：0.1m 分辨率下整桶 630 万体素，即便只传局部滑窗也是 MB 级/帧；一条 PolyTraj 只有几百字节，广播毫无压力；
- **没必要**：避障本来就是"各顾各"——只需绕开自己看得见的障碍；机间安全靠交换轨迹互推未来位置（§7.2）；
- **去中心化**：共享地图就需要地图融合/一致性协议，破坏分布式架构。

两个容易忽略的推论：

1. **队友不在你的占据地图里**：pcl_render_node 从静态全局点云渲染局部观测（点云里只有障碍物，没有队友），所以其他无人机飞得再近，你的占据/ESDF 地图上也"看不见"它——**机间互避 100% 依赖轨迹广播**（swarm 代价项里的邻居位置全部来自轨迹外推），地图对此零贡献。
2. **轨迹广播失效 = 互避失明**：邻居轨迹 `start_time` 偏差 >0.25s 会被直接丢弃，之后邻居位置只能靠旧轨迹末端速度线性外推（`poly_traj_optimizer.cpp:419-433`），越外推越不准——这是实机上时钟同步成为硬前提的根本原因。

### 2.4 局部地图是滑窗，不累积成大地图

虽然分配了 70×30×3m 的大数组，但内容是**遗忘式**的：深度通路把超出 `odom ± 5.5m` 的缓存体素置回 unknown（`grid_map.cpp:514-519`）；点云通路（默认）每帧 `resetBuffer` 把机体周围清零重写（`grid_map.cpp:823-824`）。飞过的区域不会留底——不是 SLAM 意义上"随导航逐渐建成全局地图"，而是跟着机体走的 5.5m 半径感知窗口。

---

## 3. 定位：每机如何知道自己在哪

分两层：**仿真里没有"定位算法"（真值直读），实机上靠每机独立的视觉惯性里程计（VINS 系）**。

### 3.1 仿真：真值直读，不存在定位问题

- **默认 fake_drone 链（poscmd_2_odom）**：里程计 ≡ 位置指令本身——`pose.position ← cmd.position; twist.linear ← cmd.velocity`（`fake_drone/src/poscmd_2_odom.cpp:31-33`），姿态由指令加速度解析构造。所谓"定位"就是把控制器想要的状态当作真实状态回吐，100Hz 发布到 `drone_N_visual_slam/odom`。
- **完整动力学链（so3_quadrotor_simulator，默认注释禁用）**：1000Hz odeint 积分 22 维状态 `[位置 速度 旋转 ω 电机转速]`，`stateToOdomMsg` 把**真值状态**直接填进 `nav_msgs/Odometry`，按 `rate/odom`（100~200Hz）发布（`quadrotor_simulator_so3.cpp:305-326`），无噪声注入——相当于完美传感器。

仿真里 7 架机都精确知道自己在同一个 `world` 坐标系里的位置，这是编队协同成立的前提。

### 3.2 实机：每机独立跑 VIO（VINS 系）

仓库不含定位实现，但接口证据明确指向每架机独立运行 VINS-Fusion/VINS-Mono 类视觉惯性里程计：

| 证据 | 位置 |
|---|---|
| odom 话题名就叫 `visual_slam/odom`，作为 `odometry_topic` 贯穿所有 launch | `normal_hexagon.launch:6`、`run_in_sim.launch` |
| 实机 bridge 配置把 `~my_odom` remap 到 `/vins_estimator/imu_propagate` | `swarm_bridge/launch/bridge.launch:8` |
| grid_map 预留 `/vins_estimator/extrinsic` 订阅，用于在线更新相机-IMU 外参 | `plan_env/src/grid_map.cpp:120` 附近 |

`imu_propagate` 是 VINS 的高频 IMU 传播状态输出，作为位姿源喂给建图和规划。每架机一套：相机 + IMU → VINS → 世界系（各自初始化的局部系）位姿 → grid_map 建图位姿 / FSM 状态 / pcl 传感器位姿。

注意：VIO 给出的是**以各自启动时刻为原点的局部世界系**位姿，基准是起飞点，存在累积漂移。

### 3.3 多机之间：两件麻烦事，仓库只解决了一件

- **时钟同步（已解决）**：轨迹交换硬校验 `|now − start_time| ≤ 0.25s`（`ego_replan_fsm.cpp:471`），实机部署需要 NTP/chrony 级别对时，否则邻居轨迹直接被丢弃。
- **世界坐标系对齐（不在此仓库内）**：各机 VINS 里程计各有各的原点/朝向，而编队代价与互避默认大家共用一个 `world` 系。bridge 确实会把各机 odom 打上 `child_frame_id="drone_<id>"` 通过 UDP 广播（`bridge_node.cpp:148`），具备做机间相对位姿估计的接口，但 EGO planner 侧根本没订阅 `/others_odom`——对齐模块（起飞点统一摆正、或互视定位）属于实机系统的配套件。仿真中所有节点共享同一 ROS master 与 `world` frame，该问题被天然绕过。

---

## 4. 导航：没有全局地图如何到达目标

### 4.1 目标指定的三种入口

由 `flight_type` 参数决定（本仓库 `run_in_sim.launch:80` 硬编码为 3）：

| flight_type | 方式 | 触发接口 |
|---|---|---|
| 1 `MANUAL_TARGET`（单机） | 发布目标点话题 | `/goal`（`geometry_msgs/PoseStamped`）→ `waypointCallback` |
| 2 `PRESET_TARGET`（预设航线） | launch 参数写死一串航点，顺序飞 | `waypoint{i}_{x,y,z}` 参数 + `/traj_start_trigger` 触发（`ego_replan_fsm.cpp:36-42`） |
| 3 `SWARM_MANUAL_TARGET`（默认） | RViz 点选或程序发布 | `/move_base_simple/goal` → `formationWaypointCallback` |

即"去某个地点" = 发布一个 world 坐标系下的目标点消息。不想用 RViz 手点，直接命令行/程序发：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 20.0, y: 5.0, z: 1.0}}}'
```

收到目标后机内发生什么（`formationWaypointCallback`，`ego_replan_fsm.cpp:631-690`）：

```
目标消息 (x, y)
  │  细节：z 只做有效性检查（z < -0.1 直接丢弃，:633），飞行高度被硬编码——
  │  编队中心 z = 0.5m（:642），目标里的 z 不生效
  ▼
编队中心 swarm_central_pos_ = (msg.x, msg.y, 0.5)
  ▼
每机各自的终点 end_pt_ = 中心 + swarm_scale × 六边形顶点[drone_id]   (:650)
  │  7 架机由同一个目标点自动派生出 7 个不同终点——这就是"去那里"的编队语义
  ▼
planGlobalTrajWaypoints：当前 odom 位置/速度 → end_pt_ 的 min-jerk 参考线
  │  （不避障，只指方向）                                            (:656)
  ▼
FSM 切换：静止中 WAIT_TARGET → SEQUENTIAL_START（按 id 顺序起编队）
         飞行中 EXEC_TRAJ → REPLAN_TRAJ（中途换目标，立即朝新目标重规划，:680-681）
```

三个实用要点：

- **只认坐标，不认"地点"**：系统没有语义层（不存在"去 3 号房间"这种表达），目标必须是 world 系坐标；坐标从哪来（手点、上层任务分配、外部定位系统）它不关心。
- **飞行中随时换目标**：再发一次 `/move_base_simple/goal` 即可，FSM 从当前状态直接切 `REPLAN_TRAJ` 朝新目标去（`:680-681`），无需落地。
- **要去多个地点**：用 flight_type=2 预设航点序列（`planGlobalTrajbyGivenWps` 按序飞，`:691`），或上层程序按时机逐站发布目标（到达后系统回 `WAIT_TARGET`，天然支持"到一站发下一站"）。

**指令边界：发不了什么（全部是启动参数而非运行时接口）**

- ❌ **急停**——没有用户急停话题，EMERGENCY_STOP 只由 FSM 内部触发；想"停"只能发一个当前中心位置的目标点，让编队原地收拢悬停；
- ❌ **换队形/缩放**——`formation_type`、`swarm_scale` 是 yaml 启动参数（改队形见 §6.5）；
- ❌ **调速/调高**——max_vel 等是 launch 参数；
- ❌ **单机点名**——指令永远作用于整个编队，没有"只对某架机说"的通道（槽位静态，§6.3）；
- ❌ 点击加障碍——死代码（`clickCallback` 定义了但无订阅接线，`random_forest_sensing.cpp:342`）。

指令粒度 = 一个目标坐标，剩下全部自主——"给目标不给路径"的设计哲学。

### 4.2 滚动导航闭环

**不需要全局地图，只需要"目标的世界坐标 + 滚动重规划"**。目标点以世界坐标直接给出，定位靠里程计（§3），避障靠滚动窗口：

```
1. 定位:  odom（仿真=完美里程计；实机=VINS 视觉里程计）给出"我在哪"
          —— 不需要全局地图来定位（实机上里程计漂移是真实风险）

2. 目标:  RViz "2D Nav Goal" 给出编队中心的世界坐标
          每机目标 = 中心 + scale × 六边形顶点[id]     (ego_replan_fsm.cpp:650)

3. 全局参考（不避障）:  起点→目标拉一条 min-jerk 多项式曲线
          (planner_manager.cpp:318-367)
          它的作用只是"指方向"，可能直接穿过障碍

4. 滚动循环（导航的真正引擎）:
   ┌─▶ 局部目标: 在参考曲线上取前方 7.5m 处的点（够不到就取终点）
   │            (planner_manager.cpp:138-178)
   │   局部优化: 在"自己感知到的局部地图"里优化出无碰撞轨迹
   │            （只优化前 2/3 段，后 1/3 留给下一轮）
   │   执行:     飞这 2/3 段的同时，新感知不断汇入局部地图
   └── 每 ~1s 或触发条件即重规划，窗口向前滚动

5. 终止:  局部目标与全局目标距离 < 0.1m 且轨迹快飞完 → 到达
          (ego_replan_fsm.cpp:216-224)
```

"导航到目标点"不是一次性求解的全局路径问题，而是**反馈控制式的滚动优化**：飞一段 → 看到新障碍 → 重规划 → 再飞一段。全局参考曲线保证大方向不偏，避障完全由局部地图 + 重规划涌现出来。这也是为什么 `planning_horizon`(7.5m) 必须约为感知半径(5m) 的 1.5 倍——窗口要比看得远，但又不能远到轨迹末端落在未知空间里。

### 4.3 代价：没有全局最优性保证

- 随机森林这类稀疏凸障碍环境里，局部重规划几乎总能滑过去（论文 demo 场景）；
- 理论上存在**局部极小**风险：U 形围墙这类凹形障碍可能让轨迹卡住反复重规划。代码里唯一的自救机制是失败后交替"A* 初始化 / 随机初始化"重试（`flag_relan_astar_`，`ego_replan_fsm.cpp:194-214`），没有 D* Lite、拓扑引导或探索式回退这类全局性手段；
- README 提到的 TRO 改进版（"complete formation navigation system"）补的正是全局导航层，但不在本开源仓库（v1.1，ICRA2022 版）内。

### 4.4 与传统导航栈对照

| | 传统（如 move_base） | 本项目 |
|---|---|---|
| 全局地图 | 有（全局 costmap，随建图累积） | **无** |
| 全局路径规划 | 有（A*/Dijkstra 在全局图上） | **无**（只有一条不避障的 min-jerk 参考线指方向） |
| 局部规划 | 有（DWA/TEB） | 有（滚动 MINCO 优化，7.5m 窗口） |
| 记忆 | 走过的区域保留 | 窗外即遗忘 |
| 定位 | AMCL 等（需全局地图） | 里程计/VIO（不需全局地图） |

换来的是：不需要先验建图、不用维护大地图一致性、天然适合分布式——每架机只对自己 5.5m 内的世界负责。

---

## 5. 路径规划与轨迹优化

### 5.1 分层结构："全局参考 → 滚动局部优化"

本质是优化而非传统前端搜索。前端 = 栅格 A\* + 最小 jerk 多项式初值；后端 = 以"中间航点 + 分段时间"为决策变量的最小 jerk 多项式轨迹（MinJerkOpt，MINCO 式稀疏参数化），用无约束 L-BFGS 求解。

### 5.2 前端：全局参考线、局部目标与 A\*

- **全局参考线（不避障）**：收到目标后由 `planGlobalTrajWaypoints` 生成一条过起终点的 min-jerk 多项式参考轨迹（`planner_manager.cpp:318-367`），按 max_vel 分段反复降速重生成直到满足速度限制。它只负责"指方向"。
- **局部目标提取**：每次重规划在全局轨迹上取距当前位置 ≥ 7.5m（planning_horizon ≈ 1.5× 感知半径）的点作为局部目标；接近终点时末端速度置零以便刹车（`planner_manager.cpp:138-178`）。
- **A\***：`path_searching` 里只有 26 邻域栅格 A\*（`dyn_a_star.cpp`），启发式为对角距离乘 tie-breaker，膨胀占据/ESDF 两种碰撞检查模式，0.2s 超时；后处理含视线捷径、删密点、插中点（`dyn_a_star.cpp:265-393`）。**只在首次规划或优化失败重试时**为优化提供初始航点和时间分配（`poly_traj_optimizer.cpp:623-675`）；常规重规划直接用上一条最优轨迹外推作初值（`planner_manager.cpp:87-133`）。

### 5.3 后端：MINCO 式时空联合优化（核心）

（`traj_opt/src/poly_traj_optimizer.cpp`，入口 `OptimizeTrajectory_lbfgs:7-96`）

- **轨迹表示**：每轴 5 阶（6 系数）多项式分段——这是满足"段内 PVA 连续 + 首尾边界条件"的 min-jerk 参数化的最低阶次；消息端同样强制 `order==5`
- **决策变量**：`[中间航点, 分段虚拟时间]`（时间经平方映射保证为正，`:171-189`）
- **系数求解**：给定航点和时间，带状 LU 解析求出满足首尾 PVA 与中间 p/v/a 连续的系数（`poly_traj_utils.hpp:1143-1250`），梯度回传用伴随法
- **求解器**：L-BFGS（编队模式 max_iter=20 保实时性，无编队 60，`:44-56`）

代价函数（权重见 `plan_manage/config/normal_hexagon.yaml`）：

| 代价项 | 形式 | 权重 |
|---|---|---|
| 平滑 | ∫‖jerk‖²dt（min-jerk 闭式解） | — |
| 障碍 | ESDF 距离三次方惩罚 `(d_clear−d)³`，梯度三线性插值 | 50000 |
| **机间互避** | 椭球度量 `dz²/a²+(dx²+dy²)/b²` vs `swarm_clearance`，用邻居广播轨迹的**预测未来位置** | 50000 |
| **编队相似性** | `‖L̂ − L̂_des‖²_F`（数学形式见 §6.2） | 15000 |
| 动力学可行 | `(|v|²−v_max²)³`、`(|a|²−a_max²)³`（v_max=1.5m/s、a_max=8.0m/s²，与全局轨迹限速共用同一组参数，`run_in_sim.launch:72-73`） | 10000 |
| 均匀性/时间 | 相邻点距方差、`w_time·ΣT` | 10000 / 80 |

优化结束先做最终碰撞检查（0.01s 采样前 2/3 轨迹，`:98-127`），通过才算成功。

### 5.4 约束点离散化技巧

所有空间代价只在每段 K+1=4 个"约束点"上离散评估（`addPVAGradCost2CT`，`:224-374`），且**只对前 2/3 约束点生效**（障碍 `:467`、swarm `:501`、formation `:385`）——后 1/3 留给下次重规划，这是滚动窗口的实现技巧。

### 5.5 FSM 状态机与安全机制

状态：`INIT → WAIT_TARGET → SEQUENTIAL_START / GEN_NEW_TRAJ → EXEC_TRAJ ⇄ REPLAN_TRAJ → EMERGENCY_STOP`（`ego_replan_fsm.h:36-45`）

- **100Hz 主循环**（`execFSMCallback`，`ego_replan_fsm.cpp:123-280`）：EXEC_TRAJ 中按 `t_cur > 1.0s` 或距终点 < 1.0m 触发 REPLAN_TRAJ；接近终点且轨迹快飞完 → 到达，回 WAIT_TARGET 并发布 finish
- **20Hz 安全检查**（`checkCollisionCallback`，`:282-376`）：深度丢失 → EMERGENCY_STOP；沿当前轨迹 0.01s 采样，遇障碍（`getInflateOccupancy`）或与邻居预测位置距离 < 0.8×`swarm_clearance` → 抢救性重规划；碰撞时刻距当前 < 1.0s → EMERGENCY_STOP（生成 2 段各 1s 的停止轨迹）
- **重试机制**：REPLAN_TRAJ 失败置 `flag_relan_astar_=true`，改用 A\* 初始化且不带 formation 项重试一次（`:194-214`）

---

## 6. 编队（队列）保持机制

### 6.1 四层机制

**① 队形定义（两层一致的模板）**：`normal_hexagon.yaml:6-12` 和 `poly_traj_optimizer.h:205-219` 各存一份相同的 7 点模板——**drone 0 在中心 (0,0,0)，drone 1-6 在正六边形顶点**（外接半径 2 × `swarm_scale`=2 → 实际边长 4m）。角色按 index 固定：drone i ↔ 顶点 i，**没有在线角色分配**（匈牙利算法 Munkres 是死代码）。

**② 目标层成队**：终点 = `编队中心 + scale × 顶点[id]`（`ego_replan_fsm.cpp:650`）——7 个终点本身摆成六边形，到达即成队。

**③ 飞行中保持（核心贡献）**：拉普拉斯谱相似性软约束（数学形式见 §6.2）。三个关键性质：

- 邻接矩阵用**成对距离**而非绝对坐标 → 代价对平移、旋转不变 → 编队可以整体转向、平移着绕障碍；
- 权重 15000 < 避障 50000 → 遇窄障碍时**队形允许临时压扁变形，通过后自动恢复**（软约束而非硬约束）；
- 每机用同样的广播信息独立算出同一个图代价 → 分布式一致，无需中央协调。

**④ 顺序启动 + 时间对齐 + 互避兜底**：`SEQUENTIAL_START` 按 id 递增启动（详见 §7.3），保证编队项从第一刻起就有完整的 7 顶点图；成队/飞行过程中，互避代价（`swarm_clearance`=0.5m 椭球惩罚）+ 收到邻居轨迹即时的两两碰撞检查兜底（§7.4），防止队内相撞。

### 6.2 谱相似性代价：数学形式与性质

实现在 `swarm_graph/`（`swarm_graph.cpp`），优化侧入口 `swarmGraphGradCostP`（`poly_traj_optimizer.cpp:376-460`）：

1. **当前队形图的构造**：把自己候选约束点位置 + 各邻居广播轨迹在 `t_now+t` 时刻的预测位置（轨迹超时后用末端速度线性外推，`:419-433`）拼成 7 个顶点，`updateGraph()` 更新矩阵（`:435`）：
   - 邻接矩阵 `A(i,j) = ‖p_i−p_j‖²`（平方欧氏，全连接图）
   - 度 `D(i) = Σ_j A(i,j)`
   - 对称归一化拉普拉斯 `L̂(i,j) = 1 (i=j); −A(i,j)/√(D_i·D_j) (i≠j)`
2. **代价**：`cost = ‖L̂ − L̂_des‖²_F`（`calcFNorm2`，`swarm_graph.cpp:90-99`），L̂_des 来自启动时 `setDesiredForm` 设定的模板。因邻接用平方距离，代价对平移、旋转不变，对尺度敏感（尺度由模板/swarm_scale 确定）——这就是"只锁形状、不锁朝向"的数学根源。
3. **梯度**：解析链式法则 `∂F/∂p = (df/de)ᵀ·(de/dp)`（`calcFGrad`，`:101-151`），实现里把梯度归一化为单位方向（模长 < 1e-7 置零）——有意的工程简化，破坏严格梯度一致性，仅提供下降方向。
4. **门控**：需所有邻居都有轨迹（`swarm_trajs_->size() >= formation_size_`，`:388-394`）才生效；时间梯度 `gradt = Σ w·grad[id]·vel[id]`（`:438-457`）。

### 6.3 身份与槽位分配：launch 时静态给定，终身不变

**身份来自参数，不来自协商**：每机 `drone_id` 由 launch 写死（`run_in_sim.launch` 传入 → `manager/drone_id`，`advanced_param.xml:150`），槽位绑定在两处：

- **终点派生（强绑定，逐点对应）**：`end_pt_ = 中心 + scale × relative_pos_[drone_id]`（`ego_replan_fsm.cpp:650`）——drone 0 = 中心，drone 1-6 各奔各的顶点；
- **谱代价图索引（弱绑定）**：优化时邻居预测位置按对方 drone_id 填进图对应行列，自己占第 drone_id 行，梯度只取 `grad[drone_id_]`（`poly_traj_optimizer.cpp:438-457`）。所有人同模板、同索引规则、同批广播数据（信息同源），各机独立拼出的"当前队形图"一致。

**没有在线角色分配/换位**：飞行中不会出现"2 号 3 号互换槽位"的协商——Munkres 匈牙利算法 + `Assignment.msg` 是预留接口，从未被调用（死代码）。

**对称性细节**：对正六边形这种高度对称队形，谱代价**对外圈顶点互换不敏感**（6 顶点的旋转/反射置换是图自同构，置换后 L̂_des 不变）——真正把某机钉死在"它那个顶点"上的是终点派生，不是谱约束；中心位唯一，与顶点不可互换。

### 6.4 编队需要哪些输入

**启动时静态输入（配置，一次性）**：

| 输入 | 值/来源 | 作用 |
|---|---|---|
| `formation_type` | 1 = REGULAR_HEXAGON（`normal_hexagon.yaml:19`） | 选模板，`setDesiredFormation` 硬编码 7 顶点 → L̂_des |
| `drone_id` | launch 参数（`advanced_param.xml:150`） | 槽位索引（§6.3） |
| `weight_formation` | 15000（`normal_hexagon.yaml:25`） | 与避障/互避权衡 |
| `swarm_scale` + `relative_pos_0..6` | 2.0 / 六边形顶点（`normal_hexagon.yaml:5-12`） | 终点派生用 |

**运行时动态输入（每次重规划）**：

1. **自己的候选约束点**——当前优化中的决策变量（未来轨迹上的采样点）；
2. **6 条邻居广播轨迹**（`PolyTraj`：`drone_id + start_time + order==5 + coef_x/y/z + duration`）——按 `t_now+t` 解析取值，过期用末端速度线性外推（`poly_traj_optimizer.cpp:419-433`）；
3. **统一时间基准 t_now**——由时钟同步保证（偏差 ≤0.25s，超时轨迹被丢弃）；
4. **门控条件**：全部 7 个槽位（含自己）都有轨迹，编队项才生效（`swarm_trajs_->size() ≥ formation_size_`，`:388-394`）。

**不需要的输入**：全局地图、队友实时位置测量、中心节点指令、在线角色分配——都不需要。编队所需的外部信息只有"邻居的承诺轨迹 + 一个同步的时钟"。

### 6.5 修改编队：改哪里、三个坑

编队**只能"改配置重启"，不能运行时切换**。

**① 必须同步改两处模板（最大的坑：无单一数据源）**：

```
目标端:  normal_hexagon.yaml  relative_pos_0..6 + swarm_scale   （终点派生用）
优化端:  poly_traj_optimizer.h:205-219  setDesiredFormation 硬编码 v0..v6  （谱代价用，改完要重编译）
```

两处不一致 → 终点摆一个形状、谱约束拉另一个形状，行为错乱。

**② 改机数要动三处**：`normal_hexagon.launch`（增减 include）、yaml（relative_pos 条数）、cpp 模板（顶点数 + `formation_size_`）——机数没有集中定义。

**③ 加新队形要加 case**：`FORMATION_TYPE` 枚举只有 `NONE_FORMATION` 与 `REGULAR_HEXAGON` 两个值（`poly_traj_optimizer.h:202-225`）。

**运行时动态换队形：不支持**。要自己加接口：订阅 FormationCommand 消息 → 更新 FSM 的 `swarm_relative_pts_`（目标端）+ 调 `setDesiredFormation`（优化端）；注意门控 `formation_size_` 要匹配、模板切换要平滑过渡（队形突变会使谱梯度跳变、轨迹抖动）。`Assignment.msg` + Munkres 是给"在线槽位分配"预留的接口，同样未实现。

**理论提示**：换**非对称队形**时行为会变——正六边形的高度对称性使谱代价对外圈顶点互换无感（§6.3），非对称队形的 L̂_des 不再对置换不变，谱代价会强烈绑定槽位，槽位约束从终点一处实质变成两处。

---

## 7. 机间通信与协同

**有没有中控？没有，连"队长"都算不上有。** 决策层完全对等：7 个 ego_planner_node 跑相同代码、各自优化自己的轨迹，没有任何节点替别人做决定。三个容易被误认为中控的东西其实都不是：

| 嫌疑对象 | 实际身份 |
|---|---|
| ROS master（仿真单机跑） | 只是通信枢纽（话题注册/查找），不参与任何决策；实机各机自跑全套栈，靠 UDP 互通 |
| swarm_bridge | **哑中继**：收到 PolyTraj 原样 UDP 广播，无逻辑、无状态、不仲裁，drone_id=999 只是占位符 |
| drone 0（中心机） | 顺序启动的**静态优先级约定**（别人等它的轨迹，`have_recv_pre_agent_`），它不指挥任何人、别人也不向它汇报——是"排队上车"不是"领导带队" |

协同完全是**自组织涌现**：每机广播轨迹 → 每机独立把邻居轨迹纳入自己的优化 → 队形与互避作为优化结果自然出现。

### 7.1 swarm_bridge：UDP 轨迹广播

`swarm_bridge/src/bridge_node.cpp`，纯 UDP 广播中继（端口 8081，1MB 缓冲）：

- 仿真中单 bridge（drone_id=999，`127.0.0.255` 回环广播），所有 7 个 planner 共享同一 ROS master；实机部署每台机器各跑一个 bridge
- 序列化用 `ros::serialization` 原生格式；**drone_id 靠消息体字段区分，而非话题名**——所有机器的 planner 都发布到同一全局话题 `/broadcast_traj_from_planner`，bridge 转发为 `/broadcast_traj_to_planner`（remap 见 `advanced_param.xml:58-59`）
- 接收端 `RecvBroadcastPolyTrajCallback`（`ego_replan_fsm.cpp:454-542`）：校验 order==5、系数数量、时间戳偏差 ≤ 0.25s；丢弃自己的；按 `swarm_traj[recv_id]` 下标存储

### 7.2 机间"定位"：交换承诺轨迹，按时间外推

机间协同**不依赖实时相对定位**，而是互相广播未来的多项式轨迹，邻居位置全部由轨迹外推得到：

- 每机优化完，把 5 阶多项式系数 + `drone_id` + `start_time` 广播出去
- 邻居的"现在和未来在哪" = 邻居广播轨迹在 `t_now + t` 时刻的解析取值，轨迹过期则用末端速度线性外推（`poly_traj_optimizer.cpp:419-433`）
- 互避代价、编队代价里的邻居位置全部来自这个预测，不来自任何实时测量。精度取决于对方的轨迹跟踪精度 + 时钟同步（超时轨迹直接丢弃）

本质上是用"你 0.1s 前承诺的轨迹"替代"你现在实测的位置"。bridge 里预留了真正的 odom 交换通道（`my_odom` → UDP ODOM → `/others_odom`，带 `child_frame_id="drone_<id>"`），但 planner 侧未订阅——是给实机做相对位姿估计的预留接口，此版本未启用。

### 7.3 顺序启动与时间对齐

- **顺序启动**：`SEQUENTIAL_START` 要求 drone id≥1 的机器收到所有低编号机器的首条轨迹才允许首次规划（`have_recv_pre_agent_`，`ego_replan_fsm.cpp:158-176, 527-541`）——drone 0 先飞先广播，drone k 收齐 0..k-1 才启动，保证编队项从第一刻起就有完整邻居参考。这是**分布式按 drone_id 递增的顺序启动机制**。
- **时间对齐**：编队模式下重规划起点取未来 0.1s（`replan_trajectory_time`）的轨迹状态（`ego_replan_fsm.cpp:790-799`），新轨迹 `start_time` 也设为该未来时刻（必要时 sleep 对齐，`planner_manager.cpp:259-269`），多机参考时间一致。

### 7.4 冲突响应

收到邻居轨迹立即 `checkCollision(recv_id)`（`planner_manager.cpp:293-316`）：取两者轨迹时间重叠段按 0.03s 采样检查间距 < `swarm_clearance`，冲突则切 `REPLAN_TRAJ`（`ego_replan_fsm.cpp:521-524`）。

---

## 8. 执行与控制链

### 8.1 traj_server：轨迹 → 位置指令

`plan_manage/src/traj_server.cpp`：订阅 `planning/trajectory` 重建多项式轨迹（`:56-92`），100Hz 定时器采样（`:321`）：**解析微分**取 pos/vel/acc/jerk 填入 `quadrotor_msgs/PositionCommand`（`:241-244, 284-294`）——不是数值差分，是解析前馈。yaw 由指向未来 1s（`time_forward_`）位置的方向角计算并限幅（π rad/s）+ 低通（`calculate_yaw`，`:121-215`）；轨迹结束后悬停。

### 8.2 机体仿真：两条可选链路

launch 中二选一（`simulator.xml`）：

**fake_drone（默认启用，`:106-112`）**——理想化"完美跟踪"机体：

- 工程名 `poscmd_2_odom`（`fake_drone/src/poscmd_2_odom.cpp`）：订阅 `~command`，100Hz 发布 `~odometry`
- pose ← cmd.position，twist ← cmd.velocity（`:31-33`）；姿态由 `acc + 9.8·e_z`（推力方向）与 cmd.yaw 解析构造（`:35-49`）——等效于瞬时实现 SO3 期望姿态
- 即跳过控制与动力学，指令即状态

**完整动力学链（代码完整，swarm 仿真中注释保留，`:76-104`）**：

```
PositionCommand → so3_control（位置环）→ SO3Command → quadrotor_simulator_so3（姿态环+动力学）→ odom/imu
```

- `so3_control`（nodelet）：位置环 **PD + 期望加速度前馈**（无积分项，非 PID），`force = m·g·e_z + kx·(x_d−x) + kv·(v_d−v) + m·a_d`（`SO3Control.cpp:70-73`），倾角限 45°，由推力方向生成期望姿态四元数；增益 mass=0.98kg、kx≈5.7、kv≈3.4（`so3_control_nodelet.cpp:196-221`）
- `quadrotor_simulator_so3`：内嵌几何 SO3 姿态控制器（Lee 几何控制，误差 eR/eOm + 陀螺力矩补偿，`quadrotor_simulator_so3.cpp:39-158`）→ 混控分配 4 电机 rpm → **22 维 ODE**（位置/速度/旋转/角速度/电机转速，电机一阶滞后 τ=1/30s，`Quadrotor.cpp:99-206`）用打包的 boost::odeint 1000Hz 积分，按 100~200Hz 发布真值 odom/imu
- 真实机体上姿态环+混控对应飞控固件，位置环对应机载计算机；`quadrotor_msgs` 还附带与飞控 MCU 的串口编解码（`encode_msgs.h`/`decode_msgs.h`/`comm_types.h`，SO3_CMD=115 等消息号），即 SO3Command 可直接下发真实飞控

做控制/动力学层面实验时需手动切换到这条链。

### 8.3 可视化：odom_visualization

`Utils/odom_visualization/src/odom_visualization.cpp`：订阅 `~odom`/`~cmd`，发布 mesh 机体模型（hummingbird.mesh）、路径（>0.1s 抽样）、速度箭头、颜色编码轨迹（每 0.5s 一段）、协方差椭球、TF 等，全部 latched。

---

## 9. 关键参数速查

| 参数 | 值 | 位置 |
|---|---|---|
| 集群规模 | 7 机正六边形 + 中心 | `normal_hexagon.launch` |
| 传感器感知半径 sensing_horizon | 5.0 m | `simulator.xml:128` |
| 规划视野 planning_horizon | 7.5 m（≈1.5× 感知半径） | `advanced_param.xml:73` |
| 局部地图更新范围 local_update_range | 5.5/5.5/4.5 m | `advanced_param.xml:100-102` |
| 栅格分辨率 | 0.1 m | `advanced_param.xml:96` |
| 障碍膨胀 obstacles_inflation | 0.1 m（1 体素） | `advanced_param.xml:106` |
| 优化器障碍裕度 obstacle_clearance | 0.5 m（走 ESDF 梯度） | `normal_hexagon.yaml:26` |
| 机间安全距离 swarm_clearance | 0.5 m | `normal_hexagon.yaml` |
| 编队权重 / 避障权重 / 互避权重 | 15000 / 50000 / 50000 | `normal_hexagon.yaml:20-25` |
| 编队规模 swarm_scale | 2.0（六边形边长 4m） | `normal_hexagon.yaml:5` |
| 重规划阈值 thresh_replan_time | 1.0 s | `advanced_param.xml:71` |
| 最大速度 / 最大加速度 max_vel / max_acc | 1.5 m/s / 8.0 m/s²（manager 与 optimization 共用） | `run_in_sim.launch:72-73` |
| FSM 频率 / 安全检查频率 | 100 Hz / 20 Hz | `ego_replan_fsm.cpp:69-70` |
| 虚拟天花板 | 3.0 m | `advanced_param.xml:131` |
| 全局地图范围 | 34×15×3 m（小于规划桶 70×30×3） | `normal_hexagon.launch:18-20` |
| 跨机时钟偏差上限 | 0.25 s | `ego_replan_fsm.cpp:471` |

---

## 10. 实现细节与陷阱

1. **默认传感器形态与 README 场景不符**：CUDA 关闭 → 实际是点云直融（无 raycasting、无 log-odds、无 free-space 清理），要复现标准 EGO 深度图行为需开 CUDA 重编 `local_sensing`。
2. **死代码**：`Assignment.msg` + 匈牙利算法（munkres）预留的目标分配功能从未被调用；`DataDisp` 消息字段从不填充；`/others_odom` 里程计交换在 planner 侧未订阅；map_generator 的 `local_cloud` 从不发布；`clickCallback` 点击加障碍无订阅接线（`random_forest_sensing.cpp:342`）。
3. 全局地图生成范围 34×15×3m **小于**规划地图桶 70×30×3m。
4. `simulator.xml` 里的 mockamap/内嵌地图块被 CDATA 包裹不生效，真正的地图节点在 `normal_hexagon.launch`。
5. 点云通路不做 free-space 光线清理，依赖每帧 `resetBuffer` 局部清零，动态避障粒度受 resetBuffer 范围限制。
6. `obstacles_inflation=0.1` 只膨胀 1 个体素，软裕度主要靠优化器 ESDF 代价（`obstacle_clearance=0.5`），两者职责分离。
7. 编队模式 L-BFGS 最大迭代 20 次（实时优先），无编队模式 60 次。
8. 目标消息的 z 不生效：只做有效性检查（z < -0.1 丢弃），飞行高度硬编码为编队中心 z=0.5m。
9. `simulator.xml:123` 的 `drone_id` 值是字面量字符串而非 `$(arg drone_id)`，疑似笔误。
10. **优化器内部可疑实现**：① `addPropCtoP` 用 `=` 覆盖而非 `+=` 回传航点梯度（`poly_traj_utils.hpp:1009-1018`，原 `+=` 版本被注释、标注 zxzx）——若同一航点已累积其他代价梯度会被清掉；② 代价统计向量 `costs(3)` 声明后从未写入（`poly_traj_optimizer.cpp:147`）——调权重时不要相信任何"分项成本"打印。
11. **编队门控的特殊放宽与重复检查**：`drone_id == formation_size_−1`（末位机 drone 6）会绕过"等齐所有邻居轨迹"的门控直接放行（`poly_traj_optimizer.cpp:388-391`）；同一 `i_dp` 边界检查在该函数里写了两遍（`:383-384` 与 `:393-394`，复制粘贴痕迹）。A* 起点偏移超 0.5m 会自动用非 ESDF 检查重搜一次（`dyn_a_star.cpp:265-276`）。
12. **仓库不含任何单元测试**，改动优化器后只能靠仿真整体行为验证；多机建议用 rosmon 起launch（README §3）。
13. **队形模板双份无单一数据源**：yaml 与 cpp 各存一份，修改编队必须同步两处（详见 §6.5）。

---

## 11. 总结

这是一个"**局部感知 + 滚动 MINCO 优化**"的分布式编队系统，四条主线：

- **地图**：没有全局地图（仿真中的全局点云只是传感器渲染素材），每架机维护自己的滑窗式局部占据/ESDF 地图，窗外即遗忘；
- **导航**：靠"目标世界坐标 + 不避障的全局参考线 + 7.5m 滚动重规划"到达目标，无全局最优性保证（凹形障碍存在局部极小风险）；
- **协同**：机间不测相对位姿，而是互播多项式轨迹、按时间外推（互避与编队代价里的邻居位置全部来自轨迹预测），配顺序启动与 0.25s 时钟校验；
- **编队**：队形模板（drone 0 居中、1-6 六边形顶点）+ 终点摆阵 + 拉普拉斯谱相似性软约束（平移旋转不变、遇障可临时变形）多层合成。

执行端输出 5 阶多项式轨迹，由理想化机体（fake_drone，默认）或完整 SO3 动力学链跟踪；定位在仿真中是真值直读，实机依赖每机独立 VINS（世界系对齐与全局导航层不在本开源仓库内）。
