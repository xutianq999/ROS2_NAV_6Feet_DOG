# Hexplorer miniHex_v2 移植说明（3D 导航版）

> 2026-10-25 把本导航栈从 BUNKER MINI 2.0（轮式）移植到 **Dobot Hexplorer
> DT-HEX-6R010-L / miniHex_v2（六足）**，并把导航感知从"3D 点云转 2D 激光"
> 升级为 **3D voxel 代价地图**。vendor 侧（`robot_controller_release`、
> `dobot_hex_sdk`）零修改，只通过 ROS 话题交互。

## 1. 与 Bunker 版的差异总览

| 环节 | Bunker 版（原） | Hexplorer 版（现） |
|---|---|---|
| 底盘执行 | `bunker_ros2` + ugv_sdk（CAN 500k） | `hexplorer_adapter`（`/cmd_vel`→`/vel_cmd` + 状态机） |
| 里程计 | Super-LIO（不变） | Super-LIO（不变） |
| TF 链 | `base_footprint→chassis(z0.06)→livox(z0.26)` | `base_footprint→chassis(z0.25*)→livox(0.382,0,0.096; pitch15°*)` |
| 代价地图 | `ObstacleLayer` 吃 2D `/scan_reliable` | `VoxelLayer` 直接吃 3D `/registered_scan` |
| 碰撞监控 | 只用 `/scan` | `/registered_scan`(3D) + `/scan`(2D 冗余) |
| 足迹 | 0.69×0.57 m | 0.86×0.47 m（站立包络，手册规格） |
| 速度 | vx 0.4 / wz 1.0 | vx 0.4 / wz 0.6（步态惯性大，加速度也收窄） |
| 雷达 IP | 192.168.1.170 | **192.168.1.152**（实测确认：驱动 10Hz 点云 + 200Hz IMU 出数） |

带 `*` 的两项是**实机校准点**，见第 4 节。

## 2. 底盘适配层（新增 `src/hexplorer_adapter`）

vendor 运控（`robot_controller_release/executable/main`，整机上电自启）暴露：

- `/robot_state`（`custom_msg/RobotState`）：`temp[10]` 为状态
  0=阻尼 1=位置折叠 2=力控站立 3=balanceStand 4=WALK(力控行走)；
- `/robot_cmd`（`custom_msg/RobotCommand`）：`target_state` 逐级切换状态；
- `/vel_cmd`（`geometry_msgs/Twist`）：行走速度，**与手柄输入叠加**（手册 19 页）。

`hexplorer_adapter_node` 做的事：

1. `/cmd_vel`（安全链终点：twist_mux + deadman 之后）→ 限幅 → `/vel_cmd` @50Hz；
2. `/cmd_vel` 静默 0.5s 自动发零速（等价原 deadman 兜底）；
3. `auto_stand_up=true` 时参照官方 locomotion.py 逐级 +1 步进到 WALK（每级 1s）；
4. 机器人掉出 WALK（手柄切走/趴下）立即停止转发速度；**不主动降级状态机**，
   阻尼/趴下永远交给手柄或 vendor 逻辑；
5. 速度限幅：vx≤0.4、wz≤0.6（`max_linear_y=0`，导航按差速用，六足的横移能力先不用，
   恢复行为/守卫都是按差速推理的）。

手柄随时可以叠加/压过导航指令，是天然的人工兜底通道。

## 3. 导航 3D 化

- `nav2_params.yaml`：局部/全局代价地图的障碍源从 2D `/scan_reliable` 换成
  `VoxelLayer` 直接吃 `/registered_scan`（桥接层转出的 Super-LIO 配准点云，
  internal 模式在 map 系、external 在 odom 系，都在 TF 链上）：
  - 局部（odom 系）：地面 z≈0，`min_obstacle_height 0.08` 滤地面；
  - 全局（map 系）：map 原点=启动时机身中心，地面≈-0.25，
    `min_obstacle_height -0.20`（与 grid_mapper 的 z_min 同一约定）；
  - 体素 16 层 × 0.15m。
- `collision_monitor.yaml`：加 `pointcloud` 源（min 0.08 / max 1.5，base 系），
  保留 `/scan` 作为低负载冗余。
- `/scan` 2D 切片**仍然保留**（恢复守卫 / SmartRecovery 还依赖它），
  但切片目标帧从 `livox_frame` 改为 `chassis`——雷达前倾 15°，
  在倾斜帧里切高度会被 `sin(15°)×距离` 拉歪。
- `/scan_reliable` 中继不再被代价地图使用（保留运行，回滚零成本）。

回滚到 2D 方案：把两处 costmap 的 `voxel_layer` 换回注释里的 `obstacle_layer`
（git 历史里有），其余不用动。

## 4. 实机校准点（首次上电必做）

1. **雷达网络**：本机有线口已是 `192.168.1.10/24`（enp2s0），雷达 IP
   `192.168.1.152`（实测确认；vendor 配置里的 .190 已过时，实际符合手册"1+序列号后两位"规则），
   已写入 `src/livox_ros_driver2/config/MID360_config.json`。
   若收不到数据：先确认机器人总电源已开、能 `ping 192.168.1.152`(vendor 配置里的 .190 是过时的,实测雷达在 .152,符合"1+序列号后两位"规则)；
   再看 vendor 自启的 `livox_lidar_node` 是否还在占端口——导航脚本启动前
   会自动清掉它（cleanup --driver）。
2. **机身高度**：机器人 balanceStand 站立后
   `ros2 topic echo /robot_state --field pos_body`，用 `pos_body[2]`（m）
   更新两处（数值必须一致）：
   - `src/gld_robot_description/model/gld_robot_description.urdf` 的
     `base_to_chassis`（当前占位 0.25）；
   - 重算 `src/super_lio/super_lio_bridge/config/bridge.yaml` 的 `imu_to_base`
     （见该文件注释的计算方法，当前 [-0.2794, 0, -0.4331, 0, -0.2618, 0]）。
3. **雷达俯仰方向**：手册图按"前倾 15°（低头）"处理。若实机为后仰，
   把 URDF 的 pitch 改 -0.2618 并同步重算 bridge 外参。上电后可对照
   vendor `livox_lidar_node` 发布的 `/tf_static` 核对（它发布机身→雷达 TF）。
4. **libasio**：ugv_sdk 需要 asio 头文件。本机没装 `libasio-dev`（要 sudo），
   已用 standalone asio 头文件（`~/.local/include`，build.sh 自动叠加）。
   有 sudo 时建议 `sudo apt install -y libasio-dev can-utils` 后删掉
   `~/.local/include` 里的 asio。

## 5. 编译与启动

本机（miniHex_v2 板载 MiniPC）从零搭好的依赖，全部**免 sudo**：

| 依赖 | 安装方式 |
|---|---|
| fastapi / uvicorn / python-multipart（Web 后端） | `pip install --user` |
| tornado / autobahn / bson / ujson / cbor2（rosbridge） | `pip install --user` |
| asio 头文件（ugv_sdk 需要，替代 libasio-dev） | standalone asio 解压到 `~/.local/include`，build.sh 自动加 CPATH |
| small_gicp / robin / kiss_matcher / GTSAM 4.2 | 源码编译装到 `~/.local`（替代 `sudo make install`），build.sh 自动加 CMAKE_PREFIX_PATH 和链接 rpath |

新增 vendor 进 `src/` 的 ROS 包（本机缺对应 apt 包）：
`pointcloud_to_laserscan`、`twist_mux`、`rosbridge_suite`（均来自上游 humble/ros2 分支；
rosbridge 的 ament_cmake_mypy 改为可选——Humble 的 apt 仓库没有该包）。

顺手修复的上游 Humble 兼容问题（都在我们自己的工作空间内）：
- `registration/global_relocalization`、`registration/small_gicp_relocalization`：
  small_gicp 1.x 的 `small_gicp::small_gicp` 命名空间 target 只在 install 后存在，
  FetchContent 路径补了 ALIAS；
- `m-explore-ros2`：explore/map_merge 用了 Iron+ 的 tf2 头文件名（.hpp→.h），
  map_merge 测试目标补了丢失的依赖声明；
- `me_nav2_bringup`：SmartRecovery 校验 `footprint_radius >= 外接半径`，
  六足尺寸下取 0.50。

```bash
# 1) 本工作空间（scripts/build.sh 会自动叠加 vendor custom_msg、asio 头文件、~/.local 库）
cd ~/Ros2_nav/Ros2_NAV2_Stack_261020-main
./scripts/build.sh                       # 内存充足可 BUILD_JOBS=4

# 2) Super-LIO 上游（独立工作空间，必须先 source 本工作空间）
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ~/superlio_ws/Super-LIO-ros2 && colcon build --symlink-install

# 3) 底盘（自动识别六足；vendor 运控由整机上电自启）
./scripts/chassis_start.sh               # 新终端；等待 /robot_state 出现

# 4) 导航（mapping 建图 / relocation / tracking / tracking_kiss / exploration）
./scripts/super_lio_nav.sh tracking
```

无硬件验证（2026-10-25 已在本机跑通，均不接触实机）：

```bash
# ① 3D voxel 代价地图注入测试 —— PASS
# 终端 1:
ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args \
  --params-file src/me_nav2_bringup/config/nav2_params.yaml
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true -p "node_names:=['costmap/costmap']"
# 终端 2:
python3 scripts/test_voxel_costmap.py --topic /costmap/costmap
# 结果: 3D 墙被标记(内切/致命)、地面回波与 1.8m 悬空点被高度过滤 PASS

# ② 整链启动烟雾测试（无雷达无底盘，补一个假 odom->base_footprint TF）
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint &
ros2 launch super_lio_bridge super_lio_bringup_launch.py \
  mode:=relocation rviz:=false use_livox_driver:=false use_map_server:=true
# 结果: 24 个进程全部启动、0 崩溃，
#       map/collision_monitor/navigation 三个 lifecycle_manager 全部
#       "Managed nodes are active"，local/global 代价地图均加载 voxel_layer
```

## 6. 注意事项

- **整机上电后 vendor 自启节点很多**（雷达/相机/运控/语音）。雷达节点与我们
  的驱动抢端口，导航脚本启动时会清理；相机节点（在 Jetson 上）不影响。
- 探索模式（`exploration`）沿用 grid_mapper 的 map 系高度带
  （z_min=-0.20，对应离地约 0.15m），与新的代价地图一致，无需修改。
- Super-LIO 源码包在 `~/superlio_ws/Super-LIO-ros2`（ros2 分支 f89f48d），
  `super_lio_nav.sh` 的默认路径已指向这里。
- 本机 miniconda 按用户手册默认**不激活**，ROS 全链使用系统 Python 3.10。
- 建议有 sudo 时补装 `sudo apt install -y libasio-dev can-utils`（可删
  `~/.local/include/asio`；can-utils 提供 candump，六足模式下仅诊断用途）。
