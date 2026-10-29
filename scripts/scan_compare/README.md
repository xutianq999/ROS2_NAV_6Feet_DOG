# SCAN-Planner 只读对比

先启动现有 tracking 导航，在另一个终端运行 `./scripts/scan_compare.sh`。
导航 RViz 中已配置 SCAN 碰撞圆柱和局部轨迹；橙色的 `SCAN inflated obstacles`
默认关闭，需要时在 Displays 中勾选，观察结束后取消勾选以减少画面和计算负担。
障碍点云可视化会截去传感器上方过高的部分，不能作为完整三维碰撞范围的证明。
该入口还运行 SCAN 控制器，其速度只发布到 `/scan_compare/cmd_vel_preview`，
不连接底盘或现有安全链。

适配器使用 TF 获取 odom 系的 chassis 和 livox_frame 位姿，将 Nav2 `/plan`
转换到 odom，点云读取已滤除地面的 `/registered_scan_livox`，并由传感器位姿
转换到 odom。机身高度读取项目六足 URDF，当前为
0.37 m；双圆柱前后偏移从 Nav2 的六足矩形足迹 (0.86 × 0.48 m) 计算，
当前圆柱半径按用户要求设为 0.30 m、前后偏移 0.215 m。它不完全覆盖该矩形边角，
也不是逐关节的 URDF 碰撞模型，
且六足行走时的摆腿包络仍需实测。现有 `/odom` 没有有效速度，适配器通过
连续 TF 位姿差估算 odom 坐标系速度，数据间隔异常或位姿跳变时使用零值；
该估计尚未经过实机标定，运动中轨迹不能作为可靠的实机控制依据。
路径可能因 map→odom 修正而发生变化。
上游标记写死 world，适配器将最优轨迹标记重新发布为 odom。

这是实验配置；未验证三维障碍、步态包络或与底盘的控制接口。不能直接用于
驾驶，和 MPPI 的对比也需确认两者使用的传感器数据及机器人尺寸一致。
核心规划包已编译，隔离 ROS 域的空数据启动已验证；真实定位、点云和路径输入下
的轨迹输出尚未验证。应先在机器人静止时观察。

闭环控制器增加了里程计、轨迹和点云接收超时保护（分别为 0.5 s、2.0 s、
1.0 s）：数据断流时输出零速并丢弃当前轨迹，需要新轨迹才能再次运动。
点云超时保护由本实验启动文件启用。独立 ROS 域验证：

```bash
source /opt/ros/humble/setup.bash
source SCAN-Planner-ros2-community/install/setup.bash
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 scripts/scan_compare/test_controller_timeout.py
```

`scripts/scan_tracking.sh` 使用独立的 Nav2 控制器插件，替换 FollowPath 的 MPPI。
插件将全局路径发给 SCAN，只接受标记为当前路径的速度，并在指令过期时归零。
Nav2 的目标取消、进度检查、恢复行为以及现有速度平滑、碰撞监测和仲裁链仍保留。
SCAN 专用行为树每 5 秒刷新全局路径，局部障碍仍由 SCAN 持续重规划；控制器从轨迹
生成时间开始执行，避免每次收到新轨迹都从时间零重启。过期轨迹仍会被拒绝。
控制路径使用 transient-local QoS 保留首条消息；TF 或里程计尚未就绪时保留最近路径，
就绪后重试。新路径仍使用独立标识，旧路径的速度不能用于新目标。
速度上限为 0.4 m/s 和 0.35 rad/s，禁止横移。SCAN 模式将局部与全局地图的
膨胀半径分别设为 0.20 m 与 0.35 m（原 SCAN 配置的一半），不修改 Nav2 机器人足迹。

隔离 ROS 域中的模拟测试覆盖轨迹、里程计、点云超时，旧路径命令拒绝，Nav2
动作取消，以及合成点云下的完整规划和控制链。真实雷达、实际步态包络与底盘响应
仍需验证；先在开阔区域低速试验，确保可用硬件急停。

停止旧导航和旧对比节点后，在独立终端启动底盘，再运行：

```bash
./scripts/scan_tracking.sh
```

控制器模式使用 `/scan_control/plan`，预览速度在 `/scan_compare/cmd_vel_preview`，
机器人实际指令经过 Nav2 速度平滑和安全链。两个模式不能同时启动。
