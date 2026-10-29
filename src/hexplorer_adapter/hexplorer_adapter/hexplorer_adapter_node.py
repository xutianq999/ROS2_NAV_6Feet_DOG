"""Nav2 -> Dobot Hexplorer (miniHex_v2) 底盘适配层。

## 定位

导航安全链的终点是 /cmd_vel（twist_mux 仲裁、deadman 兜底之后的最终速度）。
原来的执行者是 bunker_ros2（CAN 轮式底盘）；本机器人是 Dobot Hexplorer
miniHex_v2 六足，vendor 运控（robot_controller_release 里的 main，开机自启）
暴露的是另外一套接口，本节点做两者之间的桥：

    /cmd_vel  (geometry_msgs/Twist, 导航安全链输出)
        │  本节点：限幅 + 看门狗
        ▼
    /vel_cmd  (geometry_msgs/Twist, vendor 运控行走速度)
                速度指令会与手柄输入**叠加**（手册 19 页），
                手柄随时可以反向压过导航指令 —— 保留为人工兜底。

    /robot_state (custom_msg/RobotState, temp[10] 为状态)
        │  0=passive(阻尼) 1=standDown(位置折叠)
        │  2=standUp(力控站立) 3=balanceStand 4=Walk(力控行走)
        ▼
    /robot_cmd  (custom_msg/RobotCommand, target_state)
        逐级 +1 步进到 WALK，每级之间留状态切换时间（与官方
        dobot_hex_sdk/high_level/py/locomotion.py 的做法一致）。

## 安全行为

- /cmd_vel 静默超过 cmd_timeout 后发零速（等价于 bunker 链路的 deadman）。
- 速度指令先按 max_* 参数限幅，作为安全层之后的最后一道限幅。
- 机器人状态掉到 WALK 以下（趴下/阻尼/手柄切走）时停止转发速度，
  若 auto_stand_up 打开则重新逐级走到 WALK。
- 不发 /robot_cmd 降级指令：手柄是更高优先级的人工通道，状态机降级
  （如急停趴下）永远交给人工或 vendor 逻辑。
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from custom_msg.msg import RobotCommand, RobotState

# 与官方 locomotion.py 一致的状态编号
STATE_PASSIVE = 0
STATE_STAND_DOWN = 1
STATE_STAND_UP = 2
STATE_BALANCE_STAND = 3
STATE_WALK = 4


class HexplorerAdapter(Node):

    def __init__(self) -> None:
        super().__init__('hexplorer_adapter')

        # ---- 参数 ----
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('vel_cmd_topic', '/vel_cmd')
        self.declare_parameter('robot_cmd_topic', '/robot_cmd')
        self.declare_parameter('robot_state_topic', '/robot_state')
        self.declare_parameter('status_topic', '/chassis/status')
        # 速度限幅：安全层之后的最后一道。手册给的最大行走速度 1.8 m/s
        # 是能力上限，导航场景先压到 0.4（与 nav2_params.yaml 的 vx_max 一致），
        # 跑熟之后可以在两处同步放大。
        self.declare_parameter('max_linear_x', 0.4)
        self.declare_parameter('max_linear_y', 0.0)   # 导航按差速用，不发横移
        self.declare_parameter('max_angular_z', 0.6)
        # /cmd_vel 超过这个时间没有新消息就发零速（防 controller 崩溃后机器人飘走）
        self.declare_parameter('cmd_timeout', 0.5)
        # 转发频率。官方示例按 50Hz 持续发布，保持一致。
        self.declare_parameter('stream_rate', 50.0)
        # 启动后是否自动把状态机从当前状态逐级步进到 WALK。
        # 关掉后状态切换全部交给手柄，本节点只在 WALK 状态下转发速度。
        self.declare_parameter('auto_stand_up', True)
        # 相邻两级状态切换之间的等待时间（官方示例为 1s）
        self.declare_parameter('state_step_interval', 1.0)
        # /robot_state 超过这个时间没更新，视为运控离线，停止转发
        self.declare_parameter('state_timeout', 2.0)

        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._vel_cmd_topic = self.get_parameter('vel_cmd_topic').value
        self._max_x = float(self.get_parameter('max_linear_x').value)
        self._max_y = float(self.get_parameter('max_linear_y').value)
        self._max_w = float(self.get_parameter('max_angular_z').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self._auto_stand_up = bool(self.get_parameter('auto_stand_up').value)
        self._step_interval = float(self.get_parameter('state_step_interval').value)
        self._state_timeout = float(self.get_parameter('state_timeout').value)

        # ---- 状态 ----
        self._last_cmd: Twist | None = None
        self._last_cmd_monotonic: float | None = None
        self._robot_state: int | None = None
        self._last_state_monotonic: float | None = None
        self._next_step_allowed_at = 0.0   # 单调钟；步进状态机的节流
        self._walking = False              # 已处于 WALK 且可以转发速度

        latched = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._vel_pub = self.create_publisher(Twist, self._vel_cmd_topic, 1)
        self._robot_cmd_pub = self.create_publisher(
            RobotCommand, self.get_parameter('robot_cmd_topic').value, 1)
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, latched)

        self.create_subscription(
            Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(
            RobotState, self.get_parameter('robot_state_topic').value,
            self._on_robot_state, 10)

        rate = max(1.0, float(self.get_parameter('stream_rate').value))
        self.create_timer(1.0 / rate, self._on_timer)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'Hexplorer 适配层已启动：{self._cmd_vel_topic} -> {self._vel_cmd_topic}；'
            f'限幅 vx<={self._max_x} wz<={self._max_w}；'
            f'auto_stand_up={self._auto_stand_up}')

    # ------------------------------------------------------------------ 回调

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_monotonic = time.monotonic()

    def _on_robot_state(self, msg: RobotState) -> None:
        try:
            state = int(msg.temp[10])
        except (IndexError, ValueError, TypeError):
            return
        if state != self._robot_state:
            self.get_logger().info(f'机器人状态: {self._robot_state} -> {state}')
        self._robot_state = state
        self._last_state_monotonic = time.monotonic()

    # ------------------------------------------------------------------ 主循环

    def _on_timer(self) -> None:
        now = time.monotonic()

        walking = (self._robot_state == STATE_WALK
                   and self._last_state_monotonic is not None
                   and now - self._last_state_monotonic <= self._state_timeout)
        if walking != self._walking:
            self._walking = walking
            if walking:
                self.get_logger().info('机器人已进入 WALK，开始转发导航速度。')
            else:
                self.get_logger().warn(
                    '机器人不在 WALK（或 /robot_state 超时），停止转发速度。'
                    '手柄可随时接管状态切换。')

        if self._auto_stand_up:
            self._step_toward_walk(now)

        out = Twist()
        if self._walking and self._last_cmd is not None \
                and self._last_cmd_monotonic is not None \
                and now - self._last_cmd_monotonic <= self._cmd_timeout:
            cmd = self._last_cmd
            out.linear.x = self._clamp(cmd.linear.x, self._max_x)
            out.linear.y = self._clamp(cmd.linear.y, self._max_y)
            out.angular.z = self._clamp(cmd.angular.z, self._max_w)
        # 其余情况（无指令/超时/不在 WALK）：发零速，保持指令流连续，
        # 避免 vendor 运控端把"停发"理解为别的含义。
        self._vel_pub.publish(out)

    def _step_toward_walk(self, now: float) -> None:
        """逐级 +1 步进状态机到 WALK（官方 locomotion.py 的策略）。"""
        if self._robot_state is None or self._robot_state >= STATE_WALK:
            return
        if self._last_state_monotonic is None:
            return  # 还没收到过 /robot_state，等运控上线
        if now < self._next_step_allowed_at:
            return
        target = self._robot_state + 1
        cmd = RobotCommand()
        cmd.target_state = target
        self._robot_cmd_pub.publish(cmd)
        # 官方示例每步之后 sleep 1s；这里用时间戳节流，不阻塞回调。
        self._next_step_allowed_at = now + self._step_interval
        self.get_logger().info(
            f'状态步进: target_state={target}（等待运控切换，'
            f'若状态不动请用手柄检查机器人）')

    # ------------------------------------------------------------------ 辅助

    @staticmethod
    def _clamp(v: float, limit: float) -> float:
        if limit <= 0.0:
            return 0.0
        return max(-limit, min(limit, float(v)))

    def _publish_status(self) -> None:
        msg = String()
        state = self._robot_state
        state_names = {0: 'passive', 1: 'stand_down', 2: 'stand_up',
                       3: 'balance_stand', 4: 'walk'}
        cmd_fresh = (self._last_cmd_monotonic is not None and
                     time.monotonic() - self._last_cmd_monotonic
                     <= self._cmd_timeout)
        msg.data = (
            f'robot_state={state}({state_names.get(state, "?")}) '
            f'walking={self._walking} '
            f'cmd_fresh={cmd_fresh}')
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HexplorerAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
