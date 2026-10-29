#!/usr/bin/env python3
"""把 vendor /robot_state 的 jpos_leg（float32[18]）转成 /joint_states，
驱动 robot_state_publisher 的 18 条腿部 TF。

为什么不直接用 vendor 的 robot_jointstate_publisher_node：实测它不订阅
/robot_state（只挂了个 /system_monitor_default），发布的是恒定占位值
（1.5/0.1），腿在 RViz 里呈僵尸摊开状——不能要。

关节名与顺序和 URDF / 官方约定一致：六条腿 × (abad, thigh_pitch,
calf_pitch)，腿序 front_left, front_right, mid_left, mid_right,
rear_left, rear_right。若实机渲染出现左右镜像，交换这里的 LEGS 顺序即可。

/robot_state 是 200Hz，全量转发会让 robot_state_publisher 以同频刷 18 条
TF；限频到 publish_rate（默认 50Hz），对可视化绰绰有余。
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from custom_msg.msg import RobotState

LEGS = ('front_left', 'front_right', 'mid_left', 'mid_right',
        'rear_left', 'rear_right')
JOINTS = ('abad', 'thigh_pitch', 'calf_pitch')


class JointStateBridge(Node):

    def __init__(self) -> None:
        super().__init__('joint_state_bridge')
        self.declare_parameter('robot_state_topic', '/robot_state')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('publish_rate', 50.0)
        self._names = [f'joint_{leg}_{j}' for leg in LEGS for j in JOINTS]
        self._period = 1.0 / max(1.0, float(
            self.get_parameter('publish_rate').value))
        self._last_pub = 0.0
        self._warned = False
        self.create_subscription(
            RobotState, self.get_parameter('robot_state_topic').value,
            self._on_state, 10)
        self._pub = self.create_publisher(
            JointState, self.get_parameter('joint_states_topic').value, 10)

    def _on_state(self, msg: RobotState) -> None:
        now = time.monotonic()
        if now - self._last_pub < self._period:
            return
        if len(msg.jpos_leg) != 18:
            if not self._warned:
                self._warned = True
                self.get_logger().warning(
                    f'jpos_leg 长度 {len(msg.jpos_leg)} != 18，跳过')
            return
        self._last_pub = now
        m = JointState()
        m.header.stamp = msg.header.stamp
        m.name = self._names
        m.position = [float(v) for v in msg.jpos_leg]
        self._pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
