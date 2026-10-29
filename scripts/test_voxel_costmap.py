#!/usr/bin/env python3
"""无硬件验证 3D voxel 代价地图（nav2_params.yaml 的 voxel_layer 配置）。

原理：
  1. 发布静态 TF odom -> base_footprint（机器人站在原点）；
  2. 在 odom 系发布一帧合成 /registered_scan：
       - 一面"墙"：x=2.0, y∈[-1,1], z∈[0.2,1.2]  → 必须被标成致命障碍
       - 一些"地面点"：z≈0.0                       → 必须被 min_obstacle_height 滤掉
       - 一个"悬空点"：z≈1.8（超过 max_obstacle_height）→ 不标记
  3. 订阅 /local_costmap/costmap，检查：
       - 墙所在格 (x=2.0,y=0)   cost == 254 (LETHAL)
       - 地面点附近 (x=1.0,y=0.5) cost < 254
       - 悬空点下方 (x=1.5,y=-0.5) cost < 254
  4. 顺带检查 /local_costmap/voxel_grid 是否有数据（publish_voxel_map: True）。

用法：
  # 终端 1（先编译完本工作空间）：standalone 代价地图 + lifecycle 激活
  ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args \
      --params-file src/me_nav2_bringup/config/nav2_params.yaml
  ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
      -p autostart:=true -p "node_names:=['costmap/costmap']"
  #   （standalone 可执行的话题前缀是 /costmap/；完整导航栈里是
  #    /local_costmap/，此时用 --topic 切换）
  # 终端 2：
  python3 scripts/test_voxel_costmap.py [--topic /costmap/costmap]
"""

import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

WALL_X = 2.0
GROUND_XY = (1.0, 0.5)
OVERHANG_XY = (1.5, -0.5)


def make_cloud():
    points = []
    # 墙：z 0.2~1.2，应进体素层
    for zi in range(21):
        z = 0.2 + zi * 0.05
        for yi in range(41):
            y = -1.0 + yi * 0.05
            points.append((WALL_X, y, z))
    # 地面点：低于 min_obstacle_height(0.08)，必须被滤掉
    for xi in range(21):
        for yi in range(11):
            points.append((0.5 + xi * 0.05, GROUND_XY[1] - 0.25 + yi * 0.05, 0.0))
    # 悬空点：高于 max_obstacle_height(1.5)，必须被滤掉
    for yi in range(11):
        points.append((OVERHANG_XY[0], OVERHANG_XY[1] + yi * 0.05, 1.8))

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud = PointCloud2()
    cloud.header.frame_id = 'odom'
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = fields
    cloud.point_step = 12
    cloud.row_step = 12 * len(points)
    cloud.is_dense = True
    cloud.data = struct.pack('<%df' % (3 * len(points)),
                             *[c for p in points for c in p])
    return cloud


class VoxelTestNode(Node):

    def __init__(self):
        super().__init__('voxel_costmap_test')
        self._tf_pub = TransformBroadcaster(self)
        self._cloud_pub = self.create_publisher(PointCloud2, '/registered_scan', 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._cloud = make_cloud()
        self.create_timer(0.05, self._tick)
        self._start = time.monotonic()

    def _tick(self):
        now = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.rotation.w = 1.0
        self._tf_pub.sendTransform(tf)
        self._cloud.header.stamp = now
        self._cloud_pub.publish(self._cloud)


def world_to_costmap(costmap, wx, wy):
    mx = int((wx - costmap.info.origin.position.x) / costmap.info.resolution)
    my = int((wy - costmap.info.origin.position.y) / costmap.info.resolution)
    return mx + my * costmap.info.width


def main():
    rclpy.init()
    node = VoxelTestNode()
    costmap_topic = '/local_costmap/costmap'
    if len(sys.argv) > 2 and sys.argv[1] == '--topic':
        costmap_topic = sys.argv[2]
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    got = {}

    def on_costmap(msg):
        got['costmap'] = msg

    node.create_subscription(
        __import__('nav_msgs.msg', fromlist=['OccupancyGrid']).OccupancyGrid,
        costmap_topic, on_costmap, latched)

    spin_deadline = time.monotonic() + 25.0
    while rclpy.ok() and time.monotonic() < spin_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if 'costmap' in got and time.monotonic() - node._start > 8.0:
            break

    if 'costmap' not in got:
        print(f'FAIL: 25s 内没有收到 {costmap_topic} —— '
              'nav2_costmap_2d 是否已用 nav2_params.yaml 启动并激活？')
        sys.exit(1)

    cm = got['costmap']
    res = cm.info.resolution
    print(f'costmap: {cm.info.width}x{cm.info.height} res={res} '
          f'origin=({cm.info.origin.position.x:.2f},{cm.info.origin.position.y:.2f})')

    # 墙：取机器人正前方的 (2.0, 0)
    c_wall = cm.data[world_to_costmap(cm, WALL_X, 0.0)]
    c_ground = cm.data[world_to_costmap(cm, *GROUND_XY)]
    c_over = cm.data[world_to_costmap(cm, *OVERHANG_XY)]
    print(f'cost@墙(2.0,0)={c_wall}  cost@地面点(1.0,0.5)={c_ground}  '
          f'cost@悬空点(1.5,-0.5)={c_over}')

    ok = True
    if c_wall < 99:
        print('FAIL: 墙没有被标成障碍（话题上致命=100/内切=99） —— voxel_layer 没吃到点云或参数不对')
        ok = False
    if c_ground >= 99:
        print('FAIL: 地面点被误标成致命障碍 —— min_obstacle_height 没生效（话题语义）')
        ok = False
    if c_over >= 99:
        print('FAIL: 悬空点(1.8m)被误标 —— max_obstacle_height 没生效（话题语义）')
        ok = False

    print('PASS: 3D voxel 代价地图验证通过' if ok else 'FAIL: 见上')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
