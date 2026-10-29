#!/usr/bin/env python3
"""把当前实时的 /map（grid_mapper 实时栅格）保存为 PGM + YAML。

比 nav2_map_server 的 map_saver_cli 可靠：直接订阅 /map（transient_local），
等到第一条消息就落盘，等不到时明确报错退出。用法：

    python3 save_live_map.py <输出前缀>           # 不带 .pgm/.yaml 后缀
    python3 save_live_map.py /path/map --timeout 15
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

PGM_FREE = 254
PGM_UNKNOWN = 205
PGM_OCCUPIED = 0


def write_map(prefix: Path, grid: OccupancyGrid) -> None:
    w, h = grid.info.width, grid.info.height
    data = np.asarray(grid.data, dtype=np.int8).reshape(h, w)

    img = np.full((h, w), PGM_UNKNOWN, dtype=np.uint8)
    img[data == 0] = PGM_FREE
    img[(data > 50) & (data <= 100)] = PGM_OCCUPIED
    img[(data > 0) & (data <= 50)] = PGM_FREE

    pgm = prefix.with_suffix(".pgm")
    with open(pgm, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        # PGM 图像行从上到下，而栅格行从原点（左下）向上，需翻转
        f.write(np.flipud(img).tobytes())

    yaml = prefix.with_suffix(".yaml")
    yaml.write_text(
        f"image: {pgm.name}\n"
        f"resolution: {grid.info.resolution}\n"
        f"origin: [{grid.info.origin.position.x}, "
        f"{grid.info.origin.position.y}, "
        f"{grid.info.origin.orientation.z * 2:.6f}]\n"  # 平面图 yaw=2*atan2(z,w)≈2z
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )
    print(f"已保存: {pgm} ({w}x{h}, {grid.info.resolution} m/格)")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix", type=Path)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--topic", default="/map")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("save_live_map")
    got: list[OccupancyGrid] = []

    qos = QoSProfile(depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(
        OccupancyGrid, args.topic, lambda m: got.append(m), qos)

    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if got:
            break

    if not got:
        print(f"错误: {args.timeout} s 内没等到 {args.topic} 消息"
              "（栈未启动或 grid_mapper 没在发布）", file=sys.stderr)
        rclpy.shutdown()
        return 1

    write_map(args.prefix, got[0])
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
