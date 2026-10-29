#!/usr/bin/env python3
"""Read-only adapter for comparing SCAN trajectories with Nav2."""
import copy
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import Marker


class Adapter(Node):
    def __init__(self):
        super().__init__('scan_compare_adapter')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.body = self.create_publisher(Odometry, '/scan_compare/body_pose', 10)
        self.sensor = self.create_publisher(Odometry, '/scan_compare/sensor_pose', 10)
        self.path = self.create_publisher(Path, '/scan_compare/initial_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_body = None
        self.last_velocity = (0.0, 0.0, 0.0)
        marker_qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(Marker, '/scan_compare/trajectory_odom', marker_qos)
        self.create_subscription(Marker, '/scan_compare/optimal_list', self.on_marker, marker_qos)
        self.declare_parameter('input_path', '/plan')
        input_path = self.get_parameter('input_path').value
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if input_path == '/scan_control/plan' else 1
        self.pending_path = None
        self.create_subscription(Path, input_path, self.on_path, path_qos)
        self.create_timer(0.05, self.publish_poses)

    def publish_poses(self):
        try:
            body = self.tf.lookup_transform('odom', 'chassis', Time())
            sensor = self.tf.lookup_transform('odom', 'livox_frame', Time())
            now = self.get_clock().now()
            if (now - Time.from_msg(body.header.stamp)).nanoseconds > 1_000_000_000:
                self.last_body = None
                self.last_velocity = (0.0, 0.0, 0.0)
                return
            for transform, publisher, child in ((body, self.body, 'chassis'),
                                                (sensor, self.sensor, 'livox_frame')):
                msg = Odometry()
                msg.header = transform.header
                msg.child_frame_id = child
                msg.pose.pose.position.x = transform.transform.translation.x
                msg.pose.pose.position.y = transform.transform.translation.y
                msg.pose.pose.position.z = transform.transform.translation.z
                msg.pose.pose.orientation = transform.transform.rotation
                if child == 'chassis':
                    stamp = Time.from_msg(transform.header.stamp).nanoseconds
                    p = transform.transform.translation
                    current = (stamp, p.x, p.y, p.z)
                    if self.last_body and stamp != self.last_body[0]:
                        dt = (stamp - self.last_body[0]) / 1e9
                        self.last_velocity = (0.0, 0.0, 0.0)
                        if 0.02 <= dt <= 0.5:
                            velocity = [(current[i] - self.last_body[i]) / dt for i in (1, 2, 3)]
                            if all(math.isfinite(v) and abs(v) <= 1.0 for v in velocity):
                                self.last_velocity = velocity
                    if not self.last_body or stamp != self.last_body[0]:
                        self.last_body = current
                    # SCAN interprets the linear components in odom coordinates.
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = self.last_velocity
                publisher.publish(msg)
            if self.pending_path is not None:
                self.on_path(self.pending_path)
        except TransformException:
            self.last_body = None
            self.last_velocity = (0.0, 0.0, 0.0)

    def on_path(self, msg):
        if not msg.poses or not msg.header.frame_id:
            return
        try:
            transform = self.tf.lookup_transform('odom', msg.header.frame_id, Time())
        except TransformException as exc:
            self.pending_path = msg
            self.get_logger().warning(f'Cannot transform Nav2 path: {exc}', throttle_duration_sec=3.0)
            return
        try:
            ground_z = self.tf.lookup_transform(
                'odom', 'base_footprint', Time()).transform.translation.z
        except TransformException as exc:
            self.get_logger().warning(f'Cannot read ground height: {exc}')
            return
        output = Path()
        output.header = copy.copy(msg.header)
        output.header.frame_id = 'odom'
        for pose in msg.poses:
            p = PoseStamped()
            p.header = msg.header
            p.pose = pose.pose
            transformed = do_transform_pose(p.pose, transform)
            # Nav2 paths are 2D (z=0 in map), which lands ~0.4 m below the floor in
            # odom. SCAN adds body_height to path z, so hand it floor height.
            transformed.position.z = ground_z
            converted = PoseStamped()
            converted.header = output.header
            converted.pose = transformed
            output.poses.append(converted)
        self.path.publish(output)
        self.pending_path = None

    def on_marker(self, msg):
        # The upstream visualizer hard-codes "world" for this output.
        msg.header.frame_id = 'odom'
        self.markers.publish(msg)


def main():
    rclpy.init()
    node = Adapter()
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
