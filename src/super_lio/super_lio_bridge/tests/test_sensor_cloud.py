import copy
import struct
from collections import OrderedDict
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from super_lio_bridge.tf_bridge import SuperLioBridge


def bridge():
    sensor, world = [], []
    node = SimpleNamespace(
        _pose_history=OrderedDict(), _pending_sensor_clouds=OrderedDict(),
        _T_boot=np.eye(4), _T_imu_base=np.eye(4), _external=True,
        _odom='odom', _map='map', _sensor_frame='lidar', _ground_clearance=0.08,
        _sensor_warned=False, _sensor_cloud_pub=SimpleNamespace(publish=sensor.append),
        _cloud_pub=SimpleNamespace(publish=world.append))
    node._publish_sensor_cloud = lambda msg: SuperLioBridge._publish_sensor_cloud(node, msg)
    return node, sensor, world


def cloud(stamp=10, points=((2., 0., 1.),), padded=False, big=False):
    msg = PointCloud2()
    msg.header.frame_id = 'world'; msg.header.stamp.sec = stamp
    msg.height = len(points) if padded else 1
    msg.width = 1 if padded else len(points)
    msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
    msg.point_step = 12; msg.row_step = 16 if padded else 12 * msg.width
    msg.is_bigendian = big
    msg.data = b''.join(struct.pack(('>' if big else '<') + 'fff', *p) +
                        (b'xxxx' if padded else b'') for p in points)
    return msg


def pose(node, stamp, x):
    msg = Odometry(); msg.header.stamp.sec = stamp
    msg.pose.pose.position.x = x; msg.pose.pose.orientation.w = 1.
    SuperLioBridge._on_odom(node, msg)


def test_stamp_matching_and_header_ownership():
    node, sensor, world = bridge()
    pose(node, 10, 1.); pose(node, 11, 9.)
    msg = cloud()
    SuperLioBridge._on_cloud(node, msg)
    assert struct.unpack('<fff', bytes(sensor[-1].data))[0] == 1.
    assert msg.header.frame_id == 'world'
    assert world[-1].header.frame_id == 'odom'


def test_cloud_before_odometry_is_released_when_matching_pose_arrives():
    node, sensor, _ = bridge()
    SuperLioBridge._on_cloud(node, cloud())
    assert not sensor
    pose(node, 10, 1.)
    assert sensor[-1].width == 1
    assert not node._pending_sensor_clouds


def test_padding_endianness_and_empty_filtered_cloud():
    node, sensor, _ = bridge(); pose(node, 10, 1.)
    msg = cloud(points=((2., 0., 1.), (3., 0., float('nan'))), padded=True, big=True)
    SuperLioBridge._on_cloud(node, msg)
    assert sensor[-1].width == 1
    assert struct.unpack('>fff', bytes(sensor[-1].data))[0] == 1.
    SuperLioBridge._on_cloud(node, cloud(points=((2., 0., 0.),)))
    assert sensor[-1].width == 0 and bytes(sensor[-1].data) == b''
