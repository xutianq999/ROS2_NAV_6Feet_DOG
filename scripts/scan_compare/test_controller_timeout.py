import os, subprocess, time, signal
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import PointCloud2
from scan_planner_msgs.msg import Bspline
assert os.environ.get('ROS_DOMAIN_ID') == '198' and os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use isolated ROS domain 198 and ROS_LOCALHOST_ONLY=1'
p = subprocess.Popen(['ros2','run','scan_planner','closed_loop_controller','--ros-args','-p','odom_timeout:=0.4','-p','trajectory_timeout:=0.8','-p','require_cloud:=true','-p','cloud_timeout:=0.4','-r','cmd_vel:=/scan_timeout_test/cmd'], start_new_session=True)
rclpy.init(); n=Node('scan_timeout_fixture')
od=n.create_publisher(Odometry,'/body_pose',10); tr=n.create_publisher(Bspline,'/planning/bspline',10)
cloud=n.create_publisher(PointCloud2,'/cloud',10)
received=[]
n.create_subscription(Twist,'/scan_timeout_test/cmd',lambda m: received.append((time.monotonic(),m.linear.x)),10)
msg=Odometry(); msg.pose.pose.orientation.w=1.0
b=Bspline(); b.order=3; b.traj_id=1; b.knots=[float(i)*0.2 for i in range(8)]; b.pos_pts=[Point(x=float(i)*0.2) for i in range(4)]
cloud_msg=PointCloud2(); cloud_msg.width=1; cloud_msg.height=1; cloud_msg.data=[0]
def tick(seconds, send_odom=True, send_traj=False, send_cloud=True):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if send_odom: od.publish(msg)
        if send_traj: tr.publish(b)
        if send_cloud: cloud.publish(cloud_msg)
        rclpy.spin_once(n, timeout_sec=0.05)
try:
    tick(1.2,True,True)
    assert any(abs(x) > 0.01 for _,x in received[-15:]), 'no moving command'
    tick(1.2,True,False)
    assert received and abs(received[-1][1]) < 1e-6, 'trajectory timeout did not stop'
    tick(0.7,True,True)
    assert any(abs(x) > .01 for _,x in received[-15:]), 'fresh trajectory did not start'
    tick(0.8,False,False)
    assert abs(received[-1][1]) < 1e-6, 'odometry timeout did not stop'
    tick(0.5,True,False)
    assert abs(received[-1][1]) < 1e-6, 'stale trajectory resumed'
    tick(0.7,True,True)
    assert any(abs(x) > .01 for _,x in received[-15:]), 'fresh trajectory did not restart'
    tick(0.8,True,True,False)
    assert abs(received[-1][1]) < 1e-6, 'cloud timeout did not stop'
    tick(0.5,True,False,True)
    assert abs(received[-1][1]) < 1e-6, 'trajectory resumed after cloud loss'
    b.pos_pts[1].x = float('nan')
    tick(0.5,True,True,True)
    assert abs(received[-1][1]) < 1e-6, 'non-finite trajectory did not stop'
    b.pos_pts[1].x = 0.2
    tick(0.7,True,True,True)
    assert any(abs(x) > 0.01 for _,x in received[-15:]), 'valid trajectory did not recover'
    b.start_time = (n.get_clock().now() - rclpy.duration.Duration(seconds=3.0)).to_msg()
    tick(0.5,True,True,True)
    assert abs(received[-1][1]) < 1e-6, 'expired trajectory was executed'
    print('PASS: trajectory, odometry, cloud timeouts; invalid trajectory and recovery', flush=True)
finally:
    n.destroy_node(); rclpy.shutdown()
    os.killpg(p.pid, signal.SIGTERM)
    try:
        p.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        p.wait(timeout=5)
