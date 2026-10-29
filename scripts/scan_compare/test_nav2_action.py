import os, signal, subprocess, time, tempfile
from pathlib import Path as FilePath
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from lifecycle_msgs.srv import ChangeState
assert os.environ.get('ROS_DOMAIN_ID') == '196' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
root = FilePath(__file__).resolve().parents[2]
with (root / 'src/me_nav2_bringup/config/nav2_params.yaml').open() as f:
    config = yaml.safe_load(f)
config['controller_server']['ros__parameters']['FollowPath'] = {
    'plugin': 'scan_nav2_controller/ScanController', 'max_linear': 0.1,
    'max_angular': 0.25, 'command_timeout': 0.3}
costmap = config['local_costmap']['local_costmap']['ros__parameters']
costmap['plugins'] = ['inflation_layer']; costmap['width'] = costmap['height'] = 4
with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
    yaml.safe_dump(config, f)
    config_file = f.name
p = subprocess.Popen(['ros2','run','nav2_controller','controller_server','--ros-args','--params-file',config_file,'-r','cmd_vel:=/scan_test/command'], start_new_session=True, stdout=open('/tmp/scan_nav2_controller.log','w'), stderr=subprocess.STDOUT)
rclpy.init(); n=Node('scan_action_fixture'); tf=TransformBroadcaster(n)
cmd=n.create_publisher(TwistStamped,'/scan_compare/cmd_vel_stamped',10)
state={'stamp':None,'commands':[]}
n.create_subscription(Path,'/scan_control/plan',lambda m: state.update(stamp=m.header.stamp),10)
n.create_subscription(Twist,'/scan_test/command',lambda m: state['commands'].append((time.monotonic(),m.linear.x)),10)
def step(seconds, publish_cmd=True):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        t=TransformStamped(); t.header.stamp=n.get_clock().now().to_msg(); t.header.frame_id='odom'; t.child_frame_id='base_footprint'; t.transform.rotation.w=1.0; tf.sendTransform(t)
        if publish_cmd and state['stamp']:
            m=TwistStamped(); m.header.stamp=state['stamp']; m.header.frame_id='base_footprint'; m.twist.linear.x=0.08; cmd.publish(m)
        rclpy.spin_once(n,timeout_sec=0.03)
def wait_future(future, seconds=10):
    end=time.monotonic()+seconds
    while not future.done() and time.monotonic()<end: step(.05)
    if not future.done(): raise RuntimeError('future timeout')
    return future.result()
try:
    step(2)
    lifecycle=n.create_client(ChangeState,'/controller_server/change_state')
    assert lifecycle.wait_for_service(timeout_sec=5)
    for transition in (1,3):
        req=ChangeState.Request(); req.transition.id=transition
        assert wait_future(lifecycle.call_async(req)).success
    action=ActionClient(n,FollowPath,'/follow_path'); assert action.wait_for_server(timeout_sec=5)
    goal=FollowPath.Goal(); goal.controller_id='FollowPath'; goal.goal_checker_id='general_goal_checker'; goal.path.header.frame_id='odom'
    for x in (0.0,2.0):
        pose=PoseStamped(); pose.header.frame_id='odom'; pose.pose.position.x=x; pose.pose.orientation.w=1.; goal.path.poses.append(pose)
    handle=wait_future(action.send_goal_async(goal)); assert handle.accepted
    step(2)
    assert state['stamp'] is not None and any(v > 0.05 for _,v in state['commands']), state
    wait_future(handle.cancel_goal_async()); step(1,False)
    assert state['commands'] and abs(state['commands'][-1][1]) < 0.001, state['commands'][-5:]
    print('PASS: Nav2 SCAN plugin loads, produces bounded command and stops after cancellation', flush=True)
finally:
    n.destroy_node(); rclpy.shutdown()
    os.killpg(p.pid,signal.SIGTERM)
    try: p.wait(timeout=5)
    except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL); p.wait(timeout=5)
    os.unlink(config_file)
