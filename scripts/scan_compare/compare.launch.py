"""Run SCAN-Planner alongside Nav2 without starting any motion controller."""
import os
import ast
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def hex_geometry(root):
    """Read hexapod height/offset and apply the requested experimental radius."""
    urdf = ET.parse(os.path.join(root, 'src/gld_robot_description/model/gld_robot_description.urdf'))
    origin = urdf.find(".//joint[@name='base_to_chassis']/origin")
    height = float(origin.attrib['xyz'].split()[2])
    with open(os.path.join(root, 'src/me_nav2_bringup/config/nav2_params.yaml'), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    footprint = ast.literal_eval(config['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    half_length = max(abs(float(p[0])) for p in footprint)
    offset = half_length / 2.0
    # User-selected experimental radius; does not enclose every footprint corner.
    radius = 0.30
    box = urdf.find(".//link[@name='chassis']/collision/geometry/box")
    body_half_height = float(box.attrib['size'].split()[2]) / 2.0
    return height, offset, radius, body_half_height


def generate_launch_description():
    config = os.path.join(get_package_share_directory('scan_planner'), 'config', 'planner.yaml')
    with open(config, encoding='utf-8') as stream:
        planner_params = yaml.safe_load(stream)['scan_planner_node']['ros__parameters']
    adapter = os.path.join(os.path.dirname(__file__), 'adapter.py')
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
    height, offset, radius, body_half_height = hex_geometry(root)
    return LaunchDescription([
        DeclareLaunchArgument('input_path', default_value='/plan'),
        ExecuteProcess(cmd=['python3', adapter, '--ros-args', '-p',
                            ['input_path:=', LaunchConfiguration('input_path')]], output='screen'),
        Node(
            package='scan_planner', executable='scan_planner_node',
            namespace='scan_compare', name='scan_planner_node', output='screen',
            parameters=[planner_params, {
                'fsm.navi_mode': 3,
                'fsm.reference_path_transient_local': True,
                'grid_map.frame_id': 'odom',
                'grid_map.sliding_map_frame_id': 'scan_compare_sliding_map',
                'grid_map.sensor_type': 'lidar',
                'grid_map.cloud_is_world': False,
                'grid_map.need_extrinsic': False,
                'grid_map.body_height': height,
                'grid_map.double_cylinder_radius': radius,
                'grid_map.double_cylinder_offset': offset,
                'grid_map.obstacles_inflation_z_up': height,
                'grid_map.obstacles_inflation_z_down': body_half_height,
                'manager.max_vel': 0.4,
                'optimization.max_vel': 0.4,
            }],
            remappings=[('body_pose', '/scan_compare/body_pose'),
                        ('sensor_pose', '/scan_compare/sensor_pose'),
                        ('cloud', '/registered_scan_livox'),
                        ('initial_path', '/scan_compare/initial_path')]),
        Node(
            package='scan_planner', executable='closed_loop_controller',
            namespace='scan_compare', name='closed_loop_controller', output='screen',
            parameters=[{'max_vx': 0.4, 'max_vy': 0.0, 'max_vyaw': 0.35,
                         'require_cloud': True, 'cloud_timeout': 1.0}],
            remappings=[('body_pose', '/scan_compare/body_pose'),
                        ('cloud', '/registered_scan_livox'),
                        ('cmd_vel', '/scan_compare/cmd_vel_preview')]),
    ])
