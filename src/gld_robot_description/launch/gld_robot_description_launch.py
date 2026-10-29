import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('gld_robot_description')
    rviz_file = os.path.join(pkg_share, 'rviz', 'nav2_new.rviz')
    urdf_file = os.path.join(pkg_share, 'model', 'gld_robot_description.urdf')
    rviz = LaunchConfiguration('rviz')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 六足腿部关节 TF：vendor 自带的 robot_jointstate_publisher_node 不订阅
    # /robot_state（发的是恒定占位值，腿在 RViz 里呈僵尸摊开状），所以用
    # hexplorer_adapter 包里的 joint_state_bridge 把 /robot_state 的
    # jpos_leg 转成 /joint_states。无实机（回放/仿真）时 joint_states:=false。
    joint_states = LaunchConfiguration('joint_states')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Launch RViz for robot description'),
        DeclareLaunchArgument(
            'joint_states', default_value='true',
            description='启动关节状态桥（/robot_state -> /joint_states）'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='hexplorer_adapter',
            executable='joint_state_bridge',
            name='joint_state_bridge',
            output='screen',
            condition=IfCondition(joint_states),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_file],
            condition=IfCondition(rviz),
        )
    ])
