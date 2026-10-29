"""单独启动 Hexplorer 适配层（调试用）。

正常使用走 scripts/chassis_start.sh（会自动选择六足/bunker）；
这里提供 ros2 launch hexplorer_adapter adapter.launch.py 的独立入口。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_stand_up', default_value='true',
            description='启动后是否自动把六足状态机步进到 WALK'),
        DeclareLaunchArgument(
            'max_linear_x', default_value='0.4'),
        DeclareLaunchArgument(
            'max_angular_z', default_value='0.6'),
        Node(
            package='hexplorer_adapter',
            executable='hexplorer_adapter_node',
            name='hexplorer_adapter',
            output='screen',
            parameters=[{
                'auto_stand_up': LaunchConfiguration('auto_stand_up'),
                'max_linear_x': LaunchConfiguration('max_linear_x'),
                'max_angular_z': LaunchConfiguration('max_angular_z'),
            }],
        ),
    ])
