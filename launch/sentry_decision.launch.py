import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rm_decision_cpp')

    # 声明参数
    team_color_arg = DeclareLaunchArgument(
        'team_color',
        default_value='blue',
        description='队伍颜色: blue 或 red'
    )

    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(pkg_dir, 'behavior_tree', 'sentry_nav.xml'),
        description='行为树 XML 文件路径'
    )

    tick_rate_arg = DeclareLaunchArgument(
        'tick_rate',
        default_value='100.0',
        description='行为树 tick 频率 (Hz)'
    )

    attack_distance_arg = DeclareLaunchArgument(
        'attack_distance',
        default_value='2.0',
        description='攻击距离 (m)'
    )

    # 创建节点
    sentry_decision_node = Node(
        package='rm_decision_cpp',
        executable='tree_exec_node',
        name='sentry_decision_node',
        output='screen',
        parameters=[{
            'bt_xml_file': LaunchConfiguration('bt_xml_file'),
            'tick_rate': LaunchConfiguration('tick_rate'),
            'attack_distance': LaunchConfiguration('attack_distance'),
        }],
    )

    return LaunchDescription([
        team_color_arg,
        bt_xml_arg,
        tick_rate_arg,
        attack_distance_arg,
        sentry_decision_node,
    ])
