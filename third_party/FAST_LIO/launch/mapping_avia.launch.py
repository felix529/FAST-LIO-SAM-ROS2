from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(
            package='fast_lio', executable='fastlio_mapping', name='laserMapping', output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('fast_lio'), 'config', 'avia_ros2.yaml']),
                {'feature_extract_enable': False, 'point_filter_num': 3, 'max_iteration': 3,
                 'filter_size_surf': 0.5, 'filter_size_map': 0.5, 'cube_side_length': 1000.0,
                 'runtime_pos_log_enable': False},
            ],
        ),
        Node(condition=IfCondition(rviz), package='rviz2', executable='rviz2', name='rviz',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('fast_lio'), 'rviz_cfg', 'loam_livox.rviz'])], output='screen'),
    ])

