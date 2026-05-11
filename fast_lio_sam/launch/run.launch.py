from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    lidar = LaunchConfiguration('lidar')
    odom_topic = LaunchConfiguration('odom_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')

    fast_lio_launch = {
        'ouster': 'mapping_ouster64.launch.py',
        'velodyne': 'mapping_velodyne.launch.py',
        'livox': 'mapping_avia.launch.py',
    }

    actions = [
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='ouster'),
        DeclareLaunchArgument('odom_topic', default_value='/Odometry'),
        DeclareLaunchArgument('lidar_topic', default_value='/cloud_registered'),
        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz_sam',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('fast_lio_sam'), 'config', 'sam_rviz.rviz'])],
            output='screen',
        ),
        Node(
            package='fast_lio_sam',
            executable='fast_lio_sam_node',
            name='fast_lio_sam_node',
            output='screen',
            parameters=[PathJoinSubstitution([FindPackageShare('fast_lio_sam'), 'config', 'config_ros2.yaml'])],
            remappings=[('/Odometry', odom_topic), ('/cloud_registered', lidar_topic)],
        ),
    ]

    for key, launch_file in fast_lio_launch.items():
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('fast_lio'), 'launch', launch_file])),
                condition=IfCondition(PythonExpression(["'", lidar, "' == '", key, "'"])),
                launch_arguments={'rviz': 'false'}.items(),
            )
        )

    return LaunchDescription(actions)
