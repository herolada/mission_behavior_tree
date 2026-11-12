from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission = LaunchConfiguration('mission')
    default_params = PathJoinSubstitution(
        [FindPackageShare('mission_behavior_tree'), 'config', [mission, '.yaml']])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission', default_value='battery_patrol',
            description='Mission to run: loads config/<mission>.yaml of this package.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameters of mission_bt_node, overrides the mission config.'),
        Node(
            package='mission_behavior_tree',
            executable='mission_bt_node',
            name='mission_behavior_tree',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
