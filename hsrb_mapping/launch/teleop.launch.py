import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_vel = LaunchConfiguration('joy_vel')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')

    return LaunchDescription([
        DeclareLaunchArgument('joy_vel', default_value='/omni_base_controller/command_velocity',
                              description='Base velocity command topic name'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0',
                              description='Joystick deivice name'),
        DeclareLaunchArgument('config_filepath',
                              default_value=os.path.join(
                                  get_package_share_directory('hsrb_mapping'),
                                  'config', 'ps-holonomic.config.yaml'),
                              description='Full path to the joystick control config file'),

        Node(
            package='joy_linux', executable='joy_linux_node', name='joy_linux_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.05,
                'autorepeat_rate': 10.0}]),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath],
            remappings={('/cmd_vel', joy_vel)}),
    ])
