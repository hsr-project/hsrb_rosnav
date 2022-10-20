import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory("hsrb_mapping"),
                                   'config', 'mapping_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all slam_toolbox nodes')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("slam_toolbox"),
                                                   'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': params_file}.items())

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(slam_toolbox_launch)
    return ld
