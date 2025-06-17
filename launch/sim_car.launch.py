import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    taxi_pkg_dir = get_package_share_directory('taxi_autonomous_nav2') 
    world_path = os.path.join(taxi_pkg_dir, 'worlds', 'my_world.world')

    world = LaunchConfiguration('world', default='city')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='city',
        description='World name')

    tiago_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(taxi_pkg_dir, 'launch', 'tiago_hospital.launch.py')),
        launch_arguments={
          'world_name': world
        }.items())

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    ld = LaunchDescription()

    ld.add_action(tiago_sim_cmd)
 
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    return ld