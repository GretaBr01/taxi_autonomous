import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch_ros.actions import Node


def generate_launch_description():
    # Directory del tuo pacchetto
    taxi_pkg_dir = get_package_share_directory('taxi_autonomous')  # Cambia con il nome del tuo pacchetto

    # Avvio del sistema PlanSys2
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': os.path.join(taxi_pkg_dir, 'pddl', 'domain.pddl')
        }.items()
    )

    # Nodi azione
    move_cmd = Node(
        package='taxi_autonomous',
        executable='move_action_node',
        name='move_action_node',
        output='screen'
    )

    charge_cmd = Node(
        package='taxi_autonomous',
        executable='charge_action_node',
        name='charge_action_node',
        output='screen'
    )

    pickup_cmd = Node(
        package='taxi_autonomous',
        executable='pickup_action_node',
        name='pickup_action_node',
        output='screen'
    )

    dropoff_cmd = Node(
        package='taxi_autonomous',
        executable='dropoff_action_node',
        name='dropoff_action_node',
        output='screen'
    )

    # Costruzione della LaunchDescription
    ld = LaunchDescription()
    ld.add_action(plansys2_cmd)
    ld.add_action(move_cmd)
    ld.add_action(charge_cmd)
    ld.add_action(pickup_cmd)
    ld.add_action(dropoff_cmd)

    return ld
