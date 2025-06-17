import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Directory del tuo pacchetto
    #gazebo_pkg = get_package_share_directory('gazebo_ros')
    taxi_pkg_dir = get_package_share_directory('taxi_autonomous_nav2')  # Cambia con il nome del tuo pacchetto
    #world_path = os.path.join(taxi_pkg_dir, 'worlds', 'robotica.sdf')

    # Avvio del sistema PlanSys2
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': os.path.join(taxi_pkg_dir, 'pddl', 'domain.pddl'),
            'params_file': os.path.join(taxi_pkg_dir, 'params', 'plansys2_params.yaml')
        }.items()
    )

    # gazebo_cmd =IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
    #         ),
    #         launch_arguments={'world': world_path}.items()
    # )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', os.path.join(taxi_pkg_dir, 'worlds', 'robotica.sdf')],
        output='screen'
    )

    # Nav2
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'map': os.path.join(taxi_pkg_dir, 'maps', 'map.yaml'),
            'params_file': os.path.join(taxi_pkg_dir, 'params', 'nav2_params.yaml'),
            'autostart': 'true'
        }.items()
    )

    # Nodi azione
    drive_normal_cmd = Node(
        package='taxi_autonomous_nav2',
        executable='drive_normal_action_node',
        name='drive_normal_action_node',
        output='screen',
        parameters=[os.path.join(taxi_pkg_dir, 'config', 'coords_drive_normal.yaml')]
    )

    drive_normal_traffic_cmd = Node(
        package='taxi_autonomous_nav2',
        executable='drive_normal_traffic_action_node',
        name='drive_normal_traffic_action_node',
        output='screen',
        parameters=[os.path.join(taxi_pkg_dir, 'config', 'coords_drive_normal_traffic.yaml')]
    )

    drive_to_charge_cmd = Node(
        package='taxi_autonomous_nav2',
        executable='drive_to_charge_action_node',
        name='drive_to_charge_action_node',
        output='screen',
        parameters=[os.path.join(taxi_pkg_dir, 'config', 'coords_drive_to_charge.yaml')]
    )

    drive_to_charge_traffic_cmd = Node(
        package='taxi_autonomous_nav2',
        executable='drive_to_charge_traffic_action_node',
        name='drive_to_charge_traffic_action_node',
        output='screen',
        parameters=[os.path.join(taxi_pkg_dir, 'config', 'coords_drive_to_charge_traffic.yaml')]
    )

    charge_cmd = Node(
        package='taxi_autonomous_nav2',
        executable='charge_action_node',
        name='charge_action_node',
        output='screen'
    )

    pickup_cmd = Node(
        package='taxi_autonomous_nav2',
        executable='pickup_action_node',
        name='pickup_action_node',
        output='screen'
    )

    dropoff_cmd = Node(
        package='taxi_autonomous_nav2',
        executable='dropoff_action_node',
        name='dropoff_action_node',
        output='screen'
    )

    # Costruzione della LaunchDescription
    ld = LaunchDescription()
    ld.add_action(plansys2_cmd)
    # ld.add_action(gazebo_cmd)
    ld.add_action(gz_sim)
    ld.add_action(nav2_cmd)

    ld.add_action(drive_normal_cmd)
    ld.add_action(drive_normal_traffic_cmd)
    ld.add_action(drive_to_charge_cmd)
    ld.add_action(drive_to_charge_traffic_cmd)
    ld.add_action(charge_cmd)
    ld.add_action(pickup_cmd)
    ld.add_action(dropoff_cmd)

    return ld
