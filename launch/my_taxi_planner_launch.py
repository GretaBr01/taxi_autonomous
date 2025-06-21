import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    
    # Directory del tuo pacchetto
    taxi_pkg_dir = get_package_share_directory('my_taxi_autonomous')  # Cambia con il nome del tuo pacchetto
    
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)


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

    robot_description_content = Command([
        'xacro ', os.path.join(taxi_pkg_dir, 'worlds', 'diff_car_ros2_controll.xacro') 
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content
            # 'use_sim_time': True
        }],
        output='screen'
    )


    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', os.path.join(taxi_pkg_dir, 'worlds', 'robotica.sdf')],
        output='screen'
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('gz_ros2_control_demos'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )
  
    # spawn_robot = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-name', 'simple_tracked',
    #         '-topic', 'robot_description',
    #         '-x', '0', '-y', '2', '-z', '0.2'  # posizione iniziale
    #     ],
    #     output='screen'
    # )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_drive',
            '-topic', 'robot_description',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

 # legge lo stato attuale dei joint del robot (posizione, velocità, sforzo), 
 # pubblica questi dati sul topic /joint_states in formato sensor_msgs/JointState
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller',
                    '--param-file',
                    robot_controllers,
                    '--controller-ros-args',
                    '-r /diff_drive_controller/cmd_vel:=/cmd_vel',],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # '/diff_drive_base_controller/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry', # Gazebo -> ROS 2
            # '/odom@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance', # ROS 2 -> Gazebo
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
        output='screen'
    )



    # Nodi azione
    drive_normal_cmd = Node(
        package='my_taxi_autonomous',
        executable='drive_normal_action_node',
        name='drive_normal_action_node',
        output='screen',
        parameters=[os.path.join(taxi_pkg_dir, 'config', 'coords_drive_normal.yaml')]
    )

    drive_normal_traffic_cmd = Node(
        package='my_taxi_autonomous',
        executable='drive_normal_traffic_action_node',
        name='drive_normal_traffic_action_node',
        output='screen',
        parameters=[os.path.join(taxi_pkg_dir, 'config', 'coords_drive_normal_traffic.yaml')]
    )

    drive_to_charge_cmd = Node(
        package='my_taxi_autonomous',
        executable='drive_to_charge_action_node',
        name='drive_to_charge_action_node',
        output='screen',
        parameters=[os.path.join(taxi_pkg_dir, 'config', 'coords_drive_to_charge.yaml')]
    )

    drive_to_charge_traffic_cmd = Node(
        package='my_taxi_autonomous',
        executable='drive_to_charge_traffic_action_node',
        name='drive_to_charge_traffic_action_node',
        output='screen',
        parameters=[os.path.join(taxi_pkg_dir, 'config', 'coords_drive_to_charge_traffic.yaml')]
    )

    charge_cmd = Node(
        package='my_taxi_autonomous',
        executable='charge_action_node',
        name='charge_action_node',
        output='screen'
    )

    pickup_cmd = Node(
        package='my_taxi_autonomous',
        executable='pickup_action_node',
        name='pickup_action_node',
        output='screen'
    )

    dropoff_cmd = Node(
        package='my_taxi_autonomous',
        executable='dropoff_action_node',
        name='dropoff_action_node',
        output='screen'
    )

    bridge_cmd_vel = Node(
        package='my_taxi_autonomous',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen'
    )


    # Costruzione della LaunchDescription
    ld = LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [f' -r -v 1 {os.path.join(taxi_pkg_dir, 'worlds', 'robotica.sdf')}'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[load_diff_drive_controller],
            )
        ),
        bridge,
        spawn_robot,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
    ])
    # ld.add_action(gz_sim)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(bridge_cmd_vel)
    # ld.add_action(OpaqueFunction(function=robot_state_publisher_node))
    # ld.add_action(bridge)
    # ld.add_action(spawn_robot)
    # ld.add_action(joint_state_broadcaster_spawner)
    # ld.add_action(load_diff_drive_controller)  

    ld.add_action(plansys2_cmd)

    ld.add_action(drive_normal_cmd)
    ld.add_action(drive_normal_traffic_cmd)
    ld.add_action(drive_to_charge_cmd)
    ld.add_action(drive_to_charge_traffic_cmd)
    ld.add_action(charge_cmd)
    ld.add_action(pickup_cmd)
    ld.add_action(dropoff_cmd)

    return ld
