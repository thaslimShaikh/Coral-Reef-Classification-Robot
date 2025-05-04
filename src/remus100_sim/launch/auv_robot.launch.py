import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import 
from launch_ros.actions import Node
get_package_share_directory

controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    output="screen",
    parameters=[os.path.join(remus100_sim_dir, 'config', 'ros2_control.yaml')]
)

def generate_launch_description():
    # Get package directory
    remus100_sim_dir = get_package_share_directory('remus100_sim')
    world_file = os.path.join(remus100_sim_dir, 'worlds', 'coral_reef.world')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Gazebo Client (GUI)
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Spawn coral reef model
    spawn_coral_reef = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', 
             '-file', os.path.join(remus100_sim_dir, 'models', 'coral_reef_small.dae'), 
             '-entity', 'coral_reef'],
        output='screen'
    )

    # Spawn Remus100 AUV model
    spawn_remus100 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', 
             '-file', os.path.join(remus100_sim_dir, 'models', 'remus100.dae'), 
             '-entity', 'remus100'],
        output='screen'
    )

    # Spawn Simple Robot
    spawn_simple_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-file', os.path.join(remus100_sim_dir, 'models', 'simple_robot.sdf'),
             '-entity', 'simple_robot'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Full path to the world file'
        ),
        gazebo_launch,
        gazebo_client,
        spawn_coral_reef,
        spawn_remus100,
        spawn_simple_robot  # Added simple_robot spawning process
    ])
    

