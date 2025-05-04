import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_path = os.path.expanduser("~/ros2_ws/src/remus100_sim")

    return LaunchDescription([
        # Start Gazebo server with world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', os.path.join(package_path, 'worlds', 'coral_reef.world')],
            output='screen'
        ),

        # Start Gazebo client (GUI)
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # Spawn coral_reef_small model
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', 
                 '-file', os.path.join(package_path, 'models', 'coral_reef_small.dae'), 
                 '-entity', 'coral_reef'],
            output='screen'
        ),

        # Spawn remus100 model
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', 
                 '-file', os.path.join(package_path, 'models', 'remus100.dae'), 
                 '-entity', 'remus100'],
            output='screen'
        ),
    ])

