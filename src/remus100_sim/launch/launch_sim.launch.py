from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with the world containing coral reef
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/home/thaslim/ros2_ws/src/remus100_sim/models/remus100/model.sdf'],
            output='screen'
        ),
        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_robot', '-file', '/home/thaslim/ros2_ws/src/remus100_sim/models/simple_robot/model.sdf', '-x', '0', '-y', '0', '-z', '-5'],
            output='screen'
        ),
        # Start the navigation node
        Node(
            package='remus100_sim',  # Replace with your ROS 2 package name
            executable='coral_reef_navigator',
            output='screen'
        )
    ])
