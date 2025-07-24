# nav_sim.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf_file = os.path.join(
        '/opt/ros/foxy/share/turtlebot3_description/urdf',
        f'turtlebot3_{TURTLEBOT3_MODEL}.urdf'
    )

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_file],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'turtlebot3', '-file', urdf_file, '-x', '0.0', '-y', '0.0', '-z', '0.1'],
            output='screen'
        ),

        # Your controller node
        Node(
            package='robot_nav',
            executable='controller',
            name='trajectory_follower',
            output='screen'
        )
    ])

