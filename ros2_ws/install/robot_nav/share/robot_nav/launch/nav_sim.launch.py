from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')
        }.items()
    )

    controller_node = Node(
        package='robot_nav',
        executable='controller',
        name='pure_pursuit_controller',
        output='screen'
    )

    return LaunchDescription([
        world_launch,
        controller_node
    ])

