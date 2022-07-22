from launch import LaunchDescription
from launch.actions import Node
from launch.actions import InlcudeLaunchDescription

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        pacakge='gazebo_ros',
        launch='gazebo.launch.py'
    )

    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['entity','TwoWheel']
    )

    return LaunchDescription([
        gazebo,
        spawn_node
    ])