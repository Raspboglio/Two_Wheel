from ast import arguments
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from os.path import join as joinPath

def generate_launch_description():

    config_file = str(get_package_share_directory("two_wheel_control") + "/config/sliding_balance.yaml")
    print(config_file)
    balancing_node = Node(
        name="balancing_controller",
        package="two_wheel_control",
        executable="two_wheel_sliding_balance",
        parameters=[config_file],
    )

    return LaunchDescription([
        balancing_node,
    ])