from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
import xacro

def generate_launch_description():
    gazebo_prefix = get_package_share_directory('gazebo_ros')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_prefix,'/launch/gazebo.launch.py'])
    )

    two_wheel_prefix = get_package_share_directory('two_wheel_sim')
    two_wheel_urdf = Command([
        "xacro ",
        PathJoinSubstitution([two_wheel_prefix ,"urdf","two_wheel.xacro.urdf"])
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {"robot_description":two_wheel_urdf}
        ]
    )

    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            "-entity","Two_Wheel",
            "-topic","robot_description",
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_node
    ])