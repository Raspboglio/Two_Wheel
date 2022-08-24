from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
import os
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
            "-z", "0.2", 
            "-Y", "0" 
        ]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_two_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'two_wheel_control'],
        output='screen'
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(two_wheel_prefix, 'config/ekf.yaml')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_node,
        load_joint_state_controller,
        load_two_wheel_controller,
        robot_localization_node,
        rviz_node,
    ])