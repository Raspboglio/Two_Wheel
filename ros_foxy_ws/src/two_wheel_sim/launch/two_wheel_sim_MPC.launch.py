from ast import arguments
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
    two_wheel_prefix = get_package_share_directory('two_wheel_sim')
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_prefix,'/launch/gzserver.launch.py']),
        # launch_arguments={'world':PathJoinSubstitution([two_wheel_prefix,'worlds','test_world'])}.items()
        launch_arguments={'pause' : 'true'}.items()
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_prefix,'/launch/gzclient.launch.py']),
    )

    
    two_wheel_urdf = Command([
        "xacro ",
        PathJoinSubstitution([two_wheel_prefix ,"urdf","two_wheel_MPC.xacro.urdf"])
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            "robot_description":two_wheel_urdf,
            'use_sim_time':True,
        }]
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

    

    return LaunchDescription([
        #gazebo_server,
        #gazebo_client, 
        robot_state_publisher,
        spawn_node,
        load_joint_state_controller,
        load_two_wheel_controller,
    ])