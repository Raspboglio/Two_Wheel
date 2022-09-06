from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
import os

def generate_launch_description():

    nav_prefix = get_package_share_directory("two_wheel_nav")

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(nav_prefix, 'config/ekf.yaml')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        # output='screen',
        arguments=['-d',os.path.join(nav_prefix, 'config/rviz_config.rviz')],
        parameters=[{'use_sim_time':True}]
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ get_package_share_directory('slam_toolbox'), '/launch/online_async_launch.py']),
        launch_arguments={'use_sim_time': 'true'}.items(),
        )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch/navigation_launch.py']),
        launch_arguments=[os.path.join(nav_prefix,'/config/nav2.yaml')],
    )
    return LaunchDescription([
        robot_localization_node,
        rviz_node,
        slam,
        nav2,
    ])