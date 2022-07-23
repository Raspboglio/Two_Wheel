from ament_index_python.packages import get_package_share_directory
import controller_manager
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

def generate_launch_description():

    two_wheel_sim_path = get_package_share_directory("two_wheel_sim")

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([two_wheel_sim_path,'/launch/two_wheel_sim.launch.py'])
        )

    two_wheel_urdf = Command([
        "xacro ",
        two_wheel_sim_path+"/urdf/two_wheel.xacro.urdf"
    ])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description" : two_wheel_urdf}]
    )

    return LaunchDescription([
        controller_manager,
        robot_launch
    ])
