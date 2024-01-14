from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

    foo_dir = get_package_share_directory('turtlebot4_bringup')
    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            foo_dir + '/launch/oakd.launch.py'))


    robot_state_publisher_node = Node(
        package='adaptive_cruise_control',
        executable='road_detect_main.py',
    )

    joint_state_publisher_node = Node(
        package='adaptive_cruise_control',
        executable='lidar_node',
    )



    return LaunchDescription([
        included_launch,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])