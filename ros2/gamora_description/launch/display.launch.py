"""View the arm in rviz2 with sliders for each joint.

ROS 2 port of gamora_test_v6/launch/display.launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('gamora_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'gamora_test_v6.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')

    gui = LaunchConfiguration('gui')

    # ROS 1 called xacro at parameter-load time via $(find xacro)/xacro.
    # ParameterValue with value_type=str keeps launch from trying to parse the
    # generated XML as a nested YAML structure.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show joint_state_publisher_gui sliders.'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
