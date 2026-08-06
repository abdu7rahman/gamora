"""Run move_to_pose with the MoveIt config loaded.

moveit_py needs the robot description, SRDF, kinematics and planning parameters
on its own node — running `ros2 run gamora_control move_to_pose` bare will fail
to construct MoveItPy. Start move_group first, then this.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('gamora_test_v6', package_name='gamora_moveit_config')
        .robot_description(file_path='config/gamora_test_v6.urdf.xacro')
        .robot_description_semantic(file_path='config/gamora_test_v6.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .moveit_cpp(file_path='config/moveit_py.yaml')
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package='gamora_control',
            executable='move_to_pose',
            output='screen',
            parameters=[moveit_config.to_dict()],
        ),
    ])
