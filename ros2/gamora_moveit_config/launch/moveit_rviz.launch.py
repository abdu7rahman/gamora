"""rviz2 with the MoveIt motion planning display.

ROS 2 port of gamora/launch/moveit_rviz.launch.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('gamora_test_v6', package_name='gamora_moveit_config')
        .robot_description(file_path='config/gamora_test_v6.urdf.xacro')
        .robot_description_semantic(file_path='config/gamora_test_v6.srdf')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
