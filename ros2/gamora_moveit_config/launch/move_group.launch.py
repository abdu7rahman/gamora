"""move_group only, no rviz and no controllers.

ROS 2 port of gamora/launch/move_group.launch.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('gamora_test_v6', package_name='gamora_moveit_config')
        .robot_description(file_path='config/gamora_test_v6.urdf.xacro')
        .robot_description_semantic(file_path='config/gamora_test_v6.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
