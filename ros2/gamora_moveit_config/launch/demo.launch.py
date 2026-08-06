"""Full MoveIt 2 demo: move_group, rviz2 and mock hardware.

ROS 2 port of gamora/launch/demo.launch. MoveItConfigsBuilder replaces the
nested chain of .launch/.launch.xml includes the setup assistant generated —
planning_context, move_group, moveit_rviz, trajectory_execution and the rest are
all folded into one builder call.
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('gamora_test_v6', package_name='gamora_moveit_config')
        .robot_description(
            file_path='config/gamora_test_v6.urdf.xacro',
            mappings={'hardware': 'mock'},
        )
        .robot_description_semantic(file_path='config/gamora_test_v6.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
