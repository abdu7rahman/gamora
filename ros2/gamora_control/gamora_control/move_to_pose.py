#!/usr/bin/env python3
"""Plan and execute a single Cartesian pose goal for the arm group.

ROS 2 port of gamora_test_v6/move.py.

`moveit_commander` does not exist in ROS 2. The equivalent is the
`moveit_py` Python bindings, which need an explicit node and parameters loaded
from the MoveIt config — so this must be launched with the move_group
parameters, not run bare. `move_to_pose.launch.py` does that.
"""

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


# The grasp pose from the ROS 1 script, unchanged.
GRASP_POSE = {
    'position': (-0.14738496092831854, 0.014994630138540962, 0.2562754992270229),
    'orientation': (-0.08643387766022224, -0.7033921930997056,
                    0.7035986137006519, 0.052130588693444675),
}


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('gamora_move_to_pose')

    gamora = MoveItPy(node_name='gamora_move_to_pose')
    arm = gamora.get_planning_component('arm')

    pose_goal = PoseStamped()
    # ROS 1 set a bare Pose with no frame, which left MoveIt to assume the
    # planning frame. Being explicit avoids depending on that default.
    pose_goal.header.frame_id = 'base_link'
    (pose_goal.pose.position.x,
     pose_goal.pose.position.y,
     pose_goal.pose.position.z) = GRASP_POSE['position']
    (pose_goal.pose.orientation.x,
     pose_goal.pose.orientation.y,
     pose_goal.pose.orientation.z,
     pose_goal.pose.orientation.w) = GRASP_POSE['orientation']

    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link='link5')

    logger.info('planning to grasp pose')
    plan_result = arm.plan()

    if plan_result:
        logger.info('planning succeeded, executing')
        gamora.execute(plan_result.trajectory, controllers=[])
        logger.info('done')
    else:
        # The ROS 1 script ignored the return of arm_group.go() entirely, so a
        # failed plan looked identical to a successful one.
        logger.error('planning failed')

    gamora.shutdown()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
