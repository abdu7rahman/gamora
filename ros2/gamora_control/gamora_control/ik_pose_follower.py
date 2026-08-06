#!/usr/bin/env python3
"""Follow a target pose by calling the /compute_ik service.

ROS 2 port of gamora_test_v6/pne_realtime.py, and of the same idea in
realtime_moveit.py — both took a pose from rviz and pushed the resulting joint
configuration out on /joint_states.

realtime_moveit.py did it by running a full MoveIt plan per marker update and
publishing only the last trajectory point, which is expensive and discards the
path. This uses the IK service directly, which is what pne_realtime.py did and
what the job actually needs.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerFeedback


class IkPoseFollower(Node):

    def __init__(self):
        super().__init__('ik_pose_follower')

        self.declare_parameter('group_name', 'arm')
        self.declare_parameter('ik_link_name', 'link5')
        self.declare_parameter('ik_timeout', 0.1)
        self.declare_parameter('use_marker_feedback', True)

        self.group_name = self.get_parameter('group_name').value
        self.ik_link_name = self.get_parameter('ik_link_name').value
        self.ik_timeout = self.get_parameter('ik_timeout').value

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.create_subscription(
            PoseStamped, 'interactive_marker_poses', self.on_pose, 10)

        if self.get_parameter('use_marker_feedback').value:
            # The rviz2 MoveIt display publishes marker drags here.
            self.create_subscription(
                InteractiveMarkerFeedback,
                'rviz_moveit_motion_planning_display/robot_interaction_'
                'interactive_marker_topic/feedback',
                self.on_marker_feedback, 10)

        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')

        # ROS 1 blocked in the constructor on rospy.wait_for_service, which in
        # ROS 2 would deadlock against the executor that has not started
        # spinning yet. Poll on a timer instead.
        self.ik_ready = False
        self.create_timer(1.0, self._check_ik_service)

        self.get_logger().info('ik_pose_follower up')

    def _check_ik_service(self):
        if self.ik_ready:
            return
        if self.ik_client.service_is_ready():
            self.ik_ready = True
            self.get_logger().info('compute_ik is available')
        else:
            self.get_logger().warn(
                'waiting for compute_ik — is move_group running?',
                throttle_duration_sec=10.0)

    def on_marker_feedback(self, feedback):
        pose = PoseStamped()
        pose.header = feedback.header
        pose.pose = feedback.pose
        self.on_pose(pose)

    def on_pose(self, pose):
        if not self.ik_ready:
            return

        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.ik_link_name
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = int(self.ik_timeout)
        request.ik_request.timeout.nanosec = int(
            (self.ik_timeout % 1.0) * 1e9)

        # Async, so a slow or failing solver cannot stall the executor the way
        # the ROS 1 blocking ServiceProxy call did.
        future = self.ik_client.call_async(request)
        future.add_done_callback(self._on_ik_response)

    def _on_ik_response(self, future):
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'compute_ik call failed: {exc}')
            return

        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().warn(
                f'IK failed, error code {response.error_code.val}',
                throttle_duration_sec=1.0)
            return

        solution = response.solution.joint_state
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = solution.name
        msg.position = solution.position
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IkPoseFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
