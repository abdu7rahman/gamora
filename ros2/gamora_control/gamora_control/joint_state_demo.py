#!/usr/bin/env python3
"""Publish a fixed joint configuration on /joint_states at 10 Hz.

ROS 2 port of gamora_test_v6/jsptest.py. Useful for checking that the URDF and
rviz display line up without running a controller.
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class JointStateDemo(Node):

    def __init__(self):
        super().__init__('joint_state_demo')

        # The ROS 1 script hard-coded five names and six values, so joint6 —
        # which this arm does not have — was published while joint_leff and
        # joint_reff were not. Names come from the URDF now.
        self.declare_parameter(
            'joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5',
             'joint_leff', 'joint_reff'])
        self.declare_parameter(
            'positions_degrees', [45.0, 90.0, -45.0, 30.0, 60.0, 0.0, 0.0])
        self.declare_parameter('publish_rate', 10.0)

        self.joint_names = self.get_parameter('joint_names').value
        degrees = self.get_parameter('positions_degrees').value

        if len(degrees) != len(self.joint_names):
            self.get_logger().warn(
                f'{len(self.joint_names)} joint names but {len(degrees)} '
                'positions; padding the shortfall with zeros')
            degrees = list(degrees) + [0.0] * (len(self.joint_names) - len(degrees))

        self.positions = [math.radians(d) for d in degrees[:len(self.joint_names)]]

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self.publish_state)

        self.get_logger().info(f'publishing {self.joint_names} at {rate} Hz')

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateDemo()
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
