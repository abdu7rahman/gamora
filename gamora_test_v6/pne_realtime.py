#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayRobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState

class InteractiveMarkerControl:
    def __init__(self):
        rospy.init_node('interactive_marker_control')

        # Initialize subscribers and publishers
        rospy.Subscriber("/interactive_marker_poses", PoseStamped, self.marker_callback)
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Initialize IK service client
        rospy.wait_for_service('/compute_ik')
        self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    def marker_callback(self, pose):
        # Compute IK for the given pose
        ik_request = GetPositionIKRequest()
        ik_request.ik_request.group_name = "arm"
        ik_request.ik_request.pose_stamped = pose
        ik_response = self.ik_service(ik_request)

        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            # Publish joint states
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ik_response.solution.joint_state.name
            joint_state.position = ik_response.solution.joint_state.position
            self.joint_state_pub.publish(joint_state)
        else:
            rospy.logerr("IK computation failed")

if __name__ == '__main__':
    try:
        marker_control = InteractiveMarkerControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

