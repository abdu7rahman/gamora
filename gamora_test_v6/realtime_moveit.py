#!/usr/bin/env python

import rospy
from visualization_msgs.msg import InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_commander.conversions import pose_to_list

class RealTimeController:
    def __init__(self):
        rospy.init_node('interactive_marker_realtime_control', anonymous=True)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("arm")
        self.group.set_planning_time(20)  # Increase the planning time
        self.group.set_num_planning_attempts(10)

        self.joint_state_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.marker_subscriber = rospy.Subscriber('/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback', InteractiveMarkerFeedback, self.marker_feedback_callback)

        rospy.loginfo("RealTimeController initialized")

    def marker_feedback_callback(self, feedback):
        rospy.loginfo("Received interactive marker feedback")
        pose = PoseStamped()
        pose.header = feedback.header
        pose.pose = feedback.pose

        rospy.loginfo(f"Marker pose: {pose_to_list(pose.pose)}")

        # Set the target pose for MoveIt!
        self.group.set_pose_target(pose)

        # Plan to the new pose
        success, plan, planning_time, error_code = self.group.plan()

        if success:
            rospy.loginfo("Planning successful")
            if plan.joint_trajectory.points:
                joint_values = plan.joint_trajectory.points[-1].positions

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = rospy.Time.now()
                joint_state_msg.name = self.group.get_active_joints()
                joint_state_msg.position = joint_values

                # Publish the joint states
                self.joint_state_publisher.publish(joint_state_msg)

                # Print the joint states for debugging
                rospy.loginfo("Publishing joint states:")
                for name, value in zip(joint_state_msg.name, joint_state_msg.position):
                    rospy.loginfo(f"{name}: {value}")
            else:
                rospy.logwarn("No points in joint trajectory")
        else:
            rospy.logwarn(f"Planning failed with error code: {error_code}")

if __name__ == '__main__':
    controller = RealTimeController()
    rospy.spin()

