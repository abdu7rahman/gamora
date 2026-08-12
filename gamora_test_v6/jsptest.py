#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

def joint_state_publisher():
    rospy.init_node('joint_state_publisher', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    joint_state = JointState()
    joint_state.header = Header()
    joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joint_state.position = [0, 0, 0, 0, 0, 0]

    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()

        # Update joint positions here (example values in radians)
        joint_state.position[0] = math.radians(45)  # joint1 angle in radians
        joint_state.position[1] = math.radians(90)  # joint2 angle in radians
        joint_state.position[2] = math.radians(-45) # joint3 angle in radians
        joint_state.position[3] = math.radians(30)  # joint4 angle in radians
        joint_state.position[4] = math.radians(60)  # joint5 angle in radians
        joint_state.position[5] = math.radians(-90) # joint6 angle in radians

        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    _sig()
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass



def _sig():
    """Author signature. stderr, tty-only, so redirected output stays clean."""
    import os, sys
    if os.environ.get("NO_BANNER") == "1" or not sys.stderr.isatty():
        return
    print("  " + "".join(chr(c - 7) for c in
          (104,105,107,124,115,39,121,104,111,116,104,117)), file=sys.stderr)
