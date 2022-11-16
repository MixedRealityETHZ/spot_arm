#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

class UnityTransform:
    def __init__(self):
        rospy.init_node('unity_transform')
        rospy.Subscriber('input_pose', Pose, self.transform)
        self.out = rospy.Publisher('output_pose', Pose, queue_size=10)
        rospy.loginfo("Converting Unity poses to ROS poses")

    def transform(self, msg):
        output_msg = Pose()
        output_msg.orientation.x = -msg.orientation.z
        output_msg.orientation.y = msg.orientation.x
        output_msg.orientation.z = -msg.orientation.y
        output_msg.orientation.w = msg.orientation.w
        output_msg.position.x = msg.position.z
        output_msg.position.y = -msg.position.x
        output_msg.position.z = msg.position.y
        self.out.publish(output_msg)


if __name__ == '__main__':
    node = UnityTransform()
    rospy.spin()
