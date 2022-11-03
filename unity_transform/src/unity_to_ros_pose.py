#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

class UnityTransform:
    def __init__(self):
        rospy.init_node('unity_transform')
        input_pose_topic = '/cam_pos'
        rospy.Subscriber(input_pose_topic, Pose, self.transform)
        output_pose_topic = '/cam_pos_trans'
        self.out = rospy.Publisher(output_pose_topic, Pose, queue_size=10)
        rospy.loginfo("Converting Unity poses on " + input_pose_topic + " to ROS poses on " + output_pose_topic)

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
