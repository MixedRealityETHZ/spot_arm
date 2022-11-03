#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Pose
from tf.transformations import *


class UnityTransform:
    def __init__(self):
        rospy.init_node('unity_transform')
        rospy.Subscriber('/cmd_pos', Pose, self.transform)
        self.out = rospy.Publisher('/anything_i_like', Pose, queue_size=10)

    def transform(self, msg):
        quat1 = [-0.5, 0.5, 0.5, 0.5]
        quat2 = [msg.orientation.x, msg.orietnation.y, msg.orientation.z, msg.orientation.w]

        quat = quaternion_multiply(quat2, quat1)

        output_msg = Pose()
        output_msg.orientation.x = quat[0] 
        output_msg.orientation.y = quat[1] 
        output_msg.orientation.z = quat[2]  
        output_msg.orientation.w = quat[3] 
        
        output_msg.position.x = msg.position.x
        output_msg.position.y = msg.position.y
        output_msg.position.z = msg.position.z

        self.out.publish(output_msg)


if __name__ == '__main__':
    node = UnityTransform()
    node.spin()
