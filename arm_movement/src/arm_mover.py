#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import JointState
from math import radians
from spot_ros.srv import ArmJointMovement
#from spot_msg.srv import ArmJointMovement


class ArmMover:
    def __init__(self):
        self.command = rospy.Subscriber('command', Int16, self.command_exec)
        self.joint_state = rospy.Subscriber('joint_states', JointState, self.state_save)

        self.current_state = [] 
    
    def state_save(self, received_msg):
        self.current_state = received_msg.position[12:-1] 
        # 'arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5', 'arm_joint6'

    def command_exec(self, msg):
        new_position = self.current_state
        new_position[-1] += radians(msg.data)

        service_name = '/spot/arm_joint_move'
        rospy.wait_for_service(service_name)
        try:
            move = rospy.ServiceProxy(service_name, ArmJointMovement)
            response = move(new_position)
        except rospy.ServiceException as e:
            print(f"Service call failes: {e}")
        
    def run(self):
        while not tospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('arm_mover')

    mover = ArmMover()

    try:
        mover.run()
    except rospy.ROSInterruptException:
        pass
