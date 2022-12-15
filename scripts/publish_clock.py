import rospy
from rosgraph_msgs.msg import Clock

def clock_publisher():
    rospy.init_node('clock_publisher', anonymous=True)
    pub = rospy.Publisher('clock', Clock, queue_size=10)
    rate = rospy.Rate(1000)
    clock = Clock()

    while not rospy.is_shutdown():
       clock.clock = rospy.Time.from_sec(rospy.get_time())
       pub.publish(clock)
       rate.sleep()

if __name__ == '__main__':
    try:
        clock_publisher()
    except rospy.ROSInterruptException:
        pass
