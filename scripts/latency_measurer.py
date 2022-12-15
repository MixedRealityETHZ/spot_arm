import rospy
from sensor_msgs.msg import Image, CompressedImage

def callback(msg):
    time_diff = rospy.get_time() - msg.header.stamp.to_sec()
    with open("latency_uncompressed.txt", "a") as f:
        f.write(str(time_diff) + '\n')

    rospy.loginfo(time_diff)


if __name__ == '__main__':
    try:
        rospy.init_node('latency_measurer', anonymous=True)
        sub = rospy.Subscriber('/spot/camera/hand_color/image', Image, callback=callback, queue_size=1)
        # sub = rospy.Subscriber('/spot/camera/hand_color_repub/image/compressed', CompressedImage, callback=callback, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
