#ifndef SPOT_ARM_INTERFACE_SPOT_ARM_INTERFACE_HPP
#define SPOT_ARM_INTERFACE_SPOT_ARM_INTERFACE_HPP

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace spot_arm_interface {

class SpotArmInterface {
public:
    SpotArmInterface();

private:
    void request_hand_pose(const geometry_msgs::Pose::ConstPtr& pose_stamped);

    ros::NodeHandle nh;
    ros::Subscriber pose_subscriber;
    ros::ServiceClient hand_pose_client;
};

}

#endif
