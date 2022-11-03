#ifndef SPOT_ARM_INTERFACE_SPOT_ARM_INTERFACE_HPP
#define SPOT_ARM_INTERFACE_SPOT_ARM_INTERFACE_HPP

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

namespace spot_arm_interface {

class SpotArmInterface {
public:
    SpotArmInterface();

private:
    void publish_hand_pose_request_tf(const geometry_msgs::Pose& pose);

    void request_hand_pose_callback(const geometry_msgs::Pose::ConstPtr& pose);

    void request_hand_pose(const Eigen::Isometry3d& pose);

    void request_hand_pose(const geometry_msgs::Pose& pose);

    ros::NodeHandle nh;
    ros::Subscriber pose_subscriber;
    ros::ServiceClient hand_pose_client;
    tf2_ros::TransformBroadcaster broadcaster;

    // Initial pose
    Eigen::Isometry3d initial_pose;
};

}

#endif
