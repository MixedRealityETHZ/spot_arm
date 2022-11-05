#ifndef SPOT_ARM_INTERFACE_SPOT_ARM_INTERFACE_HPP
#define SPOT_ARM_INTERFACE_SPOT_ARM_INTERFACE_HPP

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>


namespace spot_arm_interface {

class SpotArmInterface {
public:
    SpotArmInterface();

private:
    void publish_hand_pose_request_tf(const geometry_msgs::Pose& pose);

    void request_hand_pose_callback(const geometry_msgs::Pose::ConstPtr& pose);

    void request_hand_pose(const geometry_msgs::Pose& pose, const double seconds);

    bool return_to_origin_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    
    bool set_service_active_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

    //// ROS
    ros::NodeHandle nh;
    ros::Subscriber pose_subscriber;
    ros::ServiceClient hand_pose_client;
    ros::ServiceServer return_to_origin_server;
    ros::ServiceServer set_service_active_server;
    tf2_ros::TransformBroadcaster broadcaster;

    //// Configuration
    // Initial pose
    Eigen::Isometry3d initial_pose;
    // Robot body frame
    std::string robot_body_frame;
    // Hand request frame
    std::string hand_request_frame;
    // Move duration initial
    double move_duration_initial;
    // Move duration tracking
    double move_duration_tracking;
    // Enable/disable service
    bool service_enabled;
};

}

#endif
