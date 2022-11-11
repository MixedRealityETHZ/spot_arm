#ifndef SPOT_ARM_INTERFACE_SPOT_ARM_INTERFACE_HPP
#define SPOT_ARM_INTERFACE_SPOT_ARM_INTERFACE_HPP

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

#include <spot_msgs/TrajectoryAction.h>

#include "spot_arm_interface/bounding_box.hpp"

namespace spot_arm_interface {

class SpotArmInterface {
public:
    SpotArmInterface();

private:
    void command_spot_to_pose(const ros::Time& stamp);

    void publish_hand_pose_request_tf(const geometry_msgs::Pose& pose, const ros::Time& stamp);

    void publish_hand_pose_request_tf(const geometry_msgs::Pose& pose);

    void publish_body_origin_tf(const ros::Time& stamp);

    void request_hand_pose_callback(const geometry_msgs::Pose::ConstPtr& pose);

    void request_hand_pose(const geometry_msgs::Pose& pose, const double seconds);

    bool return_to_origin_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

    bool set_spot_commands_active_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

    void spot_trajectory_done(const actionlib::SimpleClientGoalState& state,
            const spot_msgs::TrajectoryResultConstPtr& result);

    //// ROS
    ros::NodeHandle nh;
    // Listen to hand pose requests
    ros::Subscriber pose_subscriber;
    // Hand pose request client
    ros::ServiceClient hand_pose_client;
    // Return to origin server
    ros::ServiceServer return_to_origin_server;
    // Disable/Enable spot commands
    ros::ServiceServer set_spot_commands_active_server;
    // Spot body pose publisher
    ros::Publisher spot_body_pose_publisher;
    // Spot body pose action client
    actionlib::SimpleActionClient<spot_msgs::TrajectoryAction> spot_body_pose_client;
    // TF broadcaster
    tf2_ros::TransformBroadcaster broadcaster;

    //// Configuration
    // Initial pose (T_O^I)
    Eigen::Isometry3d origin_to_initial;
    // Robot body frame
    std::string body_frame;
    // Hand request frame
    std::string hand_request_frame;
    // Origin frame
    std::string body_origin_frame;
    // Move duration initial
    double move_duration_initial;
    // Move duration tracking
    double move_duration_tracking;
    // Move spot body enabled
    bool move_spot_body_enabled;
    // Hand bounding box
    BoundingBox3D hand_bbox;
    // Enable/disable spot commands
    bool spot_commands_enabled;

    //// State
    // Spot body to origin pose (T_B^O)
    Eigen::Isometry3d body_to_origin;
};

}

#endif
