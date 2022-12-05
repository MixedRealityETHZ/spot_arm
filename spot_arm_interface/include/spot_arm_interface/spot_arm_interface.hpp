#ifndef SPOT_ARM_INTERFACE_SPOT_ARM_INTERFACE_HPP
#define SPOT_ARM_INTERFACE_SPOT_ARM_INTERFACE_HPP

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <spot_msgs/TrajectoryAction.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <thread>

#include "spot_arm_interface/bounding_box.hpp"

namespace spot_arm_interface {

enum SpotMotionMode {
    STATIC,
    MOTION
};

SpotMotionMode to_spot_motion_mode(const std::string& spot_motion_mode);

struct AngleLimits {
    double roll_min{0.0};
    double roll_max{0.0};
    double pitch_min{0.0};
    double pitch_max{0.0};
    double yaw_min{0.0};
    double yaw_max{0.0};
};

class SpotArmInterface {
public:
    SpotArmInterface();

private:
    void command_spot_to_pose(const ros::Time& stamp);

    void publish_hand_pose_request_tf(const geometry_msgs::Pose& pose, const ros::Time& stamp);

    void publish_hand_pose_request_tf(const geometry_msgs::Pose& pose);

    void publish_body_origin_tf(const ros::Time& stamp);

    void report_loop();

    void request_hand_pose_callback(const geometry_msgs::Pose::ConstPtr& pose);

    void request_hand_pose(const geometry_msgs::Pose& pose, const double seconds);

    bool return_to_origin_callback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

    bool set_spot_commands_active_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

    void spot_trajectory_done(const actionlib::SimpleClientGoalState& state,
            const spot_msgs::TrajectoryResultConstPtr& result);

    void spot_trajectory_active();

    void spot_trajectory_feedback(const spot_msgs::TrajectoryFeedbackConstPtr& feedback);

    void request_reset_callback(const geometry_msgs::Pose::ConstPtr& pose);

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
    // Static TF broadcaster
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    // TF Listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    //// Reporting
    std::mutex report_mutex;
    std::thread report_loop_thread;
    int arm_command_success_count;
    int arm_command_failure_count;
    int body_command_count;

    //// Configuration
    // Initial pose (T_B^I)
    Eigen::Isometry3d body_to_initial;
    // Robot body frame
    std::string body_frame;
    // Hand request frame
    std::string hand_request_frame;
    // Odom frame
    std::string odom_frame;
    // Origin frame
    std::string body_origin_frame;
    // Move duration initial
    double move_duration_initial;
    // Move duration tracking
    double move_duration_tracking;
    // Move spot body enabled
    SpotMotionMode spot_motion_mode;
    // Hand bounding box
    BoundingBox3D hand_bbox;
    // Enable/disable spot commands
    bool spot_commands_enabled;
    // Reset flag (for reset on start)
    bool reset_on_next_pose;
    // Use body pose action interface
    bool action_interface;
    // Angle limits (ROTATION mode only)
    AngleLimits angle_limits;

    //// State
    // Spot body to origin pose (T_B^O)
    Eigen::Isometry3d body_to_origin;
    // Move duration
    bool move_to_reset;

    //// Reset
    // Listen to reset request
    ros::Subscriber reset_subscriber;
    // Initial pose (T_I^R)
    Eigen::Isometry3d initial_to_reset;
};

}

#endif
