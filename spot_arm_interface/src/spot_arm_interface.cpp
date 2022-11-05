#include "spot_arm_interface/spot_arm_interface.hpp"

#include <spot_msgs/HandPose.h>

#include "spot_arm_interface/conversion.hpp"

namespace spot_arm_interface {

SpotArmInterface::SpotArmInterface()
    : nh("~") {
    // Configuration
    initial_pose =
            Eigen::Translation3d(nh.param<double>("initial_pose/position/x", 0.7),
                    nh.param<double>("initial_pose/position/y", 0.0),
                    nh.param<double>("initial_pose/position/z", 0.5)) *
            Eigen::Quaterniond(nh.param<double>("initial_pose/position/w", 1.0),
                    nh.param<double>("initial_pose/position/x", 0.0), nh.param<double>("initial_pose/position/y", 0.0),
                    nh.param<double>("initial_pose/position/z", 0.0));
    nh.param<std::string>("robot_body_frame", robot_body_frame, "base_link");
    nh.param<std::string>("hand_request_frame", hand_request_frame, "hand_request");
    nh.param<double>("move_duration_initial", move_duration_initial, 0.5);
    nh.param<double>("move_duration_tracking", move_duration_tracking, 0.1);
    nh.param<bool>("disable_service", disable_service, false);

    // Subscriber to pose messages
    pose_subscriber = nh.subscribe<geometry_msgs::Pose>("input_pose_topic", 1,
            &SpotArmInterface::request_hand_pose_callback, this);

    // Return to origin server
    return_to_origin_server = nh.advertiseService("return_to_origin", &SpotArmInterface::return_to_origin, this);

    // Initial pose
    const auto request_pose = to_ros<geometry_msgs::Pose>(initial_pose);
    publish_hand_pose_request_tf(request_pose);
    if (!disable_service) {
        // Create service client
        const std::string hand_pose_service_name{"/spot/gripper_pose"};
        hand_pose_client = nh.serviceClient<spot_msgs::HandPose>(hand_pose_service_name);
        ROS_INFO_STREAM("Waiting for HandPose service \'" << hand_pose_service_name << "\' to come online...");
        if (!hand_pose_client.waitForExistence()) {
            throw std::runtime_error("Failed to connect to HandPose service \'" + hand_pose_service_name + "\'");
        }
        ROS_INFO_STREAM("Connected to HandPose service  \'" << hand_pose_service_name << "\'!");

        // Move to initial pose
        ROS_INFO_STREAM("Moving to starting pose.");
        request_hand_pose(request_pose, move_duration_initial);
    }
    ROS_INFO_STREAM("Ready to receive pose commands!");
}

void SpotArmInterface::publish_hand_pose_request_tf(const geometry_msgs::Pose& pose) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = robot_body_frame;
    transformStamped.child_frame_id = hand_request_frame;
    transformStamped.transform.translation.x = pose.position.x;
    transformStamped.transform.translation.y = pose.position.y;
    transformStamped.transform.translation.z = pose.position.z;
    transformStamped.transform.rotation.w = pose.orientation.w;
    transformStamped.transform.rotation.x = pose.orientation.x;
    transformStamped.transform.rotation.y = pose.orientation.y;
    transformStamped.transform.rotation.z = pose.orientation.z;
    broadcaster.sendTransform(transformStamped);
}

void SpotArmInterface::request_hand_pose(const geometry_msgs::Pose& pose, const double seconds) {
    spot_msgs::HandPose::Request request;
    request.pose_point = pose;
    request.seconds = seconds;
    spot_msgs::HandPose::Response response;
    if (!hand_pose_client.call(request, response)) {
        ROS_ERROR_STREAM("Failed to call service. Service validity: " << hand_pose_client.isValid() ? "True" : "False");
    }
    if (!response.success) {
        ROS_WARN_STREAM("Service was not successful. Message: \"" << response.message << "\"");
    }
}

void SpotArmInterface::request_hand_pose_callback(const geometry_msgs::Pose::ConstPtr& pose) {
    // Convert to Eigen
    const Eigen::Isometry3d pose_ =
            Eigen::Translation3d(pose->position.x, pose->position.y, pose->position.z) *
            Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z);

    // Transform relative to arm origin: T_O^N = T_O^I * T_I^N
    const auto request_pose = to_ros<geometry_msgs::Pose>(initial_pose * pose_);

    // Hand pose request
    publish_hand_pose_request_tf(request_pose);
    if (!disable_service) {
        request_hand_pose(request_pose, move_duration_tracking);
    }
}

bool SpotArmInterface::return_to_origin(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    const auto request_pose = to_ros<geometry_msgs::Pose>(initial_pose);
    publish_hand_pose_request_tf(request_pose);
    if (!disable_service) {
        request_hand_pose(request_pose, move_duration_initial);
    }
    response.success = true;
    response.message = "Returned to origin";
    return true;
}

}
