#include "spot_arm_interface/spot_arm_interface.hpp"

#include <spot_msgs/HandPose.h>

namespace spot_arm_interface {

SpotArmInterface::SpotArmInterface()
    : nh("~") {
    // Subscriber to pose messages
    pose_subscriber = nh.subscribe<geometry_msgs::Pose>("input_poses", 1, &SpotArmInterface::request_hand_pose, this);

    // Create service client
    const std::string hand_pose_service_name{"/spot/gripper_pose"};
    hand_pose_client = nh.serviceClient<spot_msgs::HandPose>(hand_pose_service_name);
    ROS_INFO_STREAM("Waiting for HandPose service \'" << hand_pose_service_name << "\' to come online...");
    if (!hand_pose_client.waitForExistence()) {
        throw std::runtime_error("Failed to connect to HandPose service \'" + hand_pose_service_name + "\'");
    }
    ROS_INFO_STREAM("Connected to HandPose service  \'" << hand_pose_service_name << "\'!");
    ROS_INFO_STREAM("Moving to starting pose.");

    spot_msgs::HandPose::Request request;
    initial_pose = Eigen::Translation3d(0.7, 0.0, 0.5) * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    request.pose_point.position.x = 0.7;
    request.pose_point.position.y = 0.0;
    request.pose_point.position.z = 0.5;
    request.pose_point.orientation.w = 1.0;
    request.pose_point.orientation.x = 0.0;
    request.pose_point.orientation.y = 0.0;
    request.pose_point.orientation.z = 0.0;
    spot_msgs::HandPose::Response response;
    if (!hand_pose_client.call(request, response)) {
        ROS_ERROR_STREAM("Failed to call service. Service validity: " << hand_pose_client.isValid() ? "True" : "False");
    }
    ROS_INFO_STREAM("Finished moving to starting pose.");
}

void SpotArmInterface::request_hand_pose(const geometry_msgs::Pose::ConstPtr& pose) {
    spot_msgs::HandPose::Request request;
    const Eigen::Isometry3d request_pose =
            Eigen::Translation3d(pose->position.x, pose->position.y, pose->position.z) *
            Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z);
    // T_I^N = T_I^O * T_O^N
    const Eigen::Isometry3d request_pose_transformed = initial_pose.inverse() * request_pose;
    const Eigen::Vector3d t = request_pose_transformed.translation();
    request.pose_point.position.x = t[0];
    request.pose_point.position.y = t[1];
    request.pose_point.position.z = t[2];
    const Eigen::Quaterniond q = Eigen::Quaterniond{request_pose_transformed.rotation()};
    request.pose_point.orientation.w = q.w();
    request.pose_point.orientation.x = q.x();
    request.pose_point.orientation.y = q.y();
    request.pose_point.orientation.z = q.z();
    // request.pose_point = *pose;
    spot_msgs::HandPose::Response response;
    if (!hand_pose_client.call(request, response)) {
        ROS_ERROR_STREAM("Failed to call service. Service validity: " << hand_pose_client.isValid() ? "True" : "False");
    }
    if (!response.success) {
        ROS_WARN_STREAM("Service was not successful. Message: \"" << response.message << "\"");
    }
}

}
