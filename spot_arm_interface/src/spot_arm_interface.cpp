#include "spot_arm_interface/spot_arm_interface.hpp"

#include <spot_msgs/HandPose.h>

namespace spot_arm_interface {

SpotArmInterface::SpotArmInterface()
    : nh("~") {
    // Subscriber to pose messages
    pose_subscriber = nh.subscribe<geometry_msgs::Pose>("input_poses", 1, &SpotArmInterface::request_hand_pose_callback, this);

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
    request_hand_pose(initial_pose);
}

void SpotArmInterface::publish_hand_pose_request_tf(const geometry_msgs::Pose& pose) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "hand_pose_request";
    transformStamped.transform.translation.x = pose.position.x;
    transformStamped.transform.translation.y = pose.position.y;
    transformStamped.transform.translation.z = pose.position.z;
    transformStamped.transform.rotation.w = pose.orientation.w;
    transformStamped.transform.rotation.x = pose.orientation.x;
    transformStamped.transform.rotation.y = pose.orientation.y;
    transformStamped.transform.rotation.z = pose.orientation.z;
    broadcaster.sendTransform(transformStamped);
}

void SpotArmInterface::request_hand_pose(const Eigen::Isometry3d& pose) {
    geometry_msgs::Pose pose_;
    const Eigen::Vector3d t = pose.translation();
    const Eigen::Quaterniond q = Eigen::Quaterniond{pose.rotation()};
    pose_.position.x = t[0];
    pose_.position.y = t[1];
    pose_.position.z = t[2];
    pose_.orientation.w = q.w();
    pose_.orientation.x = q.x();
    pose_.orientation.y = q.y();
    pose_.orientation.z = q.z();
    request_hand_pose(pose_);
}

void SpotArmInterface::request_hand_pose(const geometry_msgs::Pose& pose) {
    publish_hand_pose_request_tf(pose);
    spot_msgs::HandPose::Request request;
    request.pose_point = pose;
    spot_msgs::HandPose::Response response;
    if (!hand_pose_client.call(request, response)) {
        ROS_ERROR_STREAM("Failed to call service. Service validity: " << hand_pose_client.isValid() ? "True" : "False");
    }
    if (!response.success) {
        ROS_WARN_STREAM("Service was not successful. Message: \"" << response.message << "\"");
    }
}

void SpotArmInterface::request_hand_pose_callback(const geometry_msgs::Pose::ConstPtr& pose) {
    spot_msgs::HandPose::Request request;
    const Eigen::Isometry3d request_pose =
            Eigen::Translation3d(pose->position.x, pose->position.y, pose->position.z) *
            Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z);
    // T_I^N = T_I^O * T_O^N
    const Eigen::Isometry3d request_pose_transformed = initial_pose.inverse() * request_pose;
    request_hand_pose(request_pose_transformed);
}

}
