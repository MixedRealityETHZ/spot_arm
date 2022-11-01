#include "spot_arm_interface/spot_arm_interface.hpp"

#include <spot_msgs/HandPose.h>

namespace spot_arm_interface {

SpotArmInterface::SpotArmInterface()
    : nh("~") {
    // Subscriber to pose messages
    pose_subscriber =
            nh.subscribe<geometry_msgs::Pose>("input_poses", 1, &SpotArmInterface::request_hand_pose, this);

    // Create service client
    const std::string hand_pose_service_name{"/spot/gripper_pose"};
    hand_pose_client = nh.serviceClient<spot_msgs::HandPose>(hand_pose_service_name);
    ROS_INFO_STREAM("Waiting for HandPose service \'" << hand_pose_service_name << "\' to come online...");
    if (!hand_pose_client.waitForExistence()) {
        throw std::runtime_error("Failed to connect to HandPose service \'" + hand_pose_service_name + "\'");
    }
    ROS_INFO_STREAM("Connected to HandPose service  \'" << hand_pose_service_name << "\'!");
}

void SpotArmInterface::request_hand_pose(const geometry_msgs::Pose::ConstPtr& pose) {
    spot_msgs::HandPose::Request request;
    request.pose_point = *pose;
    spot_msgs::HandPose::Response response;
    if (!hand_pose_client.call(request, response)) {
        ROS_ERROR_STREAM("Failed to call service. Service validity: " << hand_pose_client.isValid() ? "True" : "False");
    }
    if (!response.success) {
        ROS_WARN_STREAM("Service was not successful. Message: \"" << response.message << "\"");
    }
}

}
