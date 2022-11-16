#include "spot_arm_interface/spot_arm_interface.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <spot_msgs/HandPose.h>

#include "spot_arm_interface/conversion.hpp"

namespace spot_arm_interface {

SpotArmInterface::SpotArmInterface()
    : nh("~"),
      body_to_origin(Eigen::Isometry3d::Identity()) {
    // Configuration
    origin_to_initial =
            Eigen::Translation3d(nh.param<double>("initial_pose/position/x", 0.7),
                    nh.param<double>("initial_pose/position/y", 0.0),
                    nh.param<double>("initial_pose/position/z", 0.5)) *
            Eigen::Quaterniond(nh.param<double>("initial_pose/orientation/w", 1.0),
                    nh.param<double>("initial_pose/orientation/x", 0.0), nh.param<double>("initial_pose/orientation/y", 0.0),
                    nh.param<double>("initial_pose/orientation/z", 0.0)).normalized();
    nh.param<std::string>("body_frame", body_frame, "base_link");
    nh.param<std::string>("hand_request_frame", hand_request_frame, "hand_request");
    nh.param<std::string>("body_origin_frame", body_origin_frame, "body_origin");
    nh.param<double>("move_duration_initial", move_duration_initial, 0.5);
    nh.param<double>("move_duration_tracking", move_duration_tracking, 0.1);
    nh.param<bool>("move_spot_body_enabled", move_spot_body_enabled, true);
    const Eigen::Vector3d hand_bbox_center{nh.param<double>("hand_bbox/center/x", origin_to_initial.translation()[0]),
            nh.param<double>("hand_bbox/center/y", origin_to_initial.translation()[1]),
            nh.param<double>("hand_bbox/center/z", origin_to_initial.translation()[2])};
    const Eigen::Vector3d hand_bbox_size{nh.param<double>("hand_bbox/size/x", std::numeric_limits<double>::max()),
            nh.param<double>("hand_bbox/size/y", std::numeric_limits<double>::max()),
            nh.param<double>("hand_bbox/size/z", std::numeric_limits<double>::max())};
    hand_bbox = BoundingBox3D(hand_bbox_center, hand_bbox_size);
    nh.param<bool>("spot_commands_enabled", spot_commands_enabled, true);

    // Subscriber to pose messages
    pose_subscriber = nh.subscribe<geometry_msgs::Pose>("input_pose_topic", 1,
            &SpotArmInterface::request_hand_pose_callback, this);

    // Subscriber to reset command topic
    reset_subscriber = nh.subscribe<geometry_msgs::Pose>("reset_robot_origin", 1,
            &SpotArmInterface::request_reset_callback, this);

    // Return to origin server
    return_to_origin_server =
            nh.advertiseService("return_to_origin", &SpotArmInterface::return_to_origin_callback, this);

    // Set service active
    set_spot_commands_active_server =
            nh.advertiseService("set_spot_commands_active", &SpotArmInterface::set_spot_commands_active_callback, this);

    // Spot body pose publisher
    spot_body_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/spot/go_to_pose", 1);

    // Send body pose tf
    const ros::Time stamp = ros::Time::now();
    publish_body_origin_tf(stamp);

    // Send arm to iniital pose T_B^I = T_B^O * T_O^I
    const auto request_pose = to_ros<geometry_msgs::Pose>(body_to_origin * origin_to_initial);
    publish_hand_pose_request_tf(request_pose, stamp);
    if (spot_commands_enabled) {
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

void SpotArmInterface::command_spot_to_pose(const ros::Time& stamp) {
    geometry_msgs::PoseStamped spot_body_pose;
    spot_body_pose.header.stamp = stamp;
    spot_body_pose.header.frame_id = body_origin_frame;
    spot_body_pose.pose = to_ros<geometry_msgs::Pose>(body_to_origin.inverse());
    spot_body_pose_publisher.publish(spot_body_pose);
}

void SpotArmInterface::publish_hand_pose_request_tf(const geometry_msgs::Pose& pose, const ros::Time& stamp) {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = body_frame;
    transform_stamped.child_frame_id = hand_request_frame;
    transform_stamped.transform.translation = point_to_vector(pose.position);
    transform_stamped.transform.rotation = pose.orientation;
    broadcaster.sendTransform(transform_stamped);
}

void SpotArmInterface::publish_hand_pose_request_tf(const geometry_msgs::Pose& pose) {
    publish_hand_pose_request_tf(pose, ros::Time::now());
}

void SpotArmInterface::publish_body_origin_tf(const ros::Time& stamp) {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = body_frame;
    transform_stamped.child_frame_id = body_origin_frame;
    transform_stamped.transform = to_ros<geometry_msgs::Transform>(body_to_origin);
    broadcaster.sendTransform(transform_stamped);
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
    // Timestamp
    const ros::Time stamp = ros::Time::now();

    // Convert pose to Eigen (T_I^N)
    const Eigen::Isometry3d initial_to_new =
            Eigen::Translation3d(pose->position.x, pose->position.y, pose->position.z) *
            Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z)
                    .normalized();

    // Transform relative to arm origin: T_O^N = T_O^I * reset transform * T_I^N
    const Eigen::Isometry3d origin_to_new = origin_to_initial * this->reset_t * initial_to_new;

    // Obtain command arm pose: T_B^N = T_B^O * T_O^N
    Eigen::Isometry3d body_to_new = body_to_origin * origin_to_new;

    // Check if invalid
    if (!hand_bbox.contains(body_to_new.translation())) {
        // Replace translation with closest valid one
        body_to_new.translation() = hand_bbox.closest_valid(body_to_new.translation());

        // Change the transformation from body to origin (T_B^O)
        if (move_spot_body_enabled) {
            // T_B^O = T_B^N * T_N^O
            body_to_origin = body_to_new * origin_to_new.inverse();

            // Publish TF (T_B^O)
            publish_body_origin_tf(stamp);

            // Send command to Spot (T_O^B) if services are enabled
            if (spot_commands_enabled) {
                command_spot_to_pose(stamp);
            }
        }
    }

    // Convert to ROS
    const geometry_msgs::Pose pose_request_msg = to_ros<geometry_msgs::Pose>(body_to_new);

    // Hand pose request
    publish_hand_pose_request_tf(pose_request_msg, stamp);
    if (spot_commands_enabled) {
        request_hand_pose(pose_request_msg, move_duration_tracking);
    }
}

void SpotArmInterface::request_reset_callback(const geometry_msgs::Pose::ConstPtr& pose) {
    // Timestamp
    const ros::Time stamp = ros::Time::now();

    const Eigen::Isometry3d initial_to_new =
            Eigen::Translation3d(pose->position.x, pose->position.y, pose->position.z) *
            Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z)
                    .normalized();
    
    this->reset_t = initial_to_new.inverse();

    // Transform relative to arm origin: T_O^N = T_O^I * reset transform * T_I^N
    const Eigen::Isometry3d origin_to_new = origin_to_initial;

    // Obtain command arm pose: T_B^N = T_B^O * T_O^N
    Eigen::Isometry3d body_to_new = body_to_origin * origin_to_new;

    // Check if invalid
    if (!hand_bbox.contains(body_to_new.translation())) {
        // Replace translation with closest valid one
        body_to_new.translation() = hand_bbox.closest_valid(body_to_new.translation());

        // Change the transformation from body to origin (T_B^O)
        if (move_spot_body_enabled) {
            // T_B^O = T_B^N * T_N^O
            body_to_origin = body_to_new * origin_to_new.inverse();

            // Publish TF (T_B^O)
            publish_body_origin_tf(stamp);

            // Send command to Spot (T_O^B) if services are enabled
            if (spot_commands_enabled) {
                command_spot_to_pose(stamp);
            }
        }
    }

    // Convert to ROS
    const geometry_msgs::Pose pose_request_msg = to_ros<geometry_msgs::Pose>(body_to_new);

    // Hand pose request
    publish_hand_pose_request_tf(pose_request_msg, stamp);
    if (spot_commands_enabled) {
        request_hand_pose(pose_request_msg, move_duration_initial);
    }




}

bool SpotArmInterface::return_to_origin_callback(std_srvs::Trigger::Request& request,
        std_srvs::Trigger::Response& response) {
    // Timestamp
    const ros::Time stamp = ros::Time::now();

    // Command spot back to pose
    if (move_spot_body_enabled) {
        publish_body_origin_tf(stamp);
        if (spot_commands_enabled) {
            command_spot_to_pose(stamp);
        }
    }

    // Send hand pose request
    const auto request_pose = to_ros<geometry_msgs::Pose>(body_to_origin * origin_to_initial);
    publish_hand_pose_request_tf(request_pose, stamp);
    if (spot_commands_enabled) {
        request_hand_pose(request_pose, move_duration_initial);
    }
    response.success = true;
    response.message = "Returned to origin";
    return true;
}

bool SpotArmInterface::set_spot_commands_active_callback(std_srvs::SetBool::Request& request,
        std_srvs::SetBool::Response& response) {
    spot_commands_enabled = request.data;
    response.success = true;
    response.message = (spot_commands_enabled ? "Service enabled!" : "Service disabled!");
    return true;
}

}
