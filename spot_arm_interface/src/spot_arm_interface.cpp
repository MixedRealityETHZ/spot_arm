#include "spot_arm_interface/spot_arm_interface.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <spot_msgs/HandPose.h>

#include <cmath>

#include "spot_arm_interface/conversion.hpp"

namespace spot_arm_interface {

SpotMotionMode to_spot_motion_mode(const std::string& spot_motion_mode) {
    if (spot_motion_mode == "STATIC") {
        return SpotMotionMode::STATIC;
    } else if (spot_motion_mode == "MOTION") {
        return SpotMotionMode::MOTION;
    } else {
        throw std::runtime_error("Unhandled spot motion mode \"" + spot_motion_mode + "\"");
    }
}

SpotArmInterface::SpotArmInterface()
    : nh("~"),
      spot_body_pose_client("/spot/trajectory", true),
      tf_listener(tf_buffer),
      arm_command_success_count(0),
      arm_command_failure_count(0),
      body_command_count(0),
      move_to_reset(false),
      initial_to_reset(Eigen::Isometry3d::Identity()),
      body_to_origin(Eigen::Isometry3d::Identity()) {
    // Configuration
    body_to_initial = Eigen::Translation3d(nh.param<double>("initial_pose/position/x", 0.7),
                              nh.param<double>("initial_pose/position/y", 0.0),
                              nh.param<double>("initial_pose/position/z", 0.5)) *
                      Eigen::Quaterniond(nh.param<double>("initial_pose/orientation/w", 1.0),
                              nh.param<double>("initial_pose/orientation/x", 0.0),
                              nh.param<double>("initial_pose/orientation/y", 0.0),
                              nh.param<double>("initial_pose/orientation/z", 0.0))
                              .normalized();
    nh.param<std::string>("body_frame", body_frame, "base_link");
    nh.param<std::string>("hand_request_frame", hand_request_frame, "hand_request");
    nh.param<std::string>("odom_frame", odom_frame, "odom");
    nh.param<std::string>("body_origin_frame", body_origin_frame, "body_origin");
    nh.param<double>("move_duration_initial", move_duration_initial, 0.5);
    nh.param<double>("move_duration_tracking", move_duration_tracking, 0.1);
    spot_motion_mode = to_spot_motion_mode(nh.param<std::string>("spot_motion_mode", "STATIC"));
    const std::string body_pose_method = nh.param<std::string>("body_pose_method", "ACTION");
    if (body_pose_method == "ACTION") {
        action_interface = true;
    } else if (body_pose_method == "TOPIC") {
        action_interface = false;
    } else {
        throw std::runtime_error("Unhandled body_pose_method " + body_pose_method + " (should be ACTION or TOPIC)");
    }
    nh.param<bool>("reset_on_start", reset_on_next_pose, true);
    const Eigen::Vector3d hand_bbox_size{nh.param<double>("hand_bbox/size/x", std::numeric_limits<double>::max()),
            nh.param<double>("hand_bbox/size/y", std::numeric_limits<double>::max()),
            nh.param<double>("hand_bbox/size/z", std::numeric_limits<double>::max())};
    hand_bbox = BoundingBox3D(body_to_initial.translation(), hand_bbox_size);
    nh.param<bool>("spot_commands_enabled", spot_commands_enabled, true);
    angle_limits.roll_min = (M_PI / 180.0) * nh.param<double>("angle_limits/roll/min", -180.0);
    angle_limits.roll_max = (M_PI / 180.0) * nh.param<double>("angle_limits/roll/max", 180.0);
    angle_limits.pitch_min = (M_PI / 180.0) * nh.param<double>("angle_limits/pitch/min", -180.0);
    angle_limits.pitch_max = (M_PI / 180.0) * nh.param<double>("angle_limits/pitch/max", 180.0);
    angle_limits.yaw_min = (M_PI / 180.0) * nh.param<double>("angle_limits/yaw/min", -180.0);
    angle_limits.yaw_max = (M_PI / 180.0) * nh.param<double>("angle_limits/yaw/max", 180.0);

    // Subscriber to pose messages
    pose_subscriber = nh.subscribe<geometry_msgs::Pose>("input_pose_topic", 1,
            &SpotArmInterface::request_hand_pose_callback, this);

    // Subscriber to reset command topic
    reset_subscriber =
            nh.subscribe<geometry_msgs::Pose>("input_reset_topic", 1, &SpotArmInterface::request_reset_callback, this);

    // Return to origin server
    return_to_origin_server =
            nh.advertiseService("return_to_origin", &SpotArmInterface::return_to_origin_callback, this);

    // Set service active
    set_spot_commands_active_server =
            nh.advertiseService("set_spot_commands_active", &SpotArmInterface::set_spot_commands_active_callback, this);

    // Spot body pose publisher
    spot_body_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/spot/go_to_pose", 1);

    // Set up reporting thread
    report_loop_thread = std::thread(boost::bind(&SpotArmInterface::report_loop, this));

    // Send body pose tf
    const ros::Time stamp = ros::Time::now();
    publish_body_origin_tf(stamp);

    // Send arm to iniital pose T_B^I
    const auto request_pose = to_ros<geometry_msgs::Pose>(body_to_initial);
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

        // Wait for spot body movement service
        if (spot_motion_mode != SpotMotionMode::STATIC) {
            ROS_INFO_STREAM("Waiting for Spot trajectory server.");
            spot_body_pose_client.waitForServer();
            ROS_INFO_STREAM("Acquired Spot trajectory server");
        }
    }
    ROS_INFO_STREAM("Ready to receive pose commands!");
}

void SpotArmInterface::command_spot_to_pose(const ros::Time& stamp) {
    geometry_msgs::PoseStamped spot_body_pose;
    spot_body_pose.header.stamp = stamp;
    spot_body_pose.header.frame_id = body_origin_frame;
    spot_body_pose.pose = to_ros<geometry_msgs::Pose>(body_to_origin.inverse());
    ROS_INFO_STREAM("Sending spot to [xyz], (wxyz): ["
                    << spot_body_pose.pose.position.x << ", " << spot_body_pose.pose.position.y << ", "
                    << spot_body_pose.pose.position.z << "], [" << spot_body_pose.pose.orientation.w << ", "
                    << spot_body_pose.pose.orientation.x << ", " << spot_body_pose.pose.orientation.y << ", "
                    << spot_body_pose.pose.orientation.z << "]");

    if (action_interface) {
        // Action interface
        spot_msgs::TrajectoryGoal spot_body_pose_goal;
        spot_body_pose_goal.target_pose = spot_body_pose;
        spot_body_pose_goal.duration.data = ros::Duration(5);
        spot_body_pose_goal.precise_positioning = true;
        spot_body_pose_client.sendGoal(spot_body_pose_goal,
                std::bind(&SpotArmInterface::spot_trajectory_done, this, std::placeholders::_1, std::placeholders::_2));
    } else {
        // Topic interface
        spot_body_pose_publisher.publish(spot_body_pose);
    }

    // Report
    report_mutex.lock();
    ++body_command_count;
    report_mutex.unlock();
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
    geometry_msgs::TransformStamped transform_stamped =
            tf_buffer.lookupTransform(odom_frame, body_frame, ros::Time(0), ros::Duration(1));
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = odom_frame;
    transform_stamped.child_frame_id = body_origin_frame;
    static_broadcaster.sendTransform(transform_stamped);
}

void SpotArmInterface::report_loop() {
    // Set frequency
    ros::Rate rate(1);

    // Main loop
    while (ros::ok()) {
        // Report number of arm and body poses sent
        report_mutex.lock();
        ROS_INFO_STREAM("In last " << rate.expectedCycleTime().toSec() << " s: [Arm Cmd Successes = "
                                   << arm_command_success_count << ", Arm Cmd Failures = " << arm_command_failure_count
                                   << ", Body Cmds = " << body_command_count << "]");
        arm_command_success_count = 0;
        arm_command_failure_count = 0;
        body_command_count = 0;
        report_mutex.unlock();

        // Sleep the thread
        rate.sleep();

        // Check loop time
        if (rate.cycleTime() < rate.expectedCycleTime()) {
            ROS_DEBUG_STREAM("Loop completed in " << rate.cycleTime().toSec()
                                                  << " seconds (required f = " << 1.0 / rate.expectedCycleTime().toSec()
                                                  << ", capable f = " << 1.0 / rate.cycleTime().toSec()
                                                  << " Hz). Sleeping thread.");
        } else {
            ROS_WARN_STREAM("SLOW WARNING: Loop completed in "
                            << rate.cycleTime().toSec()
                            << " seconds (required f = " << 1.0 / rate.expectedCycleTime().toSec()
                            << ", capable f = " << 1.0 / rate.cycleTime().toSec() << " Hz). Sleeping thread.");
        }
    }
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

    // Report
    report_mutex.lock();
    arm_command_success_count += response.success ? 1 : 0;
    arm_command_failure_count += response.success ? 0 : 1;
    report_mutex.unlock();
}

void SpotArmInterface::request_hand_pose_callback(const geometry_msgs::Pose::ConstPtr& pose) {
    // Reset if reset flag is set (such as on start up)
    if (reset_on_next_pose) {
        request_reset_callback(pose);
        reset_on_next_pose = false;
    }

    // Timestamp
    const ros::Time stamp = ros::Time::now();

    // Convert pose to Eigen, as origin (external) to external
    const Eigen::Isometry3d origin_ext_to_ext =
            Eigen::Translation3d(pose->position.x, pose->position.y, pose->position.z) *
            Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z)
                    .normalized();

    // Obtain command arm pose: T_R^N = T_B^O * T_OE^E
    Eigen::Isometry3d reset_to_new = body_to_origin * origin_ext_to_ext;

    // T_B^N = T_B^I * T_I^R * T_R^N
    Eigen::Isometry3d body_to_new = body_to_initial * initial_to_reset * reset_to_new;

    // Check if invalid
    if (!hand_bbox.contains(body_to_new.translation())) {
        // Replace translation with closest valid one
        body_to_new.translation() = hand_bbox.closest_valid(body_to_new.translation());
    }

    // Ensure within angle limits
    Eigen::Vector3d euler_angles = body_to_new.rotation().eulerAngles(0, 1, 2);
    euler_angles[0] = std::min(std::max(euler_angles[0], angle_limits.roll_min), angle_limits.roll_max);
    euler_angles[1] = std::min(std::max(euler_angles[1], angle_limits.pitch_min), angle_limits.pitch_max);
    euler_angles[2] = std::min(std::max(euler_angles[2], angle_limits.yaw_min), angle_limits.yaw_max);
    const Eigen::Quaterniond new_rotation = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX()) *
                                            Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ());
    body_to_new = Eigen::Translation<double, 3>(body_to_new.translation()) * new_rotation;
    
    // T_R^N = T_R^I * T_I^B * T_B^N
    reset_to_new = initial_to_reset.inverse() * body_to_initial.inverse() * body_to_new;

    // Change the transformation from body to origin (T_B^O)
    if (spot_motion_mode == SpotMotionMode::MOTION) {
        // T_B^O = T_R^N * T_OE^O
        body_to_origin = reset_to_new * origin_ext_to_ext.inverse();

        // Send command to Spot (T_O^B) if services are enabled
        if (spot_commands_enabled) {
            command_spot_to_pose(stamp);
        }
    }

    // Compute transform for arm: T_B^N = T_B^I * T_I^R * T_R^N
    body_to_new = body_to_initial * initial_to_reset * reset_to_new;

    // Convert to ROS
    const geometry_msgs::Pose pose_request_msg = to_ros<geometry_msgs::Pose>(body_to_new);

    // Hand pose request
    publish_hand_pose_request_tf(pose_request_msg, stamp);
    if (spot_commands_enabled) {
        request_hand_pose(pose_request_msg, move_to_reset ? move_duration_initial : move_duration_tracking);
    }
    move_to_reset = false;
}

void SpotArmInterface::request_reset_callback(const geometry_msgs::Pose::ConstPtr& pose) {
    const Eigen::Isometry3d origin_ext_to_ext =
            Eigen::Translation3d(pose->position.x, pose->position.y, pose->position.z) *
            Eigen::Quaterniond(pose->orientation.w, pose->orientation.x, pose->orientation.y, pose->orientation.z)
                    .normalized();
    // T_I^R = (T_B^{BO} T_OE^E)^{-1}
    initial_to_reset = (body_to_origin * origin_ext_to_ext).inverse();
    ROS_INFO_STREAM("Resetting arm to initial position. T_I^R = \n" << initial_to_reset.matrix());
    move_to_reset = true;
}

bool SpotArmInterface::return_to_origin_callback(std_srvs::Trigger::Request& request,
        std_srvs::Trigger::Response& response) {
    // Timestamp
    const ros::Time stamp = ros::Time::now();

    // Command spot back to pose
    if (spot_motion_mode == SpotMotionMode::MOTION) {
        if (spot_commands_enabled) {
            command_spot_to_pose(stamp);
        }
    }

    // Send hand pose request
    const auto request_pose = to_ros<geometry_msgs::Pose>(body_to_initial);
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

void SpotArmInterface::spot_trajectory_done(const actionlib::SimpleClientGoalState& state,
        const spot_msgs::TrajectoryResultConstPtr& result) {
    ROS_INFO_STREAM("Trajectory action finished ("
                    << state.toString() << ") with result: [success = " << (result->success ? "TRUE" : "FALSE")
                    << ", \"" << result->message << "\"]");
}

void SpotArmInterface::spot_trajectory_active() {
    ROS_INFO_STREAM("Trajectory active.");
}

void SpotArmInterface::spot_trajectory_feedback(const spot_msgs::TrajectoryFeedbackConstPtr& feedback) {
    ROS_INFO_STREAM("Trajectory feedback: \"" << feedback->feedback << "\"");
}

}
