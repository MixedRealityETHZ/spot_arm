#include "spot_arm_interface/spot_move_client.hpp"

namespace spot_arm_interface {

SpotMoveClient::SpotMoveClient()
    : nh("~"),
      spot_body_pose_client("/spot/trajectory", true) {
    pose_subscriber = nh.subscribe<geometry_msgs::Pose>("pose", 1, &SpotMoveClient::pose_callback, this);
    ROS_INFO_STREAM("Waiting for Spot trajectory server.");
    spot_body_pose_client.waitForServer();
    ROS_INFO_STREAM("Acquired Spot trajectory server");
}

void SpotMoveClient::pose_callback(const geometry_msgs::PoseConstPtr& pose) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "body";
    pose_stamped.pose = *pose;

    spot_msgs::TrajectoryGoal spot_body_pose_goal;
    spot_body_pose_goal.target_pose = pose_stamped;
    spot_body_pose_goal.duration.data = ros::Duration(5);
    spot_body_pose_goal.precise_positioning = true;
    spot_body_pose_client.sendGoal(spot_body_pose_goal,
            std::bind(&SpotMoveClient::spot_trajectory_done, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SpotMoveClient::spot_trajectory_active, this),
            std::bind(&SpotMoveClient::spot_trajectory_feedback, this, std::placeholders::_1));
}

void SpotMoveClient::spot_trajectory_done(const actionlib::SimpleClientGoalState& state,
        const spot_msgs::TrajectoryResultConstPtr& result) {
    ROS_INFO_STREAM("Trajectory action finished ("
                    << state.toString() << ") with result: [success = " << (result->success ? "TRUE" : "FALSE")
                    << ", \"" << result->message << "\"]");
}

void SpotMoveClient::spot_trajectory_active() {
    ROS_INFO_STREAM("Trajectory active.");
}

void SpotMoveClient::spot_trajectory_feedback(const spot_msgs::TrajectoryFeedbackConstPtr& feedback) {
    ROS_INFO_STREAM("Trajectory feedback: \"" << feedback->feedback << "\"");
}

}
