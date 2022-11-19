#ifndef SPOT_ARM_INTERFACE_SPOT_MOVE_CLIENT_HPP
#define SPOT_ARM_INTERFACE_SPOT_MOVE_CLIENT_HPP

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <spot_msgs/TrajectoryAction.h>

namespace spot_arm_interface {

class SpotMoveClient {
public:
    SpotMoveClient();

private:
    void pose_callback(const geometry_msgs::PoseConstPtr& pose);

    void spot_trajectory_done(const actionlib::SimpleClientGoalState& state,
            const spot_msgs::TrajectoryResultConstPtr& result);

    void spot_trajectory_active();

    void spot_trajectory_feedback(const spot_msgs::TrajectoryFeedbackConstPtr& feedback);

    //// ROS
    ros::NodeHandle nh;
    // Listen to hand pose requests
    ros::Subscriber pose_subscriber;
    // Spot body pose action client
    actionlib::SimpleActionClient<spot_msgs::TrajectoryAction> spot_body_pose_client;
};

}

#endif
