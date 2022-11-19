#include <ros/ros.h>

#include "spot_arm_interface/spot_move_client.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "spot_move_client");
    spot_arm_interface::SpotMoveClient spot_move_client;
    ros::spin();
    return 0;
}
