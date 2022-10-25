#include <ros/ros.h>

#include "spot_arm_interface/spot_arm_interface.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "spot_arm_interface");
    spot_arm_interface::SpotArmInterface spot_arm_interface;
    ros::spin();
    return 0;
}
