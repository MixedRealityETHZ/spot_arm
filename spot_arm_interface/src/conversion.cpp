#include "spot_arm_interface/conversion.hpp"

void to_ros(const Eigen::Isometry3d& isometry, geometry_msgs::Pose& pose) {
    const Eigen::Vector3d t = isometry.translation();
    const Eigen::Quaterniond q = Eigen::Quaterniond{isometry.rotation()};
    pose.position.x = t[0];
    pose.position.y = t[1];
    pose.position.z = t[2];
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
}
