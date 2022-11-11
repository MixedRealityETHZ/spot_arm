#include "spot_arm_interface/conversion.hpp"

geometry_msgs::Vector3 point_to_vector(const geometry_msgs::Point& point) {
    geometry_msgs::Vector3 vector;
    vector.x = point.x;
    vector.y = point.y;
    vector.z = point.z;
    return vector;
}

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

void to_ros(const Eigen::Isometry3d& isometry, geometry_msgs::Transform& pose) {
    const Eigen::Vector3d t = isometry.translation();
    const Eigen::Quaterniond q = Eigen::Quaterniond{isometry.rotation()};
    pose.translation.x = t[0];
    pose.translation.y = t[1];
    pose.translation.z = t[2];
    pose.rotation.w = q.w();
    pose.rotation.x = q.x();
    pose.rotation.y = q.y();
    pose.rotation.z = q.z();
}
