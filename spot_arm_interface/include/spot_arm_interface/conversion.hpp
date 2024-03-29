#ifndef SPOT_ARM_INTERFACE_CONVERSION_HPP
#define SPOT_ARM_INTERFACE_CONVERSION_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

#include <Eigen/Geometry>

geometry_msgs::Vector3 point_to_vector(const geometry_msgs::Point& point);

template<typename Output, typename Input>
Output to_ros(const Input& input);

void to_ros(const Eigen::Isometry3d& isometry, geometry_msgs::Pose& pose);

void to_ros(const Eigen::Isometry3d& isometry, geometry_msgs::Transform& transform);

/** Implementation */

template<typename Output, typename Input>
Output to_ros(const Input& input) {
    Output output;
    to_ros(input, output);
    return output;
}

#endif
