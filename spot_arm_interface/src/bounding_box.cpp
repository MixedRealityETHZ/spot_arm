#include "spot_arm_interface/bounding_box.hpp"

namespace spot_arm_interface {

BoundingBox3D::BoundingBox3D()
    : BoundingBox3D(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()) {}

BoundingBox3D::BoundingBox3D(const Eigen::Vector3d& center, const Eigen::Vector3d& size)
    : center(center),
      size(size) {}

Eigen::Vector3d BoundingBox3D::closest_valid(const Eigen::Vector3d& point) const {
    const Eigen::Vector3d half_size = size / 2.0;
    return Eigen::Vector3d{std::clamp(point[0], center[0] - half_size[0], center[0] + half_size[0]),
            std::clamp(point[1], center[1] - half_size[1], center[1] + half_size[1]),
            std::clamp(point[2], center[2] - half_size[2], center[2] + half_size[2])};
}

bool BoundingBox3D::contains(const Eigen::Vector3d& point) const {
    const Eigen::Vector3d half_size = size / 2.0;
    return point[0] >= (center[0] - half_size[0]) && point[0] <= (center[0] + half_size[0]) &&
           point[1] >= (center[1] - half_size[1]) && point[1] <= (center[1] + half_size[1]) &&
           point[2] >= (center[2] - half_size[2]) && point[2] <= (center[2] + half_size[2]);
}

}
