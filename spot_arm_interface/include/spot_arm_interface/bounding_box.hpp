#ifndef SPOT_ARM_INTERFACE_BOUNDING_BOX_HPP
#define SPOT_ARM_INTERFACE_BOUNDING_BOX_HPP

#include <Eigen/Core>

namespace spot_arm_interface {

class BoundingBox3D {
public:
    explicit BoundingBox3D();
    explicit BoundingBox3D(const Eigen::Vector3d& center, const Eigen::Vector3d& size);

    /**
     * @brief Computes the closest valid point to the bounding box (unchanged if already within bounding box).
     * 
     * @param point 
     * @return Eigen::Vector3d 
     */
    Eigen::Vector3d closest_valid(const Eigen::Vector3d& point) const;

    /**
     * @brief Check if point is in bounding box.
     * 
     * @param point 
     * @return true if within bounding box (inclusive of boundary)
     * @return false otherwise
     */
    bool contains(const Eigen::Vector3d& point) const;

protected:
    // Center of the bounding box (center_x, center_y, center_z)
    Eigen::Vector3d center;
    // Size of the bounding box from face to face (size_x, size_y, size_z)
    Eigen::Vector3d size;
};

}

#endif
