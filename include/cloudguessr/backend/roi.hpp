#pragma once

#include "cloudguessr/backend/types.hpp"
#include <Eigen/Core>

namespace cloudguessr {
namespace roi {

/**
 * @brief Crop points within a sphere from the cloud
 * @param cloud Input cloud
 * @param center Center of the sphere
 * @param radius Radius in meters
 * @return Cropped cloud containing only points within radius
 */
PointCloudPtr cropSphere(const PointCloudPtr& cloud, 
                         const Eigen::Vector3f& center, 
                         double radius);

/**
 * @brief Crop points within a box from the cloud
 * @param cloud Input cloud
 * @param min_pt Minimum corner
 * @param max_pt Maximum corner
 * @return Cropped cloud
 */
PointCloudPtr cropBox(const PointCloudPtr& cloud,
                      const Eigen::Vector3f& min_pt,
                      const Eigen::Vector3f& max_pt);

}  // namespace roi
}  // namespace cloudguessr
