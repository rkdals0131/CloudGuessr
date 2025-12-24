#pragma once

#include "cloudguessr/backend/types.hpp"
#include <Eigen/Core>
#include <vector>

namespace cloudguessr {
namespace icp {

/**
 * @brief Perform ICP alignment
 * @param target Target cloud (ROI from map)
 * @param source Source cloud (query)
 * @param init_transform Initial transformation guess
 * @param max_iterations Maximum ICP iterations
 * @param max_correspondence_distance Maximum correspondence distance
 * @return Alignment result
 */
AlignmentResult icpAlign(const PointCloudPtr& target,
                         const PointCloudPtr& source,
                         const Eigen::Matrix4f& init_transform = Eigen::Matrix4f::Identity(),
                         int max_iterations = 50,
                         double max_correspondence_distance = 1.0);

/**
 * @brief Create rotation matrix for yaw (rotation around Z axis)
 * @param yaw_deg Yaw angle in degrees
 * @return 4x4 transformation matrix
 */
Eigen::Matrix4f createYawRotation(double yaw_deg);

/**
 * @brief Create initial transformation from translation and yaw
 * @param translation Translation vector
 * @param yaw_deg Yaw angle in degrees
 * @return 4x4 transformation matrix
 */
Eigen::Matrix4f createInitialTransform(const Eigen::Vector3f& translation, double yaw_deg);

/**
 * @brief Sweep through yaw candidates and find best alignment
 * @param target Target cloud (ROI from map)
 * @param source Source cloud (query, centered)
 * @param clicked_xyz Position clicked by user
 * @param yaw_candidates_deg List of yaw angles to try (degrees)
 * @param max_iterations ICP max iterations
 * @param max_correspondence_distance ICP max correspondence distance
 * @return Best alignment result with yaw info
 */
YawSweepResult yawSweepAlign(const PointCloudPtr& target,
                             const PointCloudPtr& source,
                             const Eigen::Vector3f& clicked_xyz,
                             const std::vector<double>& yaw_candidates_deg,
                             int max_iterations = 50,
                             double max_correspondence_distance = 1.0);

}  // namespace icp
}  // namespace cloudguessr
