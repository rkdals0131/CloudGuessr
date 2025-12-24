#pragma once

#include "cloudguessr/backend/types.hpp"
#include <string>

namespace cloudguessr {
namespace io {

/**
 * @brief Load a point cloud from PCD or PLY file
 * @param filepath Path to the file (.pcd or .ply)
 * @return Loaded point cloud (empty if load failed)
 */
PointCloudPtr loadPointCloud(const std::string& filepath);

/**
 * @brief Save a point cloud to PCD file
 * @param cloud Point cloud to save
 * @param filepath Output path (.pcd)
 * @return true if successful
 */
bool savePointCloud(const PointCloudPtr& cloud, const std::string& filepath);

/**
 * @brief Check if point cloud contains NaN points
 * @param cloud Point cloud to check
 * @return Number of NaN points
 */
size_t countNaNPoints(const PointCloudConstPtr& cloud);

/**
 * @brief Remove NaN points from cloud
 * @param cloud Input cloud
 * @return Cloud with NaN points removed
 */
PointCloudPtr removeNaNPoints(const PointCloudPtr& cloud);

}  // namespace io
}  // namespace cloudguessr
