#pragma once

#include "cloudguessr/backend/types.hpp"

namespace cloudguessr {
namespace preprocess {

/**
 * @brief Voxel grid downsampling
 * @param cloud Input cloud
 * @param voxel_size Voxel size in meters
 * @return Downsampled cloud
 */
PointCloudPtr voxelDownsample(const PointCloudPtr& cloud, double voxel_size);

/**
 * @brief Statistical outlier removal
 * @param cloud Input cloud
 * @param mean_k Number of neighbors for mean distance
 * @param stddev_mul Standard deviation multiplier threshold
 * @return Filtered cloud
 */
PointCloudPtr removeOutliers(const PointCloudPtr& cloud, int mean_k = 50, double stddev_mul = 1.0);

/**
 * @brief Center cloud to origin (subtract centroid)
 * @param cloud Input cloud
 * @return Centered cloud and original centroid
 */
std::pair<PointCloudPtr, Eigen::Vector3f> centerCloud(const PointCloudPtr& cloud);

}  // namespace preprocess
}  // namespace cloudguessr
