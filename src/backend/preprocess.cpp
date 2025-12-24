#include "cloudguessr/backend/preprocess.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>

namespace cloudguessr {
namespace preprocess {

PointCloudPtr voxelDownsample(const PointCloudPtr& cloud, double voxel_size) {
  PointCloudPtr result(new PointCloud);
  if (!cloud || cloud->empty() || voxel_size <= 0) {
    return result;
  }

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(static_cast<float>(voxel_size),
                 static_cast<float>(voxel_size),
                 static_cast<float>(voxel_size));
  vg.filter(*result);
  return result;
}

PointCloudPtr removeOutliers(const PointCloudPtr& cloud, int mean_k, double stddev_mul) {
  PointCloudPtr result(new PointCloud);
  if (!cloud || cloud->empty()) {
    return result;
  }

  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(mean_k);
  sor.setStddevMulThresh(stddev_mul);
  sor.filter(*result);
  return result;
}

std::pair<PointCloudPtr, Eigen::Vector3f> centerCloud(const PointCloudPtr& cloud) {
  PointCloudPtr result(new PointCloud);
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  
  if (!cloud || cloud->empty()) {
    return {result, centroid};
  }

  // Compute centroid
  Eigen::Vector4f centroid4;
  pcl::compute3DCentroid(*cloud, centroid4);
  centroid = centroid4.head<3>();

  // Subtract centroid
  result->resize(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    result->points[i].x = cloud->points[i].x - centroid.x();
    result->points[i].y = cloud->points[i].y - centroid.y();
    result->points[i].z = cloud->points[i].z - centroid.z();
  }

  return {result, centroid};
}

}  // namespace preprocess
}  // namespace cloudguessr
