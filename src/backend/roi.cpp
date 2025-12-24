#include "cloudguessr/backend/roi.hpp"
#include <pcl/filters/crop_box.h>
#include <cmath>

namespace cloudguessr {
namespace roi {

PointCloudPtr cropSphere(const PointCloudPtr& cloud,
                         const Eigen::Vector3f& center,
                         double radius) {
  PointCloudPtr result(new PointCloud);
  if (!cloud || cloud->empty() || radius <= 0) {
    return result;
  }

  double radius_sq = radius * radius;
  result->reserve(cloud->size());

  for (const auto& pt : cloud->points) {
    float dx = pt.x - center.x();
    float dy = pt.y - center.y();
    float dz = pt.z - center.z();
    double dist_sq = dx*dx + dy*dy + dz*dz;
    
    if (dist_sq <= radius_sq) {
      result->push_back(pt);
    }
  }

  result->width = result->size();
  result->height = 1;
  result->is_dense = true;
  return result;
}

PointCloudPtr cropBox(const PointCloudPtr& cloud,
                      const Eigen::Vector3f& min_pt,
                      const Eigen::Vector3f& max_pt) {
  PointCloudPtr result(new PointCloud);
  if (!cloud || cloud->empty()) {
    return result;
  }

  pcl::CropBox<PointT> cb;
  cb.setInputCloud(cloud);
  cb.setMin(Eigen::Vector4f(min_pt.x(), min_pt.y(), min_pt.z(), 1.0f));
  cb.setMax(Eigen::Vector4f(max_pt.x(), max_pt.y(), max_pt.z(), 1.0f));
  cb.filter(*result);
  return result;
}

}  // namespace roi
}  // namespace cloudguessr
