#include "cloudguessr/backend/io.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <algorithm>
#include <iostream>

namespace cloudguessr {
namespace io {

PointCloudPtr loadPointCloud(const std::string& filepath) {
  PointCloudPtr cloud(new PointCloud);
  
  // Determine file type by extension
  std::string ext = filepath.substr(filepath.find_last_of(".") + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
  
  int result = -1;
  if (ext == "pcd") {
    result = pcl::io::loadPCDFile<PointT>(filepath, *cloud);
  } else if (ext == "ply") {
    result = pcl::io::loadPLYFile<PointT>(filepath, *cloud);
  } else {
    std::cerr << "[io] Unsupported file extension: " << ext << std::endl;
    return cloud;  // Return empty cloud
  }
  
  if (result < 0) {
    std::cerr << "[io] Failed to load: " << filepath << std::endl;
    cloud->clear();
  }
  
  return cloud;
}

bool savePointCloud(const PointCloudPtr& cloud, const std::string& filepath) {
  if (!cloud || cloud->empty()) {
    std::cerr << "[io] Cannot save empty cloud" << std::endl;
    return false;
  }
  
  int result = pcl::io::savePCDFileBinary(filepath, *cloud);
  return (result >= 0);
}

size_t countNaNPoints(const PointCloudConstPtr& cloud) {
  if (!cloud) return 0;
  
  size_t count = 0;
  for (const auto& pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      count++;
    }
  }
  return count;
}

PointCloudPtr removeNaNPoints(const PointCloudPtr& cloud) {
  PointCloudPtr result(new PointCloud);
  if (!cloud) return result;
  
  result->reserve(cloud->size());
  for (const auto& pt : cloud->points) {
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
      result->push_back(pt);
    }
  }
  result->width = result->size();
  result->height = 1;
  result->is_dense = true;
  return result;
}

}  // namespace io
}  // namespace cloudguessr
