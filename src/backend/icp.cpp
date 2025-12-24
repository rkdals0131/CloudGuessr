#include "cloudguessr/backend/icp.hpp"
#include <pcl/registration/icp.h>
#include <chrono>
#include <cmath>

namespace cloudguessr {
namespace icp {

Eigen::Matrix4f createYawRotation(double yaw_deg) {
  double yaw_rad = yaw_deg * M_PI / 180.0;
  Eigen::Matrix4f rot = Eigen::Matrix4f::Identity();
  rot(0, 0) = std::cos(yaw_rad);
  rot(0, 1) = -std::sin(yaw_rad);
  rot(1, 0) = std::sin(yaw_rad);
  rot(1, 1) = std::cos(yaw_rad);
  return rot;
}

Eigen::Matrix4f createInitialTransform(const Eigen::Vector3f& translation, double yaw_deg) {
  Eigen::Matrix4f T = createYawRotation(yaw_deg);
  T(0, 3) = translation.x();
  T(1, 3) = translation.y();
  T(2, 3) = translation.z();
  return T;
}

AlignmentResult icpAlign(const PointCloudPtr& target,
                         const PointCloudPtr& source,
                         const Eigen::Matrix4f& init_transform,
                         int max_iterations,
                         double max_correspondence_distance) {
  AlignmentResult result;
  
  if (!target || !source || target->empty() || source->empty()) {
    result.converged = false;
    return result;
  }

  auto start = std::chrono::high_resolution_clock::now();

  // Setup ICP
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaximumIterations(max_iterations);
  icp.setMaxCorrespondenceDistance(max_correspondence_distance);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1e-6);

  // Align
  PointCloud aligned;
  icp.align(aligned, init_transform);

  auto end = std::chrono::high_resolution_clock::now();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

  result.transform = icp.getFinalTransformation();
  result.converged = icp.hasConverged();
  result.fitness = icp.getFitnessScore(max_correspondence_distance);
  
  // Calculate RMSE differently - fitness score in PCL is mean squared distance
  // We need to convert it properly
  if (result.fitness > 0) {
    result.rmse = std::sqrt(result.fitness);
  } else {
    result.rmse = 0.0;
  }
  
  // Recalculate fitness as inlier ratio
  if (result.converged && source->size() > 0) {
    // Transform source and count inliers
    PointCloudPtr transformed(new PointCloud);
    pcl::transformPointCloud(*source, *transformed, result.transform);
    
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(target);
    
    int inliers = 0;
    double total_error = 0.0;
    std::vector<int> indices(1);
    std::vector<float> dists(1);
    
    for (const auto& pt : transformed->points) {
      if (kdtree.nearestKSearch(pt, 1, indices, dists) > 0) {
        double d = std::sqrt(dists[0]);
        if (d < max_correspondence_distance) {
          inliers++;
          total_error += d * d;
        }
      }
    }
    
    result.fitness = static_cast<double>(inliers) / source->size();
    if (inliers > 0) {
      result.rmse = std::sqrt(total_error / inliers);
    }
  }

  return result;
}

YawSweepResult yawSweepAlign(const PointCloudPtr& target,
                             const PointCloudPtr& source,
                             const Eigen::Vector3f& clicked_xyz,
                             const std::vector<double>& yaw_candidates_deg,
                             int max_iterations,
                             double max_correspondence_distance) {
  YawSweepResult result;
  result.best_yaw_index = -1;
  result.best_alignment.fitness = -1.0;

  if (yaw_candidates_deg.empty()) {
    return result;
  }

  for (size_t i = 0; i < yaw_candidates_deg.size(); ++i) {
    double yaw = yaw_candidates_deg[i];
    Eigen::Matrix4f init_T = createInitialTransform(clicked_xyz, yaw);
    
    AlignmentResult ar = icpAlign(target, source, init_T, 
                                   max_iterations, max_correspondence_distance);
    
    // Select best based on fitness (higher is better)
    if (ar.converged && ar.fitness > result.best_alignment.fitness) {
      result.best_alignment = ar;
      result.best_yaw_deg = yaw;
      result.best_yaw_index = static_cast<int>(i);
    }
  }

  return result;
}

}  // namespace icp
}  // namespace cloudguessr
