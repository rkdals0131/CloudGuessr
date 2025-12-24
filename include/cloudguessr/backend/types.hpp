#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <string>
#include <vector>

namespace cloudguessr {

// Point type alias
using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

/**
 * @brief ICP alignment result
 */
struct AlignmentResult {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  double fitness = 0.0;       // Inlier ratio (0~1)
  double rmse = 0.0;          // Root mean square error
  bool converged = false;
  double elapsed_ms = 0.0;
};

/**
 * @brief Yaw sweep result (best among candidates)
 */
struct YawSweepResult {
  AlignmentResult best_alignment;
  double best_yaw_deg = 0.0;
  int best_yaw_index = -1;
};

/**
 * @brief Scoring result
 */
struct ScoreResult {
  int score = 0;              // 0 ~ 5000
  std::string status = "OK";  // "OK" or "FAIL"
  std::string reason = "";    // Failure reason (if FAIL)
};

/**
 * @brief Round metadata from round.yaml
 */
struct RoundMetadata {
  int round_id = 0;
  double gt_x = 0.0;
  double gt_y = 0.0;
  double gt_z = 0.0;
  double roi_radius = 20.0;
  std::string difficulty = "medium";
  std::string notes = "";
};

/**
 * @brief Complete result for a round
 */
struct RoundResult {
  int round_id = 0;
  std::vector<double> clicked_xyz = {0.0, 0.0, 0.0};
  std::vector<double> gt_xyz = {0.0, 0.0, 0.0};
  double dist_error_m = 0.0;
  double best_yaw_deg = 0.0;
  double fitness = 0.0;
  double rmse = 0.0;
  int score = 0;
  double elapsed_ms = 0.0;
  std::string status = "OK";
  std::string reason = "";
};

}  // namespace cloudguessr
