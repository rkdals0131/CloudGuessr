#include "cloudguessr/backend/scoring.hpp"
#include <cmath>
#include <algorithm>

namespace cloudguessr {
namespace scoring {

int computeScore(double fitness, 
                 double rmse, 
                 double elapsed_ms,
                 double time_penalty_factor,
                 double time_threshold_ms) {
  // Base score from fitness and rmse
  // Higher fitness (0~1) -> higher score
  // Lower rmse -> higher score
  
  // Quality factor Q = fitness * rmse_factor
  // rmse_factor: 1.0 when rmse=0, decreasing as rmse increases
  double rmse_factor = 1.0 / (1.0 + rmse);  // Smooth decay
  double quality = fitness * rmse_factor;
  
  // Base score (0 ~ 5000)
  double base_score = 5000.0 * quality;
  
  // Time penalty (optional)
  double penalty = 0.0;
  if (time_penalty_factor > 0 && elapsed_ms > time_threshold_ms) {
    penalty = time_penalty_factor * (elapsed_ms - time_threshold_ms);
  }
  
  double final_score = base_score - penalty;
  
  // Clamp to valid range
  final_score = std::max(0.0, std::min(5000.0, final_score));
  
  return static_cast<int>(std::round(final_score));
}

ScoreResult classifyResult(double fitness,
                           double rmse,
                           bool converged,
                           size_t roi_points,
                           size_t query_points,
                           double min_fitness,
                           double max_rmse,
                           size_t min_points) {
  ScoreResult result;
  result.status = "OK";
  result.reason = "";

  // Check convergence
  if (!converged) {
    result.status = "FAIL";
    result.reason = "ICP did not converge";
    result.score = 0;
    return result;
  }

  // Check minimum points
  if (roi_points > 0 && roi_points < min_points) {
    result.status = "FAIL";
    result.reason = "Insufficient ROI points";
    result.score = 0;
    return result;
  }
  
  if (query_points > 0 && query_points < min_points) {
    result.status = "FAIL";
    result.reason = "Insufficient query points";
    result.score = 0;
    return result;
  }

  // Check fitness threshold
  if (fitness < min_fitness) {
    result.status = "FAIL";
    result.reason = "Fitness below threshold (" + std::to_string(fitness) + 
                    " < " + std::to_string(min_fitness) + ")";
    result.score = computeScore(fitness, rmse);
    return result;
  }

  // Check RMSE threshold
  if (rmse > max_rmse) {
    result.status = "FAIL";
    result.reason = "RMSE above threshold (" + std::to_string(rmse) + 
                    " > " + std::to_string(max_rmse) + ")";
    result.score = computeScore(fitness, rmse);
    return result;
  }

  // Success - compute score
  result.score = computeScore(fitness, rmse);
  return result;
}

double calculateDistanceError(const std::vector<double>& clicked_xyz,
                              const std::vector<double>& gt_xyz) {
  if (clicked_xyz.size() < 3 || gt_xyz.size() < 3) {
    return 0.0;
  }
  
  double dx = clicked_xyz[0] - gt_xyz[0];
  double dy = clicked_xyz[1] - gt_xyz[1];
  double dz = clicked_xyz[2] - gt_xyz[2];
  
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

}  // namespace scoring
}  // namespace cloudguessr
