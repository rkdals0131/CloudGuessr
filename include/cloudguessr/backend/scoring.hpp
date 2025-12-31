#pragma once

#include "cloudguessr/backend/types.hpp"

namespace cloudguessr {
namespace scoring {

/**
 * @brief Compute score from alignment quality
 * @param fitness Inlier ratio (0~1)
 * @param rmse Root mean square error
 * @param elapsed_ms Processing time in milliseconds
 * @param time_penalty_factor Penalty factor for time (score per ms over threshold)
 * @param time_threshold_ms Time threshold before penalty kicks in
 * @return Score 0~5000
 */
int computeScore(double fitness, 
                 double rmse, 
                 double elapsed_ms = 0.0,
                 double time_penalty_factor = 0.0,
                 double time_threshold_ms = 2000.0);

/**
 * @brief Check if result should be classified as FAIL
 * @param fitness Inlier ratio
 * @param rmse Root mean square error
 * @param converged Whether ICP converged
 * @param roi_points Number of points in ROI
 * @param query_points Number of points in query
 * @param min_fitness Minimum fitness threshold
 * @param max_rmse Maximum RMSE threshold
 * @param min_points Minimum points required
 * @return ScoreResult with status and reason
 */
ScoreResult classifyResult(double fitness,
                           double rmse,
                           bool converged,
                           size_t roi_points = 0,
                           size_t query_points = 0,
                           double min_fitness = 0.3,
                           double max_rmse = 2.0,
                           size_t min_points = 100);

/**
 * @brief Calculate distance error between clicked and ground truth positions
 * @param clicked_xyz Clicked position
 * @param gt_xyz Ground truth position
 * @return Euclidean distance in meters
 */
double calculateDistanceError(const std::vector<double>& clicked_xyz,
                              const std::vector<double>& gt_xyz);

/**
 * @brief Compute score based on distance error (primary) and fitness (secondary)
 *
 * Score formula: 5000 * (1 - (effective_distance / max_distance)^0.4) * fitness_bonus
 * - 3m 이내는 만점 구간 (effective_distance = 0)
 * - 이후 power 함수로 완만하게 감소 (로그 느낌)
 * - Fitness는 보조 지표 (0.7 ~ 1.0 배율)
 *
 * @param distance_m Distance error in meters
 * @param fitness ICP fitness (0~1), used as bonus multiplier
 * @param max_distance Map radius for normalization (default 350m)
 * @return Score 0~5000
 */
int computeScoreFromDistance(double distance_m,
                             double fitness = 1.0,
                             double max_distance = 350.0);

}  // namespace scoring
}  // namespace cloudguessr
