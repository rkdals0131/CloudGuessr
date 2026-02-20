/**
 * @file test_scoring.cpp
 * @brief Unit tests for cloudguessr::scoring module
 * UT-SCORE-01: Score range 0~5000 and monotonicity
 * UT-SCORE-02: Failure classification
 */

#include <gtest/gtest.h>
#include <tuple>
#include "cloudguessr/backend/scoring.hpp"

class ScoringTest : public ::testing::Test {
protected:
  // Default thresholds
  const double min_fitness_ = 0.3;
  const double max_rmse_ = 2.0;
  const size_t min_points_ = 100;
};

// UT-SCORE-01: Score is always in range 0~5000
TEST_F(ScoringTest, ComputeScore_InValidRange) {
  // Test various combinations
  std::vector<std::pair<double, double>> test_cases = {
    {1.0, 0.0},   // Perfect
    {0.0, 0.0},   // Zero fitness
    {0.5, 0.5},   // Medium
    {1.0, 10.0},  // High rmse
    {0.1, 0.1},   // Low fitness
  };
  
  for (const auto& [fitness, rmse] : test_cases) {
    int score = cloudguessr::scoring::computeScore(fitness, rmse);
    EXPECT_GE(score, 0) << "fitness=" << fitness << ", rmse=" << rmse;
    EXPECT_LE(score, 5000) << "fitness=" << fitness << ", rmse=" << rmse;
  }
}

// UT-SCORE-01: Higher fitness -> higher score
TEST_F(ScoringTest, ComputeScore_FitnessMonotonic) {
  double rmse = 0.5;
  
  int score_low = cloudguessr::scoring::computeScore(0.2, rmse);
  int score_mid = cloudguessr::scoring::computeScore(0.5, rmse);
  int score_high = cloudguessr::scoring::computeScore(0.8, rmse);
  
  EXPECT_LE(score_low, score_mid);
  EXPECT_LE(score_mid, score_high);
}

// UT-SCORE-01: Lower rmse -> higher score
TEST_F(ScoringTest, ComputeScore_RmseMonotonic) {
  double fitness = 0.8;
  
  int score_low_rmse = cloudguessr::scoring::computeScore(fitness, 0.1);
  int score_mid_rmse = cloudguessr::scoring::computeScore(fitness, 1.0);
  int score_high_rmse = cloudguessr::scoring::computeScore(fitness, 5.0);
  
  EXPECT_GE(score_low_rmse, score_mid_rmse);
  EXPECT_GE(score_mid_rmse, score_high_rmse);
}

// UT-SCORE-01: Perfect alignment gives maximum score
TEST_F(ScoringTest, ComputeScore_PerfectAlignment) {
  int score = cloudguessr::scoring::computeScore(1.0, 0.0);
  EXPECT_EQ(score, 5000);
}

// UT-SCORE-02: ICP not converged -> FAIL
TEST_F(ScoringTest, ClassifyResult_NotConverged_Fail) {
  auto result = cloudguessr::scoring::classifyResult(
    0.9, 0.1, false, 1000, 500, min_fitness_, max_rmse_, min_points_);
  
  EXPECT_EQ(result.status, "FAIL");
  EXPECT_EQ(result.score, 0);
  EXPECT_FALSE(result.reason.empty());
}

// UT-SCORE-02: Low fitness -> FAIL
TEST_F(ScoringTest, ClassifyResult_LowFitness_Fail) {
  auto result = cloudguessr::scoring::classifyResult(
    0.1, 0.5, true, 1000, 500, min_fitness_, max_rmse_, min_points_);
  
  EXPECT_EQ(result.status, "FAIL");
  EXPECT_EQ(result.score, 0);
  EXPECT_NE(result.reason.find("Fitness"), std::string::npos);
}

// UT-SCORE-02: High RMSE -> FAIL
TEST_F(ScoringTest, ClassifyResult_HighRmse_Fail) {
  auto result = cloudguessr::scoring::classifyResult(
    0.8, 5.0, true, 1000, 500, min_fitness_, max_rmse_, min_points_);
  
  EXPECT_EQ(result.status, "FAIL");
  EXPECT_EQ(result.score, 0);
  EXPECT_NE(result.reason.find("RMSE"), std::string::npos);
}

// UT-SCORE-02: Insufficient points -> FAIL
TEST_F(ScoringTest, ClassifyResult_InsufficientPoints_Fail) {
  auto result = cloudguessr::scoring::classifyResult(
    0.9, 0.1, true, 50, 500, min_fitness_, max_rmse_, min_points_);
  
  EXPECT_EQ(result.status, "FAIL");
  EXPECT_EQ(result.score, 0);
  EXPECT_NE(result.reason.find("ROI"), std::string::npos);
}

// Good result -> OK
TEST_F(ScoringTest, ClassifyResult_GoodResult_Ok) {
  auto result = cloudguessr::scoring::classifyResult(
    0.8, 0.5, true, 1000, 500, min_fitness_, max_rmse_, min_points_);
  
  EXPECT_EQ(result.status, "OK");
  EXPECT_TRUE(result.reason.empty());
  EXPECT_GT(result.score, 0);
}

// Test distance error calculation
TEST_F(ScoringTest, CalculateDistanceError_Correct) {
  std::vector<double> clicked = {10.0, 20.0, 0.0};
  std::vector<double> gt = {13.0, 24.0, 0.0};
  
  double error = cloudguessr::scoring::calculateDistanceError(clicked, gt);
  
  // Expected: sqrt(9 + 16) = 5.0
  EXPECT_NEAR(error, 5.0, 1e-6);
}

TEST_F(ScoringTest, CalculateDistanceError_3D) {
  std::vector<double> clicked = {0.0, 0.0, 0.0};
  std::vector<double> gt = {1.0, 2.0, 2.0};

  double error = cloudguessr::scoring::calculateDistanceError(clicked, gt);

  // Expected: sqrt(1 + 4 + 4) = 3.0
  EXPECT_NEAR(error, 3.0, 1e-6);
}

TEST_F(ScoringTest, CalculateDistanceError2D_IgnoresZ) {
  std::vector<double> clicked = {0.0, 0.0, 10.0};
  std::vector<double> gt = {3.0, 4.0, -50.0};

  double error_2d = cloudguessr::scoring::calculateDistanceError2D(clicked, gt);
  double error_3d = cloudguessr::scoring::calculateDistanceError(clicked, gt);

  EXPECT_NEAR(error_2d, 5.0, 1e-6);
  EXPECT_GT(error_3d, error_2d);
}

// UT-SCORE-03: Distance-based scoring tests (power function, max_distance=350)
TEST_F(ScoringTest, ComputeScoreFromDistance_PerfectRange) {
  // 3m 이내는 모두 만점
  EXPECT_EQ(cloudguessr::scoring::computeScoreFromDistance(0.0, 1.0), 5000);
  EXPECT_EQ(cloudguessr::scoring::computeScoreFromDistance(1.0, 1.0), 5000);
  EXPECT_EQ(cloudguessr::scoring::computeScoreFromDistance(2.0, 1.0), 5000);
  EXPECT_EQ(cloudguessr::scoring::computeScoreFromDistance(3.0, 1.0), 5000);
}

TEST_F(ScoringTest, ComputeScoreFromDistance_InValidRange) {
  // Test various distances
  std::vector<double> distances = {0.0, 5.0, 10.0, 50.0, 100.0, 200.0, 350.0};

  for (double dist : distances) {
    int score = cloudguessr::scoring::computeScoreFromDistance(dist, 1.0);
    EXPECT_GE(score, 0) << "distance=" << dist;
    EXPECT_LE(score, 5000) << "distance=" << dist;
  }
}

TEST_F(ScoringTest, ComputeScoreFromDistance_Monotonic) {
  // Closer distance -> higher score (beyond 3m perfect range)
  int score_3m = cloudguessr::scoring::computeScoreFromDistance(3.0, 1.0);
  int score_10m = cloudguessr::scoring::computeScoreFromDistance(10.0, 1.0);
  int score_50m = cloudguessr::scoring::computeScoreFromDistance(50.0, 1.0);
  int score_100m = cloudguessr::scoring::computeScoreFromDistance(100.0, 1.0);

  EXPECT_EQ(score_3m, 5000);  // Still perfect at 3m
  EXPECT_GT(score_3m, score_10m);
  EXPECT_GT(score_10m, score_50m);
  EXPECT_GT(score_50m, score_100m);
}

TEST_F(ScoringTest, ComputeScoreFromDistance_FitnessBonus) {
  // Same distance, higher fitness -> higher score
  double distance = 10.0;

  int score_low_fit = cloudguessr::scoring::computeScoreFromDistance(distance, 0.3);
  int score_mid_fit = cloudguessr::scoring::computeScoreFromDistance(distance, 0.6);
  int score_high_fit = cloudguessr::scoring::computeScoreFromDistance(distance, 1.0);

  EXPECT_LT(score_low_fit, score_high_fit);
  EXPECT_LT(score_mid_fit, score_high_fit);
  EXPECT_LE(score_low_fit, score_mid_fit);
}

TEST_F(ScoringTest, ComputeScoreFromDistance_ExpectedValues) {
  // Power function: score = 5000 * (1 - ratio^0.4), max_distance=350
  // effective_distance = distance - 3
  // 10m: eff=7, ratio=7/350=0.02, score=5000*(1-0.02^0.4)≈4150
  // 50m: eff=47, ratio=47/350≈0.134, score≈3400
  // 100m: eff=97, ratio≈0.277, score≈2900

  int score_3 = cloudguessr::scoring::computeScoreFromDistance(3.0, 1.0, 350.0);
  int score_10 = cloudguessr::scoring::computeScoreFromDistance(10.0, 1.0, 350.0);
  int score_100 = cloudguessr::scoring::computeScoreFromDistance(100.0, 1.0, 350.0);

  EXPECT_EQ(score_3, 5000);
  EXPECT_GT(score_10, 3900);   // 10m 오차 -> ~3954점
  EXPECT_GT(score_100, 1900);  // 100m 오차 -> ~2007점
}

TEST_F(ScoringTest, ComputeCompositeScore_InValidRange) {
  std::vector<std::tuple<double, double, double>> cases = {
    {0.0, 1.0, 0.1},
    {10.0, 0.8, 0.5},
    {100.0, 0.4, 1.5},
    {350.0, 0.0, 5.0},
  };

  for (const auto& [distance, fitness, rmse] : cases) {
    int score = cloudguessr::scoring::computeCompositeScore(distance, fitness, rmse);
    EXPECT_GE(score, 0);
    EXPECT_LE(score, 5000);
  }
}

TEST_F(ScoringTest, ComputeCompositeScore_DistanceMonotonic) {
  int score_near = cloudguessr::scoring::computeCompositeScore(5.0, 0.8, 0.5);
  int score_mid = cloudguessr::scoring::computeCompositeScore(50.0, 0.8, 0.5);
  int score_far = cloudguessr::scoring::computeCompositeScore(150.0, 0.8, 0.5);

  EXPECT_GT(score_near, score_mid);
  EXPECT_GT(score_mid, score_far);
}

TEST_F(ScoringTest, ComputeCompositeScore_QualityEffects) {
  int score_low_fit = cloudguessr::scoring::computeCompositeScore(20.0, 0.35, 0.5);
  int score_high_fit = cloudguessr::scoring::computeCompositeScore(20.0, 0.95, 0.5);
  EXPECT_GT(score_high_fit, score_low_fit);

  int score_low_rmse = cloudguessr::scoring::computeCompositeScore(20.0, 0.8, 0.3);
  int score_high_rmse = cloudguessr::scoring::computeCompositeScore(20.0, 0.8, 2.5);
  EXPECT_GT(score_low_rmse, score_high_rmse);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
