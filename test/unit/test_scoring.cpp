/**
 * @file test_scoring.cpp
 * @brief Unit tests for cloudguessr::scoring module
 * UT-SCORE-01: Score range 0~5000 and monotonicity
 * UT-SCORE-02: Failure classification
 */

#include <gtest/gtest.h>
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
  EXPECT_FALSE(result.reason.empty());
}

// UT-SCORE-02: Low fitness -> FAIL
TEST_F(ScoringTest, ClassifyResult_LowFitness_Fail) {
  auto result = cloudguessr::scoring::classifyResult(
    0.1, 0.5, true, 1000, 500, min_fitness_, max_rmse_, min_points_);
  
  EXPECT_EQ(result.status, "FAIL");
  EXPECT_NE(result.reason.find("Fitness"), std::string::npos);
}

// UT-SCORE-02: High RMSE -> FAIL
TEST_F(ScoringTest, ClassifyResult_HighRmse_Fail) {
  auto result = cloudguessr::scoring::classifyResult(
    0.8, 5.0, true, 1000, 500, min_fitness_, max_rmse_, min_points_);
  
  EXPECT_EQ(result.status, "FAIL");
  EXPECT_NE(result.reason.find("RMSE"), std::string::npos);
}

// UT-SCORE-02: Insufficient points -> FAIL
TEST_F(ScoringTest, ClassifyResult_InsufficientPoints_Fail) {
  auto result = cloudguessr::scoring::classifyResult(
    0.9, 0.1, true, 50, 500, min_fitness_, max_rmse_, min_points_);
  
  EXPECT_EQ(result.status, "FAIL");
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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
