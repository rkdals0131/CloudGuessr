/**
 * @file test_feedback.cpp
 * @brief Unit tests for cloudguessr::feedback module
 */

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "cloudguessr/backend/feedback.hpp"

class FeedbackTest : public ::testing::Test {};

TEST_F(FeedbackTest, QualityNormalized_InValidRange)
{
  std::vector<std::pair<double, double>> test_cases = {
    {1.0, 0.0},
    {0.9, 0.2},
    {0.3, 2.0},
    {0.1, 3.0},
  };

  for (const auto & [fitness, rmse] : test_cases) {
    const double q = cloudguessr::feedback::computeQualityNormalized(fitness, rmse, 0.3, 2.0);
    EXPECT_GE(q, 0.0);
    EXPECT_LE(q, 1.0);
  }
}

TEST_F(FeedbackTest, DistanceBand_Boundaries)
{
  using cloudguessr::feedback::DistanceBand;
  EXPECT_EQ(cloudguessr::feedback::classifyDistanceBand(5.0), DistanceBand::LE_5);
  EXPECT_EQ(cloudguessr::feedback::classifyDistanceBand(15.0), DistanceBand::LE_15);
  EXPECT_EQ(cloudguessr::feedback::classifyDistanceBand(40.0), DistanceBand::LE_40);
  EXPECT_EQ(cloudguessr::feedback::classifyDistanceBand(100.0), DistanceBand::LE_100);
  EXPECT_EQ(cloudguessr::feedback::classifyDistanceBand(100.1), DistanceBand::GT_100);
}

TEST_F(FeedbackTest, QualityBand_Boundaries)
{
  using cloudguessr::feedback::QualityBand;
  EXPECT_EQ(cloudguessr::feedback::classifyQualityBand(70.0), QualityBand::LE_70);
  EXPECT_EQ(cloudguessr::feedback::classifyQualityBand(80.0), QualityBand::LE_80);
  EXPECT_EQ(cloudguessr::feedback::classifyQualityBand(90.0), QualityBand::LE_90);
  EXPECT_EQ(cloudguessr::feedback::classifyQualityBand(90.1), QualityBand::GT_90);
}

TEST_F(FeedbackTest, GameScore_SaturatesNearPerfect)
{
  cloudguessr::feedback::ScoreModelConfig cfg;
  cfg.fail_min_fitness = 0.3;
  cfg.fail_max_rmse = 2.0;
  cfg.perfect_radius_m = 1.5;
  cfg.distance_decay_m = 55.0;
  cfg.distance_exp = 0.9;
  cfg.distance_weight = 0.82;
  cfg.quality_weight = 0.18;
  cfg.saturation_threshold = 0.975;

  const int score = cloudguessr::feedback::computeGameScore(1.5, 1.0, 0.67, cfg);
  EXPECT_EQ(score, 5000);
}

TEST_F(FeedbackTest, GameScore_DistanceDominant)
{
  cloudguessr::feedback::ScoreModelConfig cfg;
  cfg.distance_weight = 0.82;
  cfg.quality_weight = 0.18;

  const int near_low_quality = cloudguessr::feedback::computeGameScore(10.0, 0.35, 1.9, cfg);
  const int far_high_quality = cloudguessr::feedback::computeGameScore(80.0, 0.95, 0.2, cfg);
  EXPECT_GT(near_low_quality, far_high_quality);
}

TEST_F(FeedbackTest, BuildComment_ContainsDistanceAndQualityMessage)
{
  const std::string comment = cloudguessr::feedback::buildComment(12.0, 85.0);
  EXPECT_NE(comment.find("정답"), std::string::npos);
  EXPECT_NE(comment.find("정합"), std::string::npos);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
