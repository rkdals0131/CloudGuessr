#pragma once

#include <string>

namespace cloudguessr {
namespace feedback {

struct ScoreModelConfig {
  double fail_min_fitness = 0.3;
  double fail_max_rmse = 2.0;

  double perfect_radius_m = 1.5;
  double distance_decay_m = 55.0;
  double distance_exp = 0.9;

  double distance_weight = 0.82;
  double quality_weight = 0.18;
  double saturation_threshold = 0.975;
};

enum class DistanceBand {
  LE_5,
  LE_15,
  LE_40,
  LE_100,
  GT_100
};

enum class QualityBand {
  LE_70,
  LE_80,
  LE_90,
  GT_90
};

enum class ScoreTier {
  S,
  A,
  B,
  C
};

double computeQualityNormalized(
  double fitness,
  double rmse,
  double fail_min_fitness = 0.3,
  double fail_max_rmse = 2.0);

double computeQualityPercent(
  double fitness,
  double rmse,
  double fail_min_fitness = 0.3,
  double fail_max_rmse = 2.0);

int computeGameScore(
  double distance_m,
  double fitness,
  double rmse,
  const ScoreModelConfig & config = ScoreModelConfig());

DistanceBand classifyDistanceBand(double distance_m);
QualityBand classifyQualityBand(double quality_pct);

std::string distanceBandToString(DistanceBand band);
std::string qualityBandToString(QualityBand band);

ScoreTier classifyScoreTier(int score);
std::string scoreTierToString(ScoreTier tier);

std::string distanceComment(DistanceBand band);
std::string qualityComment(QualityBand band);
std::string buildComment(double distance_m, double quality_pct);

}  // namespace feedback
}  // namespace cloudguessr
