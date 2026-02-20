#include "cloudguessr/backend/feedback.hpp"

#include <algorithm>
#include <cmath>

namespace cloudguessr {
namespace feedback {

namespace {

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double normalizeFitness(double fitness, double fail_min_fitness)
{
  const double denom = std::max(1e-6, 1.0 - fail_min_fitness);
  return clamp01((fitness - fail_min_fitness) / denom);
}

double normalizeRmse(double rmse, double fail_max_rmse)
{
  const double denom = std::max(1e-6, fail_max_rmse);
  return clamp01((fail_max_rmse - rmse) / denom);
}

}  // namespace

double computeQualityNormalized(
  double fitness,
  double rmse,
  double fail_min_fitness,
  double fail_max_rmse)
{
  const double fit_n = normalizeFitness(fitness, fail_min_fitness);
  const double rmse_n = normalizeRmse(rmse, fail_max_rmse);
  return clamp01(0.70 * fit_n + 0.30 * rmse_n);
}

double computeQualityPercent(
  double fitness,
  double rmse,
  double fail_min_fitness,
  double fail_max_rmse)
{
  return 100.0 * computeQualityNormalized(fitness, rmse, fail_min_fitness, fail_max_rmse);
}

int computeGameScore(
  double distance_m,
  double fitness,
  double rmse,
  const ScoreModelConfig & config)
{
  const double effective_distance = std::max(0.0, distance_m - config.perfect_radius_m);
  const double distance_decay = std::max(1e-6, config.distance_decay_m);
  const double distance_exp = std::max(0.1, config.distance_exp);
  const double distance_n = std::exp(-std::pow(effective_distance / distance_decay, distance_exp));

  const double quality_n = computeQualityNormalized(
    fitness, rmse, config.fail_min_fitness, config.fail_max_rmse);

  double distance_weight = std::max(0.0, config.distance_weight);
  double quality_weight = std::max(0.0, config.quality_weight);
  const double weight_sum = distance_weight + quality_weight;
  if (weight_sum <= 1e-6) {
    distance_weight = 1.0;
    quality_weight = 0.0;
  } else {
    distance_weight /= weight_sum;
    quality_weight /= weight_sum;
  }

  const double raw_n = clamp01(distance_weight * distance_n + quality_weight * quality_n);
  const double saturation_threshold = std::max(1e-6, config.saturation_threshold);
  const double score_n = clamp01(raw_n / saturation_threshold);
  const int score = static_cast<int>(std::round(5000.0 * score_n));
  return std::max(0, std::min(5000, score));
}

DistanceBand classifyDistanceBand(double distance_m)
{
  if (distance_m <= 5.0) {
    return DistanceBand::LE_5;
  }
  if (distance_m <= 15.0) {
    return DistanceBand::LE_15;
  }
  if (distance_m <= 40.0) {
    return DistanceBand::LE_40;
  }
  if (distance_m <= 100.0) {
    return DistanceBand::LE_100;
  }
  return DistanceBand::GT_100;
}

QualityBand classifyQualityBand(double quality_pct)
{
  if (quality_pct <= 70.0) {
    return QualityBand::LE_70;
  }
  if (quality_pct <= 80.0) {
    return QualityBand::LE_80;
  }
  if (quality_pct <= 90.0) {
    return QualityBand::LE_90;
  }
  return QualityBand::GT_90;
}

std::string distanceBandToString(DistanceBand band)
{
  switch (band) {
    case DistanceBand::LE_5: return "LE_5";
    case DistanceBand::LE_15: return "LE_15";
    case DistanceBand::LE_40: return "LE_40";
    case DistanceBand::LE_100: return "LE_100";
    case DistanceBand::GT_100: return "GT_100";
  }
  return "GT_100";
}

std::string qualityBandToString(QualityBand band)
{
  switch (band) {
    case QualityBand::LE_70: return "LE_70";
    case QualityBand::LE_80: return "LE_80";
    case QualityBand::LE_90: return "LE_90";
    case QualityBand::GT_90: return "GT_90";
  }
  return "LE_70";
}

ScoreTier classifyScoreTier(int score)
{
  if (score >= 4500) {
    return ScoreTier::S;
  }
  if (score >= 3000) {
    return ScoreTier::A;
  }
  if (score >= 1500) {
    return ScoreTier::B;
  }
  return ScoreTier::C;
}

std::string scoreTierToString(ScoreTier tier)
{
  switch (tier) {
    case ScoreTier::S: return "S";
    case ScoreTier::A: return "A";
    case ScoreTier::B: return "B";
    case ScoreTier::C: return "C";
  }
  return "C";
}

std::string distanceComment(DistanceBand band)
{
  switch (band) {
    case DistanceBand::LE_5: return "완벽합니다! 거의 정답 지점입니다.";
    case DistanceBand::LE_15: return "아주 좋습니다! 정답 근처를 정확히 찾았습니다.";
    case DistanceBand::LE_40: return "좋은 추측입니다. 꽤 가까웠습니다.";
    case DistanceBand::LE_100: return "방향은 맞았지만 조금 더 이동이 필요했습니다.";
    case DistanceBand::GT_100: return "정답과 거리가 꽤 있습니다.";
  }
  return "다음 라운드에 도전하세요!";
}

std::string qualityComment(QualityBand band)
{
  switch (band) {
    case QualityBand::LE_70: return "정합 신뢰도는 낮은 편입니다.";
    case QualityBand::LE_80: return "정합 신뢰도는 보통 수준입니다.";
    case QualityBand::LE_90: return "정합 신뢰도는 양호합니다.";
    case QualityBand::GT_90: return "정합 신뢰도가 매우 좋습니다.";
  }
  return "";
}

std::string buildComment(double distance_m, double quality_pct)
{
  const auto distance_band = classifyDistanceBand(distance_m);
  const auto quality_band = classifyQualityBand(quality_pct);
  return distanceComment(distance_band) + " " + qualityComment(quality_band);
}

}  // namespace feedback
}  // namespace cloudguessr
