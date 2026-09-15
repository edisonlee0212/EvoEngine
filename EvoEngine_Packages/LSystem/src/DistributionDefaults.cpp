#include "DistributionDefaults.hpp"

#include <algorithm>

using namespace l_system_package;
using namespace evo_engine;

namespace {

void SetCurveLinearRange(Curve2D& curve, const float y_start, const float y_end, const int sample_count) {
  curve.SetTangent(false);
  auto& values = curve.UnsafeGetValues();
  values.clear();

  const int safe_samples = std::max(2, sample_count);
  const float clamped_start = std::clamp(y_start, 0.0f, 1.0f);
  const float clamped_end = std::clamp(y_end, 0.0f, 1.0f);

  for (int i = 0; i < safe_samples; ++i) {
    const float x = static_cast<float>(i) / static_cast<float>(safe_samples - 1);
    const float y = clamped_start + (clamped_end - clamped_start) * x;
    values.emplace_back(x, y);
  }
}

}  // namespace

void DistributionDefaults::SetCurveLinear01(Curve2D& curve, const int sample_count) {
  SetCurveLinearRange(curve, 0.0f, 1.0f, sample_count);
}

void DistributionDefaults::SetCurveFlat(Curve2D& curve, const float y_value, const int sample_count) {
  SetCurveLinearRange(curve, y_value, y_value, sample_count);
}

void DistributionDefaults::ApplyMeanPlotDefaults(Plot2D<float>& plot) {
  plot.min_value = 0.0f;
  plot.max_value = 1.0f;
  SetCurveLinear01(plot.curve);
}

void DistributionDefaults::ApplyStdPlotDefaults(Plot2D<float>& plot) {
  plot.min_value = 0.0f;
  plot.max_value = 0.0f;
  SetCurveFlat(plot.curve, 0.0f);
}

void DistributionDefaults::ApplyMeanStdPlotDefaults(PlottedDistribution<float>& distribution) {
  ApplyMeanPlotDefaults(distribution.mean);
  ApplyStdPlotDefaults(distribution.deviation);
}

void DistributionDefaults::ApplyLinearGrowthCurveDefaults(PlottedDistribution<float>& distribution,
                                                          const float mean_start, const float mean_end,
                                                          const int sample_count) {
  distribution.mean.min_value = 0.0f;
  distribution.mean.max_value = 1.0f;
  SetCurveLinearRange(distribution.mean.curve, mean_start, mean_end, sample_count);

  distribution.deviation.min_value = 0.0f;
  distribution.deviation.max_value = 0.0f;
  SetCurveFlat(distribution.deviation.curve, 0.0f);
}

void DistributionDefaults::ApplySingleDefaults(SingleDistribution<float>& distribution, const float mean,
                                               const float deviation) {
  distribution.mean = mean;
  distribution.deviation = std::max(0.0f, deviation);
}

SingleDistribution<float> DistributionDefaults::MakeSingleDefaults(const float mean, const float deviation) {
  SingleDistribution<float> distribution{};
  ApplySingleDefaults(distribution, mean, deviation);
  return distribution;
}
