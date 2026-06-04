#include "DistributionDefaults.hpp"

#include <imgui.h>

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

evo_engine::PlottedDistributionSettings DistributionDefaults::MakePlottedGuiSettings(const std::string& tip) {
  evo_engine::PlottedDistributionSettings settings;
  settings.tip = tip;
  settings.mean_settings.m_tip = "Mean response curve. x is normalized axis [0, 1].";
  settings.dev_settings.m_tip = "Standard deviation (sigma) curve. Zero keeps deterministic behavior.";
  return settings;
}

bool DistributionDefaults::InspectPlottedDistributionCategory(
    const char* category_label, const std::initializer_list<PlottedDistributionUiEntry> entries,
    const int tree_node_flags) {
  if (!category_label || category_label[0] == '\0') {
    return false;
  }

  bool changed = false;
  if (ImGui::TreeNodeEx(category_label, static_cast<ImGuiTreeNodeFlags>(tree_node_flags))) {
    for (const auto& entry : entries) {
      if (!entry.distribution || !entry.label || entry.label[0] == '\0') {
        continue;
      }
      const std::string tip = entry.tip ? entry.tip : "";
      changed |= entry.distribution->OnInspect(entry.label, MakePlottedGuiSettings(tip));
    }
    ImGui::TreePop();
  }

  return changed;
}