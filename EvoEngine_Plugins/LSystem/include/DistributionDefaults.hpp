#pragma once

#include <Plot2D.hpp>
#include <initializer_list>
#include <string>

namespace l_system_plugin {

class DistributionDefaults {
 public:
  struct PlottedDistributionUiEntry {
    const char* label = "";
    evo_engine::PlottedDistribution<float>* distribution = nullptr;
    const char* tip = "";
  };

  struct SingleDistributionUiPreset {
    float speed = 0.01f;
    const char* format = "%.3f";
    std::string tip;
  };

  static void SetCurveLinear01(evo_engine::Curve2D& curve, int sample_count = 9);
  static void SetCurveFlat(evo_engine::Curve2D& curve, float y_value = 0.0f, int sample_count = 2);

  static void ApplyMeanPlotDefaults(evo_engine::Plot2D<float>& plot);
  static void ApplyStdPlotDefaults(evo_engine::Plot2D<float>& plot);
  static void ApplyMeanStdPlotDefaults(evo_engine::PlottedDistribution<float>& distribution);
  static void ApplyLinearGrowthCurveDefaults(evo_engine::PlottedDistribution<float>& distribution,
                                             float mean_start = 0.0f,
                                             float mean_end = 1.0f,
                                             int sample_count = 9);

  static void ApplySingleDefaults(evo_engine::SingleDistribution<float>& distribution,
                                  float mean = 0.0f,
                                  float deviation = 0.0f);
  static evo_engine::SingleDistribution<float> MakeSingleDefaults(float mean = 0.0f,
                                                                   float deviation = 0.0f);

  static evo_engine::PlottedDistributionSettings MakePlottedGuiSettings(
      const std::string& tip = "");
  static bool InspectPlottedDistributionCategory(
      const char* category_label,
      std::initializer_list<PlottedDistributionUiEntry> entries,
      int tree_node_flags = 0);
};

template <typename... TDistributions>
void ApplyMeanStdPlotDefaultsToAll(TDistributions&... distributions) {
  (DistributionDefaults::ApplyMeanStdPlotDefaults(distributions), ...);
}

template <typename... TDistributions>
void ApplyLinearGrowthCurveDefaultsToAll(TDistributions&... distributions) {
  (DistributionDefaults::ApplyLinearGrowthCurveDefaults(distributions), ...);
}

}  // namespace l_system_plugin
