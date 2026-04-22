
#pragma once

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <type_traits>

namespace evo_engine {

/**
 * @brief Flags for customizing the behavior of a CurveEditor.
 */
enum class CurveEditorFlags {
  ShowGrid = 1 << 1,         /**< Display a grid within the editor. */
  Reset = 1 << 2,            /**< Reset the editor settings. */
  AllowResize = 1 << 3,      /**< Allow resizing of the editor. */
  AllowRemoveSides = 1 << 4, /**< Allow removal of sides. */
  DisableStartEndY = 1 << 5, /**< Prevent modification of start/end Y values. */
  ShowDebug = 1 << 6         /**< Show debugging information. */
};

/**
 * @brief Represents a 2D curve that supports tangent manipulation and value constraints.
 */
class Curve2D {
  bool tangent_;                  /**< Indicates if the curve uses tangents. */
  std::vector<glm::vec2> values_; /**< Stores the points of the curve. */
  glm::vec2 min_;                 /**< Minimum constraints for the curve's range. */
  glm::vec2 max_;                 /**< Maximum constraints for the curve's range. */

 public:
  /**
   * @brief Constructs a Curve2D with defined ranges and optional tangent.
   *
   * @param min The minimum value constraints for the curve.
   * @param max The maximum value constraints for the curve.
   * @param tangent Whether the curve supports tangents or not.
   */
  explicit Curve2D(const glm::vec2& min = {0, 0}, const glm::vec2& max = {1, 1}, bool tangent = true);

  /**
   * @brief Constructs a Curve2D with start and end values in addition to defined ranges and tangent.
   *
   * @param start The starting Y value of the curve.
   * @param end The ending Y value of the curve.
   * @param min The minimum value constraints for the curve.
   * @param max The maximum value constraints for the curve.
   * @param tangent Whether the curve supports tangents or not.
   */
  Curve2D(float start, float end, const glm::vec2& min = {0, 0}, const glm::vec2& max = {1, 1}, bool tangent = true);

  /**
   * @brief Clears all the points in the curve.
   */
  void Clear();

  /**
   * @brief Retrieves the curve's internal vector of points for modification.
   *
   * @return A reference to the vector of points.
   */
  [[nodiscard]] std::vector<glm::vec2>& UnsafeGetValues();

  /**
   * @brief Enables or disables tangent manipulation for the curve.
   *
   * @param value `true` to enable tangents, `false` to disable.
   */
  void SetTangent(bool value);

  /**
   * @brief Sets the starting Y value of the curve.
   *
   * @param value The start value to set.
   */
  void SetStart(float value);

  /**
   * @brief Sets the ending Y value of the curve.
   *
   * @param value The end value to set.
   */
  void SetEnd(float value);

  /**
   * @brief Checks whether tangents are enabled for the curve.
   *
   * @return `true` if tangents are enabled, otherwise `false`.
   */
  [[nodiscard]] bool IsTangent() const;

  /**
   * @brief Handles curve inspection via an editor UI.
   *
   * @param label Label for the curve in the UI.
   * @param editor_size Size of the editor panel.
   * @param flags Customization flags for the editor.
   * @return `true` if any changes were made during inspection.
   */
  bool OnInspect(const std::string& label, const ImVec2& editor_size = ImVec2(-1, -1),
                 unsigned flags = static_cast<unsigned>(CurveEditorFlags::AllowResize) |
                                  static_cast<unsigned>(CurveEditorFlags::ShowGrid));

  /**
   * @brief Evaluates the value of the curve at a given X-coordinate.
   *
   * @param x The X-coordinate where the curve value is queried.
   * @param iteration Number of iterations for solving the curve value.
   * @return The Y-value on the curve corresponding to the provided X-coordinate.
   */
  [[nodiscard]] float GetValue(float x, unsigned iteration = 8) const;

  /**
   * @brief Serializes the curve data to a YAML emitter.
   *
   * @param name The name of the curve to be serialized.
   * @param out The YAML emitter where the data is written.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Deserializes the curve data from a YAML node.
   *
   * @param name The name of the curve to be deserialized.
   * @param in The YAML node containing the curve data.
   */
  void Load(const std::string& name, const YAML::Node& in);
};

/**
 * @brief Settings for curve descriptors.
 */
struct CurveDescriptorSettings {
  float speed = 0.01f;          /**< Adjustment speed for curve descriptor controls. */
  float min_max_control = true; /**< Enables min/max control for the curve descriptor. */
  float end_adjustment = true;  /**< Enables end adjustment for the curve. */
  std::string m_tip;            /**< Tooltip for the curve descriptor in the UI. */
};

/**
 * @brief Represents a 2D plot composed of a curve and defined value range.
 *
 * @tparam T Type of the plot's minimum and maximum values.
 */
template <class T>
struct Plot2D {
  T min_value = 0;                                     /**< Minimum value for the plot. */
  T max_value = 1;                                     /**< Maximum value for the plot. */
  Curve2D curve = Curve2D(0.5f, 0.5f, {0, 0}, {1, 1}); /**< The curve comprising the plot. */

  /**
   * @brief Default constructor initializing the plot with default values.
   */
  Plot2D();

  /**
   * @brief Parameterized constructor initializing the plot with specified values and curve.
   *
   * @param min Minimum value for the plot.
   * @param max Maximum value for the plot.
   * @param curve The curve defining the plot.
   */
  Plot2D(T min, T max, const Curve2D& curve = Curve2D(0.5f, 0.5f, {0, 0}, {1, 1}));

  /**
   * @brief Inspects the plot via an editor UI.
   *
   * @param name The name of the plot to be displayed in the UI.
   * @param settings Settings for editing the plot descriptor.
   * @return `true` if any changes were made during inspection.
   */
  bool OnInspect(const std::string& name, const CurveDescriptorSettings& settings = {});

  /**
   * @brief Serializes the plot data to a YAML emitter.
   *
   * @param name The name of the plot to be serialized.
   * @param out The YAML emitter where the plot data is written.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Deserializes the plot data from a YAML node.
   *
   * @param name The name of the plot to be deserialized.
   * @param in The YAML node containing the plot data.
   */
  void Load(const std::string& name, const YAML::Node& in);

  /**
   * @brief Calculates the value of the plot at a given time (t).
   *
   * @param t The time parameter ranging from 0 to 1.
   * @return The calculated value of the plot at the specified time.
   */
  [[nodiscard]] T GetValue(float t) const;
};

/**
 * @brief Represents a single distribution with mean and deviation values.
 *
 * @tparam T Type of the mean value.
 */
template <class T>
struct SingleDistribution {
  T mean;                 /**< The mean value of the distribution. */
  float deviation = 0.0f; /**< The deviation of the distribution. */

  /**
   * @brief Inspects the single distribution via an editor UI.
   *
   * @param name The name of the distribution to be displayed in the UI.
   * @param speed Adjustment speed for the distribution controls.
   * @param tip Tooltip string for the UI.
   * @return `true` if any changes were made during inspection.
   */
  bool OnInspect(const std::string& name, float speed = 0.01f, const std::string& tip = "");

  /**
   * @brief Serializes the single distribution data to a YAML emitter.
   *
   * @param name The name of the distribution to be serialized.
   * @param out The YAML emitter where the distribution data is written.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Deserializes the single distribution data from a YAML node.
   *
   * @param name The name of the distribution to be deserialized.
   * @param in The YAML node containing the distribution data.
   */
  void Load(const std::string& name, const YAML::Node& in);

  /**
   * @brief Calculates a random value sampled from the distribution.
   *
   * @return A random value sampled from the distribution.
   */
  [[nodiscard]] T GetValue() const;
};

/**
 * @brief Settings for configuring a plotted distribution.
 */
struct PlottedDistributionSettings {
  float speed = 0.01f;                   /**< Adjustment speed for distribution controls. */
  CurveDescriptorSettings mean_settings; /**< Settings for the mean curve. */
  CurveDescriptorSettings dev_settings;  /**< Settings for the deviation curve. */
  bool show_uncertainty_preview = true;  /**< Displays mean +/- 1sigma and +/- 2sigma bands for float plots. */
  std::string tip;                       /**< Tooltip for the distribution in the UI. */
};

/**
 * @brief Represents a plotted distribution with mean and deviation curves.
 *
 * @tparam T Type of the mean values.
 */
template <class T>
struct PlottedDistribution {
  Plot2D<T> mean;          /**< Plot for the mean values. */
  Plot2D<float> deviation; /**< Plot for the deviation values. */

  /**
   * @brief Inspects the plotted distribution via an editor UI.
   *
   * @param name The name of the distribution to be displayed in the UI.
   * @param settings Settings for configuring the plotted distribution.
   * @return `true` if any changes were made during inspection.
   */
  bool OnInspect(const std::string& name, const PlottedDistributionSettings& settings = {});

  /**
   * @brief Serializes the plotted distribution data to a YAML emitter.
   *
   * @param name The name of the distribution to be serialized.
   * @param out The YAML emitter where the distribution data is written.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Deserializes the plotted distribution data from a YAML node.
   *
   * @param name The name of the distribution to be deserialized.
   * @param in The YAML node containing the distribution data.
   */
  void Load(const std::string& name, const YAML::Node& in);

  /**
   * @brief Calculates a value from the plotted distribution at a given time (t).
   *
   * @param t The time parameter ranging from 0 to 1.
   * @return The calculated value from the distribution at the specified time.
   */
  T GetValue(float t) const;
};

inline void DrawPlottedDistributionUncertaintyPreview(const Plot2D<float>& mean_plot,
                                                      const Plot2D<float>& deviation_plot,
                                                      const char* preview_id) {
  ImVec2 size = ImVec2(ImGui::GetContentRegionAvail().x, 130.0f);
  if (size.x < 80.0f) {
    size.x = 80.0f;
  }

  const ImVec2 pos = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(preview_id, size);

  auto* draw_list = ImGui::GetWindowDrawList();
  const ImVec2 min = pos;
  const ImVec2 max = ImVec2(pos.x + size.x, pos.y + size.y);
  draw_list->AddRectFilled(min, max, IM_COL32(26, 30, 34, 255), 4.0f);
  draw_list->AddRect(min, max, IM_COL32(95, 104, 114, 255), 4.0f);

  constexpr int kSamples = 128;
  std::array<float, kSamples> mean_values{};
  std::array<float, kSamples> sigma_values{};
  float y_min = FLT_MAX;
  float y_max = -FLT_MAX;
  float max_sigma = 0.0f;

  for (int i = 0; i < kSamples; i++) {
    const float t = static_cast<float>(i) / static_cast<float>(kSamples - 1);
    const float mean_v = mean_plot.GetValue(t);
    const float sigma_v = std::max(0.0f, deviation_plot.GetValue(t));
    mean_values[i] = mean_v;
    sigma_values[i] = sigma_v;
    max_sigma = std::max(max_sigma, sigma_v);
    y_min = std::min(y_min, mean_v - 2.0f * sigma_v);
    y_max = std::max(y_max, mean_v + 2.0f * sigma_v);
  }

  if (!std::isfinite(y_min) || !std::isfinite(y_max)) {
    return;
  }

  if (std::abs(y_max - y_min) < 1e-5f) {
    const float expand = std::max(0.1f, std::abs(y_max) * 0.1f + 0.1f);
    y_min -= expand;
    y_max += expand;
  }

  auto to_screen = [&](float t, float y) {
    const float nx = std::clamp(t, 0.0f, 1.0f);
    const float ny = std::clamp((y - y_min) / (y_max - y_min), 0.0f, 1.0f);
    return ImVec2(
        min.x + nx * (size.x - 1.0f),
        max.y - ny * (size.y - 1.0f));
  };

  if (y_min < 0.0f && y_max > 0.0f) {
    const ImVec2 a = to_screen(0.0f, 0.0f);
    const ImVec2 b = to_screen(1.0f, 0.0f);
    draw_list->AddLine(a, b, IM_COL32(140, 148, 156, 110), 1.0f);
  }

  std::array<ImVec2, kSamples> upper_2{};
  std::array<ImVec2, kSamples> lower_2{};
  std::array<ImVec2, kSamples> upper_1{};
  std::array<ImVec2, kSamples> lower_1{};
  std::array<ImVec2, kSamples> mean_line{};

  for (int i = 0; i < kSamples; i++) {
    const float t = static_cast<float>(i) / static_cast<float>(kSamples - 1);
    upper_2[i] = to_screen(t, mean_values[i] + 2.0f * sigma_values[i]);
    lower_2[i] = to_screen(t, mean_values[i] - 2.0f * sigma_values[i]);
    upper_1[i] = to_screen(t, mean_values[i] + 1.0f * sigma_values[i]);
    lower_1[i] = to_screen(t, mean_values[i] - 1.0f * sigma_values[i]);
    mean_line[i] = to_screen(t, mean_values[i]);
  }

  auto draw_band = [&](const std::array<ImVec2, kSamples>& upper,
                       const std::array<ImVec2, kSamples>& lower,
                       const ImU32 color) {
    for (int i = 0; i < kSamples - 1; i++) {
      const float sep0 = std::abs(upper[i].x - lower[i].x) + std::abs(upper[i].y - lower[i].y);
      const float sep1 = std::abs(upper[i + 1].x - lower[i + 1].x) +
                         std::abs(upper[i + 1].y - lower[i + 1].y);
      if (sep0 <= 1e-4f && sep1 <= 1e-4f) {
        continue;
      }
      draw_list->AddQuadFilled(upper[i], upper[i + 1], lower[i + 1], lower[i], color);
    }
  };

  if (max_sigma > 1e-6f) {
    draw_band(upper_2, lower_2, IM_COL32(66, 153, 225, 55));
    draw_band(upper_1, lower_1, IM_COL32(120, 190, 255, 100));
  }

  draw_list->AddPolyline(mean_line.data(), kSamples, IM_COL32(245, 245, 245, 255), 0, 1.7f);

  draw_list->AddText(ImVec2(min.x + 8.0f, min.y + 6.0f), IM_COL32(240, 240, 240, 255), "mean");
  draw_list->AddText(ImVec2(min.x + 54.0f, min.y + 6.0f), IM_COL32(160, 220, 255, 255), "+/-1sigma");
  draw_list->AddText(ImVec2(min.x + 128.0f, min.y + 6.0f), IM_COL32(120, 180, 255, 255), "+/-2sigma");
}

/**
 * @brief Serializes the single distribution data to a YAML emitter.
 *
 * @tparam T Type of the mean value.
 * @param name The name of the distribution to be serialized.
 * @param out The YAML emitter where the distribution data is written.
 */
template <class T>
void SingleDistribution<T>::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  {
    out << YAML::Key << "mean" << YAML::Value << mean;
    out << YAML::Key << "deviation" << YAML::Value << deviation;
  }
  out << YAML::EndMap;
}

/**
 * @brief Deserializes the single distribution data from a YAML node.
 *
 * @tparam T Type of the mean value.
 * @param name The name of the distribution to be deserialized.
 * @param in The YAML node containing the distribution data.
 */
template <class T>
void SingleDistribution<T>::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& cd = in[name];
    if (cd["mean"])
      mean = cd["mean"].as<T>();
    else if (cd["m_mean"])
      mean = cd["m_mean"].as<T>();
    if (cd["deviation"])
      deviation = cd["deviation"].as<float>();
    else if (cd["m_deviation"])
      deviation = cd["m_deviation"].as<float>();
  }
}

/**
 * @brief Inspects the single distribution via an editor UI.
 *
 * @tparam T Type of the mean value.
 * @param name The name of the distribution to be displayed in the UI.
 * @param speed Adjustment speed for the distribution controls.
 * @param tip Tooltip string for the UI.
 * @return `true` if any changes were made during inspection.
 */
template <class T>
bool SingleDistribution<T>::OnInspect(const std::string& name, const float speed, const std::string& tip) {
  bool changed = false;
  ImGui::PushID(name.c_str());
  if (ImGui::BeginTable("SingleDistributionInline", 3,
                        ImGuiTableFlags_NoSavedSettings |
                            ImGuiTableFlags_SizingStretchProp |
                            ImGuiTableFlags_BordersInnerV)) {
    ImGui::TableSetupColumn("Parameter", ImGuiTableColumnFlags_WidthStretch, 0.46f);
    ImGui::TableSetupColumn("Mean", ImGuiTableColumnFlags_WidthStretch, 0.34f);
    ImGui::TableSetupColumn("Deviation", ImGuiTableColumnFlags_WidthStretch, 0.20f);
    ImGui::TableNextRow();

    ImGui::TableSetColumnIndex(0);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(name.c_str());
    if (!tip.empty() && ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::TextUnformatted(tip.c_str());
      ImGui::EndTooltip();
    }

    ImGui::TableSetColumnIndex(1);
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (typeid(T).hash_code() == typeid(float).hash_code()) {
      if (ImGui::DragFloat("##Mean", reinterpret_cast<float*>(&mean), speed))
        changed = true;
    } else if (typeid(T).hash_code() == typeid(glm::vec2).hash_code()) {
      if (ImGui::DragFloat2("##Mean", reinterpret_cast<float*>(&mean), speed))
        changed = true;
    } else if (typeid(T).hash_code() == typeid(glm::vec3).hash_code()) {
      if (ImGui::DragFloat3("##Mean", reinterpret_cast<float*>(&mean), speed))
        changed = true;
    }

    ImGui::TableSetColumnIndex(2);
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::DragFloat("##Deviation", &deviation, speed))
      changed = true;

    ImGui::EndTable();
  }
  ImGui::PopID();
  return changed;
}

/**
 * @brief Calculates a random value sampled from the single distribution.
 *
 * @tparam T Type of the mean value.
 * @return A random value sampled from the distribution.
 */
template <class T>
T SingleDistribution<T>::GetValue() const {
  return glm::gaussRand(mean, T(deviation));
}

/**
 * @brief Inspects the plotted distribution via an editor UI.
 *
 * @tparam T Type of the mean values.
 * @param name The name of the distribution to be displayed in the UI.
 * @param settings Settings for configuring the plotted distribution.
 * @return `true` if any changes were made during inspection.
 */
template <class T>
bool PlottedDistribution<T>::OnInspect(const std::string& name, const PlottedDistributionSettings& settings) {
  bool changed = false;
  if (ImGui::TreeNode(name.c_str())) {
    if (!settings.tip.empty() && ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::TextUnformatted(settings.tip.c_str());
      ImGui::EndTooltip();
    }
    if constexpr (std::is_same_v<T, float>) {
      if (settings.show_uncertainty_preview) {
        ImGui::TextUnformatted("Uncertainty Preview");
        const std::string preview_id = "uncertainty_preview##" + name;
        DrawPlottedDistributionUncertaintyPreview(mean, deviation, preview_id.c_str());
      }
    }

    const auto mean_title = name + " (mean)";
    const auto dev_title = name + " (deviation)";
    const std::string table_id = "PlottedDistributionInline##" + name;
    if (ImGui::BeginTable(table_id.c_str(), 2,
                          ImGuiTableFlags_NoSavedSettings |
                              ImGuiTableFlags_SizingStretchSame |
                              ImGuiTableFlags_BordersInnerV)) {
      ImGui::TableNextRow();

      ImGui::TableSetColumnIndex(0);
      changed |= mean.OnInspect(mean_title, settings.mean_settings);

      ImGui::TableSetColumnIndex(1);
      if (deviation.OnInspect(dev_title, settings.dev_settings))
        changed = true;

      ImGui::EndTable();
    }
    ImGui::TreePop();
  }
  return changed;
}

/**
 * @brief Serializes the plotted distribution data to a YAML emitter.
 *
 * @tparam T Type of the mean values.
 * @param name The name of the distribution to be serialized.
 * @param out The YAML emitter where the distribution data is written.
 */
template <class T>
void PlottedDistribution<T>::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  {
    mean.Save("mean", out);
    deviation.Save("deviation", out);
  }
  out << YAML::EndMap;
}

/**
 * @brief Deserializes the plotted distribution data from a YAML node.
 *
 * @tparam T Type of the mean values.
 * @param name The name of the distribution to be deserialized.
 * @param in The YAML node containing the distribution data.
 */
template <class T>
void PlottedDistribution<T>::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& cd = in[name];
    mean.Load("mean", cd);
    deviation.Load("deviation", cd);

    mean.Load("m_mean", cd);
    deviation.Load("m_deviation", cd);
  }
}

/**
 * @brief Calculates a value from the plotted distribution at a given time (t).
 *
 * @tparam T Type of the mean values.
 * @param t The time parameter ranging from 0 to 1.
 * @return The calculated value from the distribution at the specified time.
 */
template <class T>
T PlottedDistribution<T>::GetValue(float t) const {
  return glm::gaussRand(mean.GetValue(t), T(deviation.GetValue(t)));
}

/**
 * @brief Inspects the plot via an editor UI.
 *
 * @tparam T Type of the plot's minimum and maximum values.
 * @param name The name of the plot to be displayed in the UI.
 * @param settings Settings for editing the plot descriptor.
 * @return `true` if any changes were made during inspection.
 */
template <class T>
bool Plot2D<T>::OnInspect(const std::string& name, const CurveDescriptorSettings& settings) {
  bool changed = false;
  if (ImGui::TreeNode(name.c_str())) {
    if (!settings.m_tip.empty() && ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::TextUnformatted(settings.m_tip.c_str());
      ImGui::EndTooltip();
    }
    if (settings.min_max_control) {
      if (typeid(T).hash_code() == typeid(float).hash_code()) {
        changed = ImGui::DragFloat(("Min##" + name).c_str(), static_cast<float*>(&min_value), settings.speed);
        if (ImGui::DragFloat(("Max##" + name).c_str(), static_cast<float*>(&max_value), settings.speed))
          changed = true;
      } else if (typeid(T).hash_code() == typeid(glm::vec2).hash_code()) {
        changed = ImGui::DragFloat2(("Min##" + name).c_str(), static_cast<float*>(&min_value), settings.speed);
        if (ImGui::DragFloat2(("Max##" + name).c_str(), static_cast<float*>(&max_value), settings.speed))
          changed = true;
      } else if (typeid(T).hash_code() == typeid(glm::vec3).hash_code()) {
        changed = ImGui::DragFloat3(("Min##" + name).c_str(), static_cast<float*>(&min_value), settings.speed);
        if (ImGui::DragFloat3(("Max##" + name).c_str(), static_cast<float*>(&max_value), settings.speed))
          changed = true;
      }
    }
    const auto flag =
        settings.end_adjustment
            ? static_cast<unsigned>(CurveEditorFlags::AllowResize) | static_cast<unsigned>(CurveEditorFlags::ShowGrid)
            : static_cast<unsigned>(CurveEditorFlags::AllowResize) | static_cast<unsigned>(CurveEditorFlags::ShowGrid) |
                  static_cast<unsigned>(CurveEditorFlags::DisableStartEndY);
    if (curve.OnInspect(("Curve2D##" + name).c_str(), ImVec2(-1, -1), flag)) {
      changed = true;
    }

    ImGui::TreePop();
  }
  return changed;
}

/**
 * @brief Serializes the plot data to a YAML emitter.
 *
 * @tparam T Type of the plot's minimum and maximum values.
 * @param name The name of the plot to be serialized.
 * @param out The YAML emitter where the plot data is written.
 */
template <class T>
void Plot2D<T>::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  {
    out << YAML::Key << "min_value" << YAML::Value << min_value;
    out << YAML::Key << "max_value" << YAML::Value << max_value;
    curve.Save("curve", out);
  }
  out << YAML::EndMap;
}

/**
 * @brief Deserializes the plot data from a YAML node.
 *
 * @tparam T Type of the plot's minimum and maximum values.
 * @param name The name of the plot to be deserialized.
 * @param in The YAML node containing the plot data.
 */
template <class T>
void Plot2D<T>::Load(const std::string& name, const YAML::Node& in) {
  if (in[name]) {
    const auto& cd = in[name];
    if (cd["min_value"])
      min_value = cd["min_value"].as<T>();
    else if (cd["m_minValue"])
      min_value = cd["m_minValue"].as<T>();
    if (cd["max_value"])
      max_value = cd["max_value"].as<T>();
    else if (cd["m_maxValue"])
      max_value = cd["m_maxValue"].as<T>();

    curve.Load("curve", cd);
    curve.Load("m_curve", cd);
  }
}

/**
 * @brief Default constructor initializing the plot with default values.
 *
 * @tparam T Type of the plot's minimum and maximum values.
 */
template <class T>
Plot2D<T>::Plot2D() {
  curve = Curve2D(0.5f, 0.5f, {0, 0}, {1, 1});
}

/**
 * @brief Parameterized constructor initializing the plot with specified values and curve.
 *
 * @tparam T Type of the plot's minimum and maximum values.
 * @param min Minimum value for the plot.
 * @param max Maximum value for the plot.
 * @param curve The curve defining the plot.
 */
template <class T>
Plot2D<T>::Plot2D(T min, T max, const Curve2D& curve) {
  min_value = min;
  max_value = max;
  this->curve = curve;
}

/**
 * @brief Calculates the value of the plot at a given time (t).
 *
 * @tparam T Type of the plot's minimum and maximum values.
 * @param t The time parameter ranging from 0 to 1.
 * @return The calculated value of the plot at the specified time.
 */
template <class T>
T Plot2D<T>::GetValue(const float t) const {
  return glm::mix(min_value, max_value, glm::clamp(curve.GetValue(t), 0.0f, 1.0f));
}

}  // namespace evo_engine
