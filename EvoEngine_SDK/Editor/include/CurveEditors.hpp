#pragma once

#include <imgui.h>
#include <type_traits>
#include "Curve.hpp"
#include "EvoEngineEditorAPI.hpp"
#include "Plot2D.hpp"

namespace evo_engine {
enum class CurveEditorFlags {
  ShowGrid = 1 << 1,         /**< Display a grid within the editor. */
  Reset = 1 << 2,            /**< Reset the editor settings. */
  AllowResize = 1 << 3,      /**< Allow resizing of the editor. */
  AllowRemoveSides = 1 << 4, /**< Allow removal of sides. */
  DisableStartEndY = 1 << 5, /**< Prevent modification of start/end Y values. */
  ShowDebug = 1 << 6         /**< Show debugging information. */
};

struct CurveDescriptorSettings {
  float speed = 0.01f;          /**< Adjustment speed for curve descriptor controls. */
  float min_max_control = true; /**< Enables min/max control for the curve descriptor. */
  float end_adjustment = true;  /**< Enables end adjustment for the curve. */
  std::string m_tip;            /**< Tooltip for the curve descriptor in the UI. */
};

struct PlottedDistributionSettings {
  float speed = 0.01f;                   /**< Adjustment speed for distribution controls. */
  CurveDescriptorSettings mean_settings; /**< Settings for the mean curve. */
  CurveDescriptorSettings dev_settings;  /**< Settings for the deviation curve. */
  std::string tip;                       /**< Tooltip for the distribution in the UI. */
};
namespace editor_widgets {
EVOENGINE_EDITOR_API bool Draw(Curve2D& curve, const std::string& label, const ImVec2& editor_size = ImVec2(-1, -1),
                               unsigned flags = static_cast<unsigned>(CurveEditorFlags::AllowResize) |
                                                static_cast<unsigned>(CurveEditorFlags::ShowGrid));
EVOENGINE_EDITOR_API void Draw(BezierSpline& spline);
template <class T>
bool Draw(SingleDistribution<T>& value, const std::string& name, const float speed = 0.01f,
          const std::string& tip = "") {
  bool changed = false;
  if (ImGui::TreeNode(name.c_str())) {
    if (!tip.empty() && ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::TextUnformatted(tip.c_str());
      ImGui::EndTooltip();
    }
    if constexpr (std::is_same_v<T, float>) {
      changed = ImGui::DragFloat("Mean", reinterpret_cast<float*>(&value.mean), speed);
    } else if constexpr (std::is_same_v<T, glm::vec2>) {
      changed = ImGui::DragFloat2("Mean", reinterpret_cast<float*>(&value.mean), speed);
    } else if constexpr (std::is_same_v<T, glm::vec3>) {
      changed = ImGui::DragFloat3("Mean", reinterpret_cast<float*>(&value.mean), speed);
    }
    if (ImGui::DragFloat("Deviation", &value.deviation, speed))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

template <class T>
bool Draw(Plot2D<T>& value, const std::string& name, const CurveDescriptorSettings& settings = {}) {
  bool changed = false;
  if (ImGui::TreeNode(name.c_str())) {
    if (!settings.m_tip.empty() && ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::TextUnformatted(settings.m_tip.c_str());
      ImGui::EndTooltip();
    }
    if (settings.min_max_control) {
      if constexpr (std::is_same_v<T, float>) {
        changed =
            ImGui::DragFloat(("Min##" + name).c_str(), reinterpret_cast<float*>(&value.min_value), settings.speed);
        if (ImGui::DragFloat(("Max##" + name).c_str(), reinterpret_cast<float*>(&value.max_value), settings.speed))
          changed = true;
      } else if constexpr (std::is_same_v<T, glm::vec2>) {
        changed =
            ImGui::DragFloat2(("Min##" + name).c_str(), reinterpret_cast<float*>(&value.min_value), settings.speed);
        if (ImGui::DragFloat2(("Max##" + name).c_str(), reinterpret_cast<float*>(&value.max_value), settings.speed))
          changed = true;
      } else if constexpr (std::is_same_v<T, glm::vec3>) {
        changed =
            ImGui::DragFloat3(("Min##" + name).c_str(), reinterpret_cast<float*>(&value.min_value), settings.speed);
        if (ImGui::DragFloat3(("Max##" + name).c_str(), reinterpret_cast<float*>(&value.max_value), settings.speed))
          changed = true;
      }
    }
    const auto flag =
        settings.end_adjustment
            ? static_cast<unsigned>(CurveEditorFlags::AllowResize) | static_cast<unsigned>(CurveEditorFlags::ShowGrid)
            : static_cast<unsigned>(CurveEditorFlags::AllowResize) | static_cast<unsigned>(CurveEditorFlags::ShowGrid) |
                  static_cast<unsigned>(CurveEditorFlags::DisableStartEndY);
    if (Draw(value.curve, ("Curve2D##" + name).c_str(), ImVec2(-1, -1), flag)) {
      changed = true;
    }

    ImGui::TreePop();
  }
  return changed;
}

template <class T>
bool Draw(PlottedDistribution<T>& value, const std::string& name, const PlottedDistributionSettings& settings = {}) {
  bool changed = false;
  if (ImGui::TreeNode(name.c_str())) {
    if (!settings.tip.empty() && ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::TextUnformatted(settings.tip.c_str());
      ImGui::EndTooltip();
    }
    auto mean_title = name + " (mean)";
    const auto dev_title = name + " (deviation)";
    changed = Draw(value.mean, mean_title, settings.mean_settings);
    if (Draw(value.deviation, dev_title, settings.dev_settings))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}
}  // namespace editor_widgets
}  // namespace evo_engine
