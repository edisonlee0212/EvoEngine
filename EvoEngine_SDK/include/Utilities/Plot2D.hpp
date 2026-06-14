
#pragma once

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
   * @brief Draws curve editing controls.
   *
   * @param label Label for the curve in the UI.
   * @param editor_size Size of the editor panel.
   * @param flags Customization flags for the editor.
   * @return `true` if any changes were made.
   */
  bool Draw(const std::string& label, const ImVec2& editor_size = ImVec2(-1, -1),
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
   * @brief Draws plot editing controls.
   *
   * @param name The name of the plot to be displayed in the UI.
   * @param settings Settings for editing the plot descriptor.
   * @return `true` if any changes were made.
   */
  bool Draw(const std::string& name, const CurveDescriptorSettings& settings = {});

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
   * @brief Draws single distribution editing controls.
   *
   * @param name The name of the distribution to be displayed in the UI.
   * @param speed Adjustment speed for the distribution controls.
   * @param tip Tooltip string for the UI.
   * @return `true` if any changes were made.
   */
  bool Draw(const std::string& name, float speed = 0.01f, const std::string& tip = "");

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
   * @brief Draws plotted distribution editing controls.
   *
   * @param name The name of the distribution to be displayed in the UI.
   * @param settings Settings for configuring the plotted distribution.
   * @return `true` if any changes were made.
   */
  bool Draw(const std::string& name, const PlottedDistributionSettings& settings = {});

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
 * @brief Draws single distribution editing controls.
 *
 * @tparam T Type of the mean value.
 * @param name The name of the distribution to be displayed in the UI.
 * @param speed Adjustment speed for the distribution controls.
 * @param tip Tooltip string for the UI.
 * @return `true` if any changes were made.
 */
template <class T>
bool SingleDistribution<T>::Draw(const std::string& name, const float speed, const std::string& tip) {
  bool changed = false;
  if (ImGui::TreeNode(name.c_str())) {
    if (!tip.empty() && ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::TextUnformatted(tip.c_str());
      ImGui::EndTooltip();
    }
    if (typeid(T).hash_code() == typeid(float).hash_code()) {
      changed = ImGui::DragFloat("Mean", reinterpret_cast<float*>(&mean), speed);
    } else if (typeid(T).hash_code() == typeid(glm::vec2).hash_code()) {
      changed = ImGui::DragFloat2("Mean", reinterpret_cast<float*>(&mean), speed);
    } else if (typeid(T).hash_code() == typeid(glm::vec3).hash_code()) {
      changed = ImGui::DragFloat3("Mean", reinterpret_cast<float*>(&mean), speed);
    }
    if (ImGui::DragFloat("Deviation", &deviation, speed))
      changed = true;
    ImGui::TreePop();
  }
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
 * @brief Draws plotted distribution editing controls.
 *
 * @tparam T Type of the mean values.
 * @param name The name of the distribution to be displayed in the UI.
 * @param settings Settings for configuring the plotted distribution.
 * @return `true` if any changes were made.
 */
template <class T>
bool PlottedDistribution<T>::Draw(const std::string& name, const PlottedDistributionSettings& settings) {
  bool changed = false;
  if (ImGui::TreeNode(name.c_str())) {
    if (!settings.tip.empty() && ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::TextUnformatted(settings.tip.c_str());
      ImGui::EndTooltip();
    }
    auto mean_title = name + " (mean)";
    const auto dev_title = name + " (deviation)";
    changed = mean.Draw(mean_title, settings.mean_settings);
    if (deviation.Draw(dev_title, settings.dev_settings))
      changed = true;
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
 * @brief Draws plot editing controls.
 *
 * @tparam T Type of the plot's minimum and maximum values.
 * @param name The name of the plot to be displayed in the UI.
 * @param settings Settings for editing the plot descriptor.
 * @return `true` if any changes were made.
 */
template <class T>
bool Plot2D<T>::Draw(const std::string& name, const CurveDescriptorSettings& settings) {
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
    if (curve.Draw(("Curve2D##" + name).c_str(), ImVec2(-1, -1), flag)) {
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
