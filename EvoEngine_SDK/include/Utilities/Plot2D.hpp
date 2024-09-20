#pragma once
namespace evo_engine {
enum class CurveEditorFlags {
  ShowGrid = 1 << 1,
  Reset = 1 << 2,
  AllowResize = 1 << 3,
  AllowRemoveSides = 1 << 4,
  DisableStartEndY = 1 << 5,
  ShowDebug = 1 << 6
};

class Curve2D {
  bool tangent_;
  std::vector<glm::vec2> values_;
  glm::vec2 min_;
  glm::vec2 max_;

 public:
  explicit Curve2D(const glm::vec2& min = {0, 0}, const glm::vec2& max = {1, 1}, bool tangent = true);
  Curve2D(float start, float end, const glm::vec2& min = {0, 0}, const glm::vec2& max = {1, 1}, bool tangent = true);
  void Clear();
  [[nodiscard]] std::vector<glm::vec2>& UnsafeGetValues();
  void SetTangent(bool value);
  void SetStart(float value);
  void SetEnd(float value);
  [[nodiscard]] bool IsTangent() const;
  bool OnInspect(const std::string& label, const ImVec2& editor_size = ImVec2(-1, -1),
                 unsigned flags = static_cast<unsigned>(CurveEditorFlags::AllowResize) |
                                  static_cast<unsigned>(CurveEditorFlags::ShowGrid));
  [[nodiscard]] float GetValue(float x, unsigned iteration = 8) const;

  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
};

struct CurveDescriptorSettings {
  float speed = 0.01f;
  float min_max_control = true;
  float end_adjustment = true;
  std::string m_tip;
};

template <class T>
struct Plot2D {
  T min_value = 0;
  T max_value = 1;
  Curve2D curve = Curve2D(0.5f, 0.5f, {0, 0}, {1, 1});
  Plot2D();
  Plot2D(T min, T max, Curve2D curve = Curve2D(0.5f, 0.5f, {0, 0}, {1, 1}));
  bool OnInspect(const std::string& name, const CurveDescriptorSettings& settings = {});
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);

  [[nodiscard]] T GetValue(float t) const;
};
template <class T>
struct SingleDistribution {
  T mean;
  float deviation = 0.0f;
  bool OnInspect(const std::string& name, float speed = 0.01f, const std::string& tip = "");
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
  [[nodiscard]] T GetValue() const;
};

struct PlottedDistributionSettings {
  float speed = 0.01f;
  CurveDescriptorSettings mean_settings;
  CurveDescriptorSettings dev_settings;
  std::string tip;
};

template <class T>
struct PlottedDistribution {
  Plot2D<T> mean;
  Plot2D<float> deviation;
  bool OnInspect(const std::string& name, const PlottedDistributionSettings& settings = {});
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
  T GetValue(float t) const;
};

template <class T>
void SingleDistribution<T>::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  {
    out << YAML::Key << "mean" << YAML::Value << mean;
    out << YAML::Key << "deviation" << YAML::Value << deviation;
  }
  out << YAML::EndMap;
}
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
template <class T>
bool SingleDistribution<T>::OnInspect(const std::string& name, float speed, const std::string& tip) {
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
template <class T>
T SingleDistribution<T>::GetValue() const {
  return glm::gaussRand(mean, T(deviation));
}

template <class T>
bool PlottedDistribution<T>::OnInspect(const std::string& name, const PlottedDistributionSettings& settings) {
  bool changed = false;
  if (ImGui::TreeNode(name.c_str())) {
    if (!settings.tip.empty() && ImGui::IsItemHovered()) {
      ImGui::BeginTooltip();
      ImGui::TextUnformatted(settings.tip.c_str());
      ImGui::EndTooltip();
    }
    auto mean_title = name + " (mean)";
    const auto dev_title = name + " (deviation)";
    changed = mean.OnInspect(mean_title, settings.mean_settings);
    if (deviation.OnInspect(dev_title, settings.dev_settings))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

template <class T>
void PlottedDistribution<T>::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  {
    mean.Save("mean", out);
    deviation.Save("deviation", out);
  }
  out << YAML::EndMap;
}
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
template <class T>
T PlottedDistribution<T>::GetValue(float t) const {
  return glm::gaussRand(mean.GetValue(t), T(deviation.GetValue(t)));
}

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
    auto flag =
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
template <class T>
Plot2D<T>::Plot2D() {
  curve = evo_engine::Curve2D(0.5f, 0.5f, {0, 0}, {1, 1});
}
template <class T>
Plot2D<T>::Plot2D(T min, T max, const evo_engine::Curve2D curve) {
  min_value = min;
  max_value = max;
  this->curve = curve;
}
template <class T>
T Plot2D<T>::GetValue(const float t) const {
  return glm::mix(min_value, max_value, glm::clamp(curve.GetValue(t), 0.0f, 1.0f));
}
}  // namespace evo_engine