#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "NodeGraph.hpp"
#include "Skeleton.hpp"
using namespace evo_engine;

namespace eco_sys_lab {
enum class ProceduralNoiseOperatorType {
  None,
  Reset,
  Add,
  Subtract,
  Multiply,
  Divide,
  Pow,
  Min,
  Max,
  FlipUp,
  FlipDown
};

enum class ProceduralNoiseValueType { Constant, Linear, Sine, Tangent, Simplex, Perlin };

template <typename T>
struct ProceduralNoiseStage {
  std::string m_name = "New node";
  ProceduralNoiseOperatorType m_operatorType = ProceduralNoiseOperatorType::None;
  ProceduralNoiseValueType m_valueType = ProceduralNoiseValueType::Constant;
  T m_frequency = T(1.0f);
  float m_constantValue = 0.f;
  T m_offset = T(0.0f);
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  void Save(const std::string& name, YAML::Emitter& out) const;
  void Load(const std::string& name, const YAML::Node& in);
  void Calculate(const T& samplePoint, float& value) const;
};

template <typename T>
void ProceduralNoiseStage<T>::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_name" << YAML::Value << m_name;
  out << YAML::Key << "m_operatorType" << YAML::Value << static_cast<unsigned>(m_operatorType);
  out << YAML::Key << "m_valueType" << YAML::Value << static_cast<unsigned>(m_valueType);
  out << YAML::Key << "frequency" << YAML::Value << m_frequency;
  out << YAML::Key << "m_constantValue" << YAML::Value << m_constantValue;
  out << YAML::Key << "m_offset" << YAML::Value << m_offset;
}

template <typename T>
void ProceduralNoiseStage<T>::Deserialize(const YAML::Node& in) {
  if (in["m_name"])
    m_name = in["m_name"].as<std::string>();
  if (in["m_operatorType"])
    m_operatorType = static_cast<ProceduralNoiseOperatorType>(in["m_operatorType"].as<unsigned>());
  if (in["m_valueType"])
    m_valueType = static_cast<ProceduralNoiseValueType>(in["m_valueType"].as<unsigned>());
  if (in["m_frequency"])
    m_frequency = in["m_frequency"].as<T>();
  if (in["m_constantValue"])
    m_constantValue = in["m_constantValue"].as<float>();
  if (in["m_offset"])
    m_offset = in["m_offset"].as<T>();
}

template <typename T>
void ProceduralNoiseStage<T>::Save(const std::string& name, YAML::Emitter& out) const {
  out << YAML::Key << name << YAML::Value << YAML::BeginMap;
  Serialize(out);
  out << YAML::EndMap;
}

template <typename T>
void ProceduralNoiseStage<T>::Load(const std::string& name, const YAML::Node& in) {
  if (in[name])
    Deserialize(in[name]);
}

struct ProceduralNoiseFlowData {};
struct ProceduralNoiseSkeletonData {};

class ProceduralNoise2D : public IAsset {
  bool OnInspect(SkeletonNodeHandle node_handle);
  float Process(SkeletonNodeHandle nodeHandle, const glm::vec2& samplePoint, float value);

 public:
  Skeleton<ProceduralNoiseSkeletonData, ProceduralNoiseFlowData, ProceduralNoiseStage<glm::vec2>> m_pipeline{};
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  float Process(const glm::vec2& samplePoint, float value);
};
class ProceduralNoise3D : public IAsset {
  bool OnInspect(SkeletonNodeHandle nodeHandle);
  float Process(SkeletonNodeHandle nodeHandle, const glm::vec3& samplePoint, float value);

 public:
  Skeleton<ProceduralNoiseSkeletonData, ProceduralNoiseFlowData, ProceduralNoiseStage<glm::vec3>> m_pipeline{};
  float Process(const glm::vec3& samplePoint, float value);
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
};

template <typename T>
void ProceduralNoiseStage<T>::Calculate(const T& samplePoint, float& value) const {
  float stageValue = 0.f;
  const auto actualSamplePoint = (samplePoint + m_offset) * m_frequency;
  switch (m_valueType) {
    case ProceduralNoiseValueType::Constant:
      stageValue = m_constantValue;
      break;
    case ProceduralNoiseValueType::Linear:
      stageValue = actualSamplePoint.x + actualSamplePoint.y;
      break;
    case ProceduralNoiseValueType::Sine:
      stageValue = glm::sin(actualSamplePoint.x) + glm::sin(actualSamplePoint.y);
      break;
    case ProceduralNoiseValueType::Tangent:
      stageValue = glm::tan(actualSamplePoint.x) + glm::tan(actualSamplePoint.y);
      break;
    case ProceduralNoiseValueType::Simplex:
      stageValue = glm::simplex(actualSamplePoint);
      break;
    case ProceduralNoiseValueType::Perlin:
      stageValue = glm::perlin(actualSamplePoint);
      break;
  }

  switch (m_operatorType) {
    case ProceduralNoiseOperatorType::None:
      break;
    case ProceduralNoiseOperatorType::Add:
      value += stageValue;
      break;
    case ProceduralNoiseOperatorType::Subtract:
      value -= stageValue;
      break;
    case ProceduralNoiseOperatorType::Multiply:
      value *= stageValue;
      break;
    case ProceduralNoiseOperatorType::Divide:
      value /= stageValue;
      break;
    case ProceduralNoiseOperatorType::Pow:
      value = glm::pow(value, stageValue);
      break;
    case ProceduralNoiseOperatorType::Min:
      value = glm::min(value, stageValue);
      break;
    case ProceduralNoiseOperatorType::Max:
      value = glm::max(value, stageValue);
      break;
    case ProceduralNoiseOperatorType::FlipUp:
      value = glm::abs(value - stageValue) + stageValue;
      break;
    case ProceduralNoiseOperatorType::FlipDown:
      value = -glm::abs(-value + stageValue) + stageValue;
      break;
    case ProceduralNoiseOperatorType::Reset:
      value = stageValue;
      break;
  }
}

}  // namespace eco_sys_lab
