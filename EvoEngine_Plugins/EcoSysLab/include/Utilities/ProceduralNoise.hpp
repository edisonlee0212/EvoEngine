
#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "Skeleton.hpp"
using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @enum ProceduralNoiseOperatorType
 * @brief Defines various operator types for procedural noise calculations.
 */
enum class ProceduralNoiseOperatorType {
  Empty,     ///< No operation
  Reset,     ///< Reset to initial value
  Add,       ///< Add operation
  Subtract,  ///< Subtract operation
  Multiply,  ///< Multiply operation
  Divide,    ///< Divide operation
  Pow,       ///< Power operation
  Min,       ///< Minimum operation
  Max,       ///< Maximum operation
  FlipUp,    ///< Flip upwards operation
  FlipDown   ///< Flip downwards operation
};

/**
 * @enum ProceduralNoiseValueType
 * @brief Defines types of procedural noise values.
 */
enum class ProceduralNoiseValueType {
  Constant,  ///< Constant value
  Linear,    ///< Linearly varying value
  Sine,      ///< Sine wave
  Tangent,   ///< Tangent wave
  Simplex,   ///< Simplex noise
  Perlin     ///< Perlin noise
};

/**
 * @struct ProceduralNoiseStage
 * @brief Represents a stage in the procedural noise processing pipeline.
 * @tparam T Type of the value being processed.
 */
template <typename T>
struct ProceduralNoiseStage {
  std::string m_name = "New node";                                                  ///< Name of the noise stage
  ProceduralNoiseOperatorType m_operatorType = ProceduralNoiseOperatorType::Empty;  ///< Operator type
  ProceduralNoiseValueType m_valueType = ProceduralNoiseValueType::Constant;        ///< Value type
  T m_frequency = T(1.0f);                                                          ///< Frequency of noise
  float m_constantValue = 0.f;                                                      ///< Constant value
  T m_offset = T(0.0f);                                                             ///< Offset applied to noise values

  /**
   * @brief Serializes the noise stage to a YAML emitter.
   * @param out The YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the noise stage from a YAML node.
   * @param in The YAML node.
   */
  void Deserialize(const YAML::Node& in);

  /**
   * @brief Saves the noise stage under a given name.
   * @param name The name under which to save.
   * @param out The YAML emitter.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads the noise stage from a YAML node.
   * @param name The name of the node to load from.
   * @param in The YAML node.
   */
  void Load(const std::string& name, const YAML::Node& in);

  /**
   * @brief Computes the noise value based on the given sample point.
   * @param samplePoint Input sample point.
   * @param value The value to be modified by this stage.
   */
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

/**
 * @struct ProceduralNoiseFlowData
 * @brief Represents procedural noise flow data.
 */
struct ProceduralNoiseFlowData {};

/**
 * @struct ProceduralNoiseSkeletonData
 * @brief Represents skeleton data for procedural noise.
 */
struct ProceduralNoiseSkeletonData {};

/**
 * @class ProceduralNoise2D
 * @brief Represents a procedural noise generation system for 2D data.
 */
class ProceduralNoise2D : public IAsset {
  /**
   * @brief Handles inspection of the given skeleton node.
   * @param node_handle The handle to the skeleton node.
   * @return True if no modifications were made, otherwise false.
   */
  bool OnInspect(SkeletonNodeHandle node_handle);

  /**
   * @brief Processes the provided sample point and modifies the value.
   * @param nodeHandle The handle to the skeleton node.
   * @param samplePoint The sample point.
   * @param value The value to modify.
   * @return The processed value.
   */
  float Process(SkeletonNodeHandle nodeHandle, const glm::vec2& samplePoint, float value);

 public:
  Skeleton<ProceduralNoiseSkeletonData, ProceduralNoiseFlowData, ProceduralNoiseStage<glm::vec2>>
      m_pipeline{};  ///< Processing pipeline

  /**
   * @brief Serializes the asset to YAML.
   * @param out The YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the asset from YAML.
   * @param in The YAML node.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Inspects the asset in the editor.
   * @param editorLayer The editor layer.
   * @return True if content was not modified.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

  /**
   * @brief Processes the provided sample point and modifies the value.
   * @param samplePoint The sample point.
   * @param value The value to modify.
   * @return The processed value.
   */
  float Process(const glm::vec2& samplePoint, float value);
};

/**
 * @class ProceduralNoise3D
 * @brief Represents a procedural noise generation system for 3D data.
 */
class ProceduralNoise3D : public IAsset {
  /**
   * @brief Handles inspection of the given skeleton node.
   * @param nodeHandle Handle to the skeleton node.
   * @return True if no modifications were made, otherwise false.
   */
  bool OnInspect(SkeletonNodeHandle nodeHandle);

  /**
   * @brief Processes the provided sample point and modifies the value.
   * @param nodeHandle The handle to the skeleton node.
   * @param samplePoint The sample point.
   * @param value The value to modify.
   * @return The processed value.
   */
  float Process(SkeletonNodeHandle nodeHandle, const glm::vec3& samplePoint, float value);

 public:
  Skeleton<ProceduralNoiseSkeletonData, ProceduralNoiseFlowData, ProceduralNoiseStage<glm::vec3>>
      m_pipeline{};  ///< Processing pipeline

  /**
   * @brief Processes the provided sample point and modifies the value.
   * @param samplePoint The sample point.
   * @param value The value to modify.
   * @return The processed value.
   */
  float Process(const glm::vec3& samplePoint, float value);

  /**
   * @brief Serializes the asset to YAML.
   * @param out The YAML emitter.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the asset from YAML.
   * @param in The YAML node.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Inspects the asset in the editor.
   * @param editorLayer The editor layer.
   * @return True if content was not modified.
   */
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
    case ProceduralNoiseOperatorType::Empty:
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

}  // namespace eco_sys_lab_plugin
