
#pragma once

#include <glm/glm.hpp>
#include <vector>

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @brief Enum representing different types of noise functions.
 */
enum class NoiseType {
  Constant,  ///< Constant noise.
  Linear,    ///< Linear noise.
  Simplex,   ///< Simplex noise.
  Perlin,    ///< Perlin noise.
};

/**
 * @brief Represents the parameters of a noise function.
 */
struct NoiseDescriptor {
  unsigned type = 0;                  ///< The type of noise function used.
  float frequency = 0.1f;             ///< The frequency of the noise.
  float intensity = 1.0f;             ///< The intensity of the noise effect.
  float multiplier = 1.0f;            ///< A multiplier applied to the noise output.
  float min = -10;                    ///< The minimum output value of the noise function.
  float max = 10;                     ///< The maximum output value of the noise function.
  float offset = 0.0f;                ///< Offset applied to the noise computation.
  glm::vec3 shift = glm::vec3(0.0f);  ///< Shift applied to the noise computation.
  bool ridgid = false;                ///< Whether the noise should be rigid.

  /**
   * @brief Serializes the noise descriptor to a YAML emitter.
   * @param out The YAML emitter to serialize data into.
   */
  void Serialize(YAML::Emitter& out) const;

  /**
   * @brief Deserializes the noise descriptor from a YAML node.
   * @param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in);
};

/**
 * @brief Class representing a 2D noise function.
 */
class Noise2D {
 public:
  glm::vec2 min_max = glm::vec2(0, 1);  ///< Minimum and maximum output range of the noise.

  std::vector<NoiseDescriptor> noise_descriptors;  ///< List of noise layers applied.

  /**
   * @brief Constructs a Noise2D object.
   */
  Noise2D();

  /**
   * @brief Handles inspection of noise parameters in the editor.
   * @return True if the asset's content is not modified during inspection.
   */
  bool OnInspect();

  /**
   * @brief Saves the noise parameters to a YAML emitter.
   * @param name The name of the noise configuration.
   * @param out The YAML emitter to save data into.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads noise parameters from a YAML node.
   * @param name The name of the noise configuration.
   * @param in The YAML node containing the serialized data.
   */
  void Load(const std::string& name, const YAML::Node& in);

  /**
   * @brief Applies a random offset to the noise.
   * @param min The minimum offset value.
   * @param max The maximum offset value.
   */
  void RandomOffset(float min, float max);

  /**
   * @brief Computes the noise value at a given position.
   * @param position The position in 2D space.
   * @return The computed noise value.
   */
  [[nodiscard]] float GetValue(const glm::vec2& position) const;
};

/**
 * @brief Class representing a 3D noise function.
 */
class Noise3D {
 public:
  glm::vec2 min_max = glm::vec2(0, 1);  ///< Minimum and maximum output range of the noise.

  std::vector<NoiseDescriptor> noise_descriptors;  ///< List of noise layers applied.

  /**
   * @brief Constructs a Noise3D object.
   */
  Noise3D();

  /**
   * @brief Handles inspection of noise parameters in the editor.
   * @return True if the asset's content is not modified during inspection.
   */
  bool OnInspect();

  /**
   * @brief Saves the noise parameters to a YAML emitter.
   * @param name The name of the noise configuration.
   * @param out The YAML emitter to save data into.
   */
  void Save(const std::string& name, YAML::Emitter& out) const;

  /**
   * @brief Loads noise parameters from a YAML node.
   * @param name The name of the noise configuration.
   * @param in The YAML node containing the serialized data.
   */
  void Load(const std::string& name, const YAML::Node& in);

  /**
   * @brief Computes the noise value at a given position.
   * @param position The position in 3D space.
   * @return The computed noise value.
   */
  [[nodiscard]] float GetValue(const glm::vec3& position) const;
};

}  // namespace eco_sys_lab_plugin
