
#pragma once
#include "IAsset.hpp"
#include "Vertex.hpp"

namespace digital_agriculture_package {
using namespace evo_engine;

template <typename T>
struct IlluminationSampler {
  Vertex v_0;
  Vertex v_1;
  Vertex v_2;
  glm::vec3 direction{};
  T energy{};
  bool front_face = true;
  bool back_face = true;

  [[nodiscard]] float GetArea() const {
    const float a = glm::distance(v_0.position, v_1.position);
    const float b = glm::distance(v_1.position, v_2.position);
    const float c = glm::distance(v_2.position, v_0.position);
    const float p = (a + b + c) * 0.5f;
    return glm::sqrt(p * (p - a) * (p - b) * (p - c));
  }

  [[nodiscard]] glm::vec3 GetCenter() const {
    return (v_0.position + v_1.position + v_2.position) / 3.0f;
  }
};

/**
 * @class PARSensorGroup
 * @brief A class representing a group of PAR (Photosynthetically Active Radiation) sensors.
 *
 * This class is responsible for managing a collection of illumination samplers and calculating the illumination.
 */
class PARSensorGroup : public IAsset {
 public:
  /**
   * @brief A collection of illumination samplers that measure light properties.
   */
  std::vector<IlluminationSampler<glm::vec3>> samplers;
};

}  // namespace digital_agriculture_package
