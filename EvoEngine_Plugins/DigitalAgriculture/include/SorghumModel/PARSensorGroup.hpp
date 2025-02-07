
#pragma once
#ifdef CUDA_MODULE_PLUGIN

#  include <CUDAModule.hpp>
using namespace evo_engine;

namespace digital_agriculture_plugin {

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
  std::vector<IlluminationSampler<glm::vec3>> m_samplers;

  /**
   * @brief Calculates the illumination based on the given ray properties.
   *
   * @param ray_properties The properties of the rays used for illumination calculation.
   * @param seed A seed value for randomization in the calculation.
   * @param push_normal_distance A small distance to push along the normal to avoid self-intersections.
   */
  void CalculateIllumination(const RayProperties& ray_properties, int seed, float push_normal_distance);

  /**
   * @brief Inspects the PARSensorGroup in the editor layer.
   *
   * This function provides an interface for inspecting the asset in an editor environment.
   *
   * @param editor_layer A shared pointer to the editor layer.
   * @return True if the asset's content is not modified during inspection, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Serializes the object's data into a YAML emitter.
   *
   * @param out The YAML emitter used to store the serialized data.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the object's data from a YAML node.
   *
   * @param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;
};

}  // namespace digital_agriculture_plugin

#endif
