
#pragma once
#ifdef CUDA_MODULE_SERVICE

#  include <CUDAModule.hpp>
namespace digital_agriculture_package {
using namespace evo_engine;

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

  /**
   * @brief Calculates the illumination based on the given ray properties.
   *
   * @param ray_properties The properties of the rays used for illumination calculation.
   * @param seed A seed value for randomization in the calculation.
   * @param push_normal_distance A small distance to push along the normal to avoid self-intersections.
   */
  void CalculateIllumination(const RayProperties& ray_properties, int seed, float push_normal_distance);
};

}  // namespace digital_agriculture_package

#endif
