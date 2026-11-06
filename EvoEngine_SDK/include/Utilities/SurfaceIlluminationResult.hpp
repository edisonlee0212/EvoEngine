#pragma once

#include <glm/glm.hpp>

namespace evo_engine {

/** Geometric area, incident flux summed over sampled faces, and area-weighted face-mean irradiance.
 * For double-sided surfaces total_flux is twice average_flux times area; for mixed surfaces it need not be.
 */
struct SurfaceIlluminationResult {
  float total_area = 0.0f;
  glm::vec3 total_flux = glm::vec3(0.0f);
  glm::vec3 average_flux = glm::vec3(0.0f);
};

}  // namespace evo_engine
