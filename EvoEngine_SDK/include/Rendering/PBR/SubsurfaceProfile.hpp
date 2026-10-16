#pragma once

#include <glm/glm.hpp>
#include "IAsset.hpp"

namespace evo_engine {
enum class SubsurfaceMode : uint32_t { Solid = 0, ThinSurface = 1 };

class EVOENGINE_API SubsurfaceProfile final : public IAsset {
 public:
  glm::vec3 albedo{1.0f};
  glm::vec3 scattering_distance{0.001f, 0.0005f, 0.00025f};
  float ior = 1.4f;

  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }
};

// Normalized two-exponential diffusion; the radial PDF includes the polar Jacobian.
EVOENGINE_API float SubsurfaceRadialPdf(float radius, float distance);
EVOENGINE_API float SampleSubsurfaceRadius(float u, float distance);
EVOENGINE_API glm::vec3 SubsurfaceThinTransmission(float thickness, glm::vec3 distance);
}  // namespace evo_engine
