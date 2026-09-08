#pragma once

#include "HddagiSettings.hpp"
#include "SdfgiScene.hpp"

namespace evo_engine {
struct HddagiUpdateRegion {
  SdfgiPendingRegion core;
  SdfgiPendingRegion light;
  SdfgiPendingRegion raster;
};
struct HddagiUpdatePlan {
  std::vector<SdfgiCascade> cascades;
  std::vector<glm::ivec3> scroll;
  std::vector<HddagiUpdateRegion> regions;
  uint32_t full_cascades = 0;
  uint32_t reset_history_cascades = 0;
  uint64_t region_count = 0;
};
EVOENGINE_API std::string BuildHddagiUpdatePlan(const GiProbeSettings& probes, const HddagiSettings& settings,
                                                glm::vec3 anchor, const std::vector<SdfgiCascade>& previous,
                                                const std::vector<SdfgiContributorChange>& changes, bool force_full,
                                                HddagiUpdatePlan& output);
}  // namespace evo_engine
