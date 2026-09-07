#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace YAML {
class Emitter;
class Node;
}  // namespace YAML

namespace evo_engine {
struct SdfgiSettings;

struct EVOENGINE_API GiProbeSettings {
  enum class VerticalScale : uint32_t { Percent50 = 0, Percent75 = 1, Percent100 = 2 };
  uint32_t probe_count_x = 33;
  uint32_t probe_count_y = 17;
  uint32_t cascade_count = 4;
  float base_probe_distance = 0.8f;
  VerticalScale vertical_scale = VerticalScale::Percent75;
  uint64_t anchor_camera_entity = 0;

  [[nodiscard]] glm::ivec3 ProbeSize() const;
  [[nodiscard]] glm::vec3 Interval(uint32_t cascade) const;
  [[nodiscard]] std::string Validate() const;
  [[nodiscard]] bool operator==(const GiProbeSettings& other) const;
};

struct GiCascadePlacement {
  glm::ivec3 center{0};
  glm::vec3 interval{0};
  glm::vec3 first_probe{0};
};

enum class GiAnchorSource : uint32_t { None, Explicit, MainCamera, EditorScene };

struct GiAnchor {
  uint64_t camera_id = 0;
  glm::vec3 world_position{0};
  GiAnchorSource source = GiAnchorSource::None;
  bool override_fell_back = false;
};

struct EVOENGINE_API GiProbeFrame {
  uint64_t frame = UINT64_MAX;
  GiProbeSettings settings;
  GiAnchor anchor;
  std::vector<GiCascadePlacement> placements;
  std::string failure;

  bool Update(uint64_t scene_frame, const GiProbeSettings& probes, const GiAnchor& selected_anchor);
};

EVOENGINE_API std::string BuildGiCascadePlacements(const GiProbeSettings& settings, glm::vec3 anchor,
                                                   std::vector<GiCascadePlacement>& placements);
EVOENGINE_API GiProbeSettings GiProbesFromSdfgi(const SdfgiSettings& settings);
EVOENGINE_API SdfgiSettings DeriveSdfgiSettings(const GiProbeSettings& probes, SdfgiSettings settings);
EVOENGINE_API void SerializeGiProbeSettings(YAML::Emitter& out, const GiProbeSettings& settings);
EVOENGINE_API void DeserializeGiProbeSettings(const YAML::Node& in, GiProbeSettings& settings);
}  // namespace evo_engine
