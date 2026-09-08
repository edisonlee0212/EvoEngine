// Godot gi.cpp::process_gi and gi.glsl metadata, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Shared-anchor coordinates preserve multi-camera field ownership.
#include "HddagiGather.hpp"

using namespace evo_engine;

HddagiCameraLayout evo_engine::BuildHddagiCameraLayout(const glm::uvec2 viewport, const bool half_resolution) {
  HddagiCameraLayout layout;
  layout.viewport = glm::max(viewport, glm::uvec2(1));
  layout.pixel_stride = half_resolution ? 2 : 1;
  layout.gi = glm::max(layout.viewport / layout.pixel_stride, glm::uvec2(1));
  layout.reflection_filter_radius = half_resolution ? 6 : 12;
  return layout;
}

HddagiGatherData evo_engine::BuildHddagiGatherData(const GiProbeSettings& probes, const HddagiSettings& settings,
                                                   const std::vector<SdfgiCascade>& cascades, glm::vec3 anchor) {
  if (const auto failure = settings.Validate(probes); !failure.empty())
    throw std::invalid_argument(failure);
  if (cascades.size() != probes.cascade_count)
    throw std::invalid_argument("HDDAGI gather requires every cascade");
  HddagiGatherData data;
  data.grid = (probes.ProbeSize() - 1) * 8;
  data.probe_size = probes.ProbeSize();
  data.cascade_count = probes.cascade_count;
  data.energy = settings.energy;
  data.y_mult = SdfgiYMultiplier(probes.vertical_scale);
  anchor.y *= data.y_mult;
  data.anchor_origin = anchor;
  data.normal_bias = settings.normal_bias;
  data.reflection_bias = settings.reflection_bias;
  data.occlusion_bias = settings.occlusion_bias;
  data.blend_ambient = settings.filter_ambient;
  for (uint32_t c = 0; c < cascades.size(); ++c) {
    const auto& input = cascades[c];
    auto& output = data.cascades.data[c];
    output.offset = glm::vec3(input.position - input.size / 2) * input.cell_size - anchor;
    output.to_cell = 1.0f / input.cell_size;
    output.region_world_offset = (input.position - input.size / 2) / 8;
  }
  return data;
}
