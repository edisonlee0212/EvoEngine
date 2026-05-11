// =============================================================================
//  TasselInstancePacker — CPU graph walker, Phase 1a of the GPU migration.
//
//  Mirror of the internode path in MaizeTassel::RebuildGeometry() so that a
//  later phase can flip the output buffer without any visual change.
// =============================================================================

#include "gpu/TasselInstancePacker.hpp"

#if defined(LSYSTEM_GPU_PIPELINE)

#include "TasselGrowthModel.hpp"

#include <cmath>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

namespace l_system_plugin::gpu {

namespace {

bool IsFiniteVec3(const glm::vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool IsFiniteQuat(const glm::quat& q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

// Deterministic color hash. Must match MaizeTassel.cpp's anonymous-namespace
// HashToColor so the ByNode color mode is pixel-identical between the CPU
// ParticleInfo path and the GPU SoA path. Kept local to this translation unit
// for Phase 1a to avoid a public utility header; Phase 5 will consolidate.
glm::vec4 HashToColor(uint32_t id) {
  const float hue = static_cast<float>((id * 2654435761u) & 1023u) / 1024.0f;
  const float s = 0.72f;
  const float v = 0.92f;
  const float h6 = hue * 6.0f;
  const int sector = static_cast<int>(h6);
  const float f = h6 - static_cast<float>(sector);
  const float p = v * (1.0f - s);
  const float q = v * (1.0f - s * f);
  const float t = v * (1.0f - s * (1.0f - f));

  glm::vec3 rgb(v, t, p);
  switch (sector % 6) {
    case 0: rgb = glm::vec3(v, t, p); break;
    case 1: rgb = glm::vec3(q, v, p); break;
    case 2: rgb = glm::vec3(p, v, t); break;
    case 3: rgb = glm::vec3(p, q, v); break;
    case 4: rgb = glm::vec3(t, p, v); break;
    default: rgb = glm::vec3(v, p, q); break;
  }
  return glm::vec4(rgb, 1.0f);
}

glm::vec4 ResolveColor(ColorMode mode, uint32_t node_index, const glm::vec4& instance_color) {
  switch (mode) {
    case ColorMode::ByNode:
      return HashToColor(node_index);
    case ColorMode::ByInstance:
      return instance_color;
    case ColorMode::ByType:
      return glm::vec4(0.22f, 0.82f, 0.33f, 1.0f);
    case ColorMode::Shaded:
    default:
      return glm::vec4(0.45f, 0.55f, 0.2f, 1.0f);
  }
}

}  // namespace

PackInternodesResult PackTasselInternodes(
    const TasselGraph& graph,
    const PackInternodesOptions& options,
    std::vector<TasselInternodeInstance>& out) {
  PackInternodesResult result{};

  // Matches the CPU path: unit cylinder is authored along local +Y, but node
  // rotations align local +Z to the internode growth direction. Rotate +Y
  // onto -Z before applying the node rotation.
  const glm::quat cylinder_axis_fix =
      glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));

  const auto& sorted = graph.PeekSortedNodeList();
  result.total_nodes_scanned = static_cast<uint32_t>(sorted.size());

  for (const auto handle : sorted) {
    const auto& node = graph.PeekNode(handle);
    if (!node.data.template Is<TasselInternode>()) {
      continue;
    }

    const auto& info = node.info;
    if (info.length <= 0.0f) {
      continue;
    }
    if (!IsFiniteVec3(info.global_position) ||
        !std::isfinite(info.length) ||
        !std::isfinite(info.thickness)) {
      result.invalid_instance_count++;
      continue;
    }
    const float half_thick = info.thickness * 0.5f;
    if (half_thick <= 0.0f) {
      continue;
    }

    glm::quat instance_rotation =
        glm::normalize(info.global_rotation * cylinder_axis_fix);
    if (!IsFiniteQuat(instance_rotation)) {
      instance_rotation = glm::quat(1, 0, 0, 0);
      result.invalid_instance_count++;
    }

    const glm::vec4 color = ResolveColor(
        options.color_mode,
        static_cast<uint32_t>(node.GetIndex()),
        options.instance_color);

    TasselInternodeInstance inst{};
    inst.pos_length = glm::vec4(info.global_position, info.length);
    inst.rot = glm::vec4(instance_rotation.x, instance_rotation.y,
                         instance_rotation.z, instance_rotation.w);
    inst.color_thick = glm::vec4(color.r, color.g, color.b, half_thick);
    out.push_back(inst);
    result.internode_count++;
  }

  // Pair internodes (the short connector between proximal and distal
  // ellipsoids of a spikelet pair) are intentionally not packed here in
  // Phase 1a. They will be hoisted from MaizeTassel.cpp's
  // CollectSpikeletPairsFromGraph helper into a shared utility in Phase 1b
  // and appended here without additional buffer plumbing.
  (void)options.include_pair_internodes;

  return result;
}

}  // namespace l_system_plugin::gpu

#endif  // LSYSTEM_GPU_PIPELINE
