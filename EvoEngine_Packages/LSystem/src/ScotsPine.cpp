#include "ScotsPine.hpp"
#include "LSystemInspectionAdapters.hpp"
#include "LSystemRenderSnapshot.hpp"
#include "LSystemSerializationAdapters.hpp"
#include "ScotsPineDescriptor.hpp"
#include "ScotsPineModules.hpp"
#include "ScotsPineTemporalGrowth.hpp"

// Phase 1 biologically-emergent organ geometry primitives.
#include "ElasticaSolver.hpp"
#include "GrowthField.hpp"
#include "MaterialProfile.hpp"
#include "OrganCenterline.hpp"

#include <AssetManager.hpp>
#include <EditorLayer.hpp>
#include <Material.hpp>
#include <Mesh.hpp>
#include <MeshRenderer.hpp>
#include <Particles.hpp>
#include <Scene.hpp>
#include <Strands.hpp>
#include <StrandsRenderer.hpp>
#include <Times.hpp>
#include <Transform.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <glm/gtx/quaternion.hpp>
#include <iomanip>
#include <limits>
#include <mutex>
#include <ostream>
#include <unordered_set>

using namespace l_system_package;
using namespace evo_engine;

namespace {

ScotsPine::ColorMode g_scots_pine_color_mode = ScotsPine::ColorMode::Shaded;

// Visualization-only knobs (see ScotsPine.hpp for contract notes).
// All three default to neutral values that reproduce existing behaviour
// byte-identically when no caller opts in.
std::atomic<float> g_internode_visual_radius_multiplier{1.0f};
std::atomic<bool> g_render_needles_enabled{true};
std::atomic<std::uint64_t> g_scots_pine_snapshot_version{0};
// Leader debug colour stored as four floats; protected by a coarse mutex
// because atomic<glm::vec4> is not portable. Reads happen once per rebuild
// in the same thread that calls into RebuildGeometry; contention is nil.
std::mutex g_leader_internode_debug_color_mutex;
glm::vec4 g_leader_internode_debug_color{0.0f, 0.0f, 0.0f, 0.0f};
std::mutex g_seasonal_color_tint_mutex;
ScotsPine::SeasonalColorTint g_scots_pine_seasonal_color_tint{};

const glm::vec4 kSyntheticLabelStemYear0{1.0f, 0.0f, 0.0f, 1.0f};
const glm::vec4 kSyntheticLabelStemLater{0.0f, 1.0f, 0.0f, 1.0f};
const glm::vec4 kSyntheticLabelNeedleYear0{0.0f, 0.0f, 1.0f, 1.0f};
const glm::vec4 kSyntheticLabelNeedleLaterBase{1.0f, 1.0f, 0.0f, 1.0f};
const glm::vec4 kSyntheticLabelNeedleLaterTip{1.0f, 0.0f, 1.0f, 1.0f};
const glm::vec4 kSyntheticLabelFascicleSheath{0.0f, 1.0f, 1.0f, 1.0f};
constexpr float kStrandProfileAdaxialFlatSemiellipse = 1.0f;

struct ScotsPineRaytraceParityState {
  bool internodes_ready = false;
  bool sheaths_ready = false;
  bool needles_ready = false;
  std::size_t internode_instances = 0;
  std::size_t sheath_instances = 0;
  std::size_t needle_segments = 0;
  std::size_t needle_points = 0;
};

bool IsFiniteQuat(const glm::quat& q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

bool IsFiniteVec3(const glm::vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool IsFiniteVec2(const glm::vec2& v) {
  return std::isfinite(v.x) && std::isfinite(v.y);
}

bool IsFiniteVec4(const glm::vec4& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) && std::isfinite(v.w);
}

SnapshotInstance ToSnapshotInstance(const ParticleInfo& particle) {
  SnapshotInstance snapshot{};
  snapshot.transform = particle.instance_matrix.value;
  snapshot.color = particle.instance_color;
  return snapshot;
}

ParticleInfo ToParticleInfo(const SnapshotInstance& snapshot) {
  ParticleInfo particle{};
  particle.instance_matrix.value = snapshot.transform;
  particle.instance_color = snapshot.color;
  return particle;
}

SnapshotStrandPoint ToSnapshotStrandPoint(const StrandPoint& strand_point) {
  SnapshotStrandPoint snapshot{};
  snapshot.position = strand_point.position;
  snapshot.thickness = strand_point.thickness;
  snapshot.normal = strand_point.normal;
  snapshot.tex_coord = strand_point.tex_coord;
  snapshot.color = strand_point.color;
  snapshot.material_properties = strand_point.material_properties;
  return snapshot;
}

StrandPoint ToStrandPoint(const SnapshotStrandPoint& snapshot) {
  StrandPoint strand_point{};
  strand_point.position = snapshot.position;
  strand_point.thickness = snapshot.thickness;
  strand_point.normal = snapshot.normal;
  strand_point.tex_coord = snapshot.tex_coord;
  strand_point.color = snapshot.color;
  strand_point.material_properties = snapshot.material_properties;
  return strand_point;
}

void BuildInstancePayloadFromSnapshot(const std::vector<InstancePrimitiveSnapshot>& snapshots,
                                      std::vector<ParticleInfo>& out_instances) {
  out_instances.clear();
  out_instances.reserve(snapshots.size());
  for (const auto& primitive : snapshots) {
    out_instances.emplace_back(ToParticleInfo(primitive.instance));
  }
}

void BuildStrandsPayloadFromSnapshot(const std::vector<StrandsPrimitiveSnapshot>& snapshots,
                                     StrandPointAttributes& out_attributes, std::vector<glm::uint>& out_segments,
                                     std::vector<StrandPoint>& out_points) {
  out_attributes = StrandPointAttributes{};
  out_segments.clear();
  out_points.clear();
  for (const auto& primitive : snapshots) {
    out_attributes.normal = out_attributes.normal || primitive.attributes.normal;
    out_attributes.tex_coord = out_attributes.tex_coord || primitive.attributes.tex_coord;
    out_attributes.color = out_attributes.color || primitive.attributes.color;

    if (primitive.points.empty() || primitive.segments.empty()) {
      continue;
    }

    const auto point_offset = static_cast<glm::uint>(out_points.size());
    out_points.reserve(out_points.size() + primitive.points.size());
    for (const auto& snapshot_point : primitive.points) {
      out_points.emplace_back(ToStrandPoint(snapshot_point));
    }

    out_segments.reserve(out_segments.size() + primitive.segments.size());
    for (const auto segment_start : primitive.segments) {
      if (segment_start >= primitive.points.size()) {
        continue;
      }
      out_segments.emplace_back(point_offset + segment_start);
    }
  }
}

ScotsPineRaytraceParityState EvaluateScotsPineRaytraceParity(const std::shared_ptr<Scene>& scene,
                                                              const PlantRenderTarget* render_target,
                                                              const std::size_t expected_internode_instances,
                                                              const std::size_t expected_sheath_instances,
                                                              const std::size_t expected_needle_segments,
                                                              const std::size_t expected_needle_points) {
  ScotsPineRaytraceParityState state{};
  state.internodes_ready = expected_internode_instances == 0;
  state.sheaths_ready = expected_sheath_instances == 0;
  state.needles_ready = expected_needle_segments == 0 && expected_needle_points == 0;

  if (!scene || !render_target) {
    return state;
  }

  if (expected_internode_instances > 0) {
    if (const auto it = render_target->GetInstanceChannels().find(ScotsPine::kChannelInternodes);
        it != render_target->GetInstanceChannels().end() && it->second) {
      const Entity entity = it->second->GetEntity();
      if (scene->IsEntityValid(entity) && scene->HasPrivateComponent<Particles>(entity)) {
        if (const auto particles = scene->GetOrSetPrivateComponent<Particles>(entity).lock()) {
          const auto mesh = particles->mesh.Get<Mesh>();
          const auto material = particles->material.Get<Material>();
          const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
          if (particles->IsEnabled() && mesh && material && particle_info_list) {
            state.internode_instances = particle_info_list->PeekParticleInfoList().size();
            state.internodes_ready =
                !mesh->UnsafeGetVertices().empty() && state.internode_instances >= expected_internode_instances;
          }
        }
      }
    }
  }

  if (expected_sheath_instances > 0) {
    if (const auto it = render_target->GetInstanceChannels().find(ScotsPine::kChannelNeedleSheaths);
        it != render_target->GetInstanceChannels().end() && it->second) {
      const Entity entity = it->second->GetEntity();
      if (scene->IsEntityValid(entity) && scene->HasPrivateComponent<Particles>(entity)) {
        if (const auto particles = scene->GetOrSetPrivateComponent<Particles>(entity).lock()) {
          const auto mesh = particles->mesh.Get<Mesh>();
          const auto material = particles->material.Get<Material>();
          const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
          if (particles->IsEnabled() && mesh && material && particle_info_list) {
            state.sheath_instances = particle_info_list->PeekParticleInfoList().size();
            state.sheaths_ready =
                !mesh->UnsafeGetVertices().empty() && state.sheath_instances >= expected_sheath_instances;
          }
        }
      }
    }
  }

  if (expected_needle_segments > 0 && expected_needle_points > 0) {
    state.needles_ready = false;
    if (const auto it = render_target->GetStrandsChannels().find(ScotsPine::kChannelNeedles);
        it != render_target->GetStrandsChannels().end() && it->second) {
      const Entity entity = it->second->GetEntity();
      if (scene->IsEntityValid(entity) && scene->HasPrivateComponent<StrandsRenderer>(entity)) {
        if (const auto strands_renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(entity).lock()) {
          const auto strands = strands_renderer->strands.Get<Strands>();
          const auto material = strands_renderer->material.Get<Material>();
          if (strands_renderer->IsEnabled() && strands && material) {
            state.needle_segments = strands->UnsafeGetSegments().size();
            state.needle_points = strands->UnsafeGetStrandPoints().size();
            state.needles_ready =
                state.needle_segments >= expected_needle_segments && state.needle_points >= expected_needle_points;
          }
        }
      }
    }
  }

  return state;
}

glm::vec4 SanitizeFiniteColor(const glm::vec4& value, const glm::vec4& fallback) {
  if (!IsFiniteVec4(value))
    return fallback;
  return glm::clamp(value, glm::vec4(0.0f), glm::vec4(1.0f));
}

glm::vec4 NeedleSemiellipseProfileProperties(const float width_radius_m, const float thickness_radius_m) {
  return glm::vec4(std::max(0.0f, width_radius_m), std::max(0.0f, thickness_radius_m),
                   kStrandProfileAdaxialFlatSemiellipse, 0.0f);
}

ScotsPine::SeasonalColorTint SanitizeSeasonalColorTint(ScotsPine::SeasonalColorTint tint) {
  tint.color = SanitizeFiniteColor(tint.color, glm::vec4(1.0f));
  tint.strength = std::isfinite(tint.strength) ? std::clamp(tint.strength, 0.0f, 1.0f) : 0.0f;
  tint.enabled = tint.enabled && tint.strength > 1.0e-6f;
  return tint;
}

glm::vec4 ApplySeasonalColorTint(const glm::vec4& color, const ScotsPine::SeasonalColorTint& tint) {
  if (!tint.enabled) {
    return color;
  }
  const glm::vec3 tinted = glm::mix(glm::vec3(color), glm::vec3(tint.color), tint.strength);
  return glm::vec4(glm::clamp(tinted, glm::vec3(0.0f), glm::vec3(1.0f)), color.a);
}

bool IsFiniteMat4(const glm::mat4& m) {
  for (int c = 0; c < 4; c++) {
    for (int r = 0; r < 4; r++) {
      if (!std::isfinite(m[c][r]))
        return false;
    }
  }
  return true;
}

glm::vec4 HashToColor(const uint32_t id) {
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
    case 0:
      rgb = glm::vec3(v, t, p);
      break;
    case 1:
      rgb = glm::vec3(q, v, p);
      break;
    case 2:
      rgb = glm::vec3(p, v, t);
      break;
    case 3:
      rgb = glm::vec3(p, q, v);
      break;
    case 4:
      rgb = glm::vec3(t, p, v);
      break;
    default:
      rgb = glm::vec3(v, p, q);
      break;
  }
  return glm::vec4(rgb, 1.0f);
}

uint32_t MixBits(const uint32_t value) {
  uint32_t x = value;
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

float HashToUnitOpen01(const uint32_t hash) {
  constexpr float kDenominator = 16777217.0f;
  const float u = static_cast<float>(hash & 0x00ffffffu) / kDenominator;
  return std::clamp(u + (1.0f / kDenominator), 1.0e-6f, 1.0f - 1.0e-6f);
}

float DeterministicNormalFromNodeRandom(const float node_random, const uint32_t salt) {
  constexpr float kTwoPi = 6.28318530717958647692f;
  const float clamped = std::clamp(node_random, 0.0f, 1.0f);
  const uint32_t base = static_cast<uint32_t>(std::round(clamped * 16777215.0f));
  const uint32_t h1 = MixBits(base ^ salt ^ 0x9e3779b9u);
  const uint32_t h2 = MixBits(base ^ salt ^ 0x85ebca6bu);
  const float u1 = HashToUnitOpen01(h1);
  const float u2 = HashToUnitOpen01(h2);
  const float radius = std::sqrt(-2.0f * std::log(u1));
  return radius * std::cos(kTwoPi * u2);
}

float DeterministicSignedUnitFromNodeRandom(const float node_random, const uint32_t salt) {
  const float clamped = std::clamp(node_random, 0.0f, 1.0f);
  const uint32_t base = static_cast<uint32_t>(std::round(clamped * 16777215.0f));
  return HashToUnitOpen01(MixBits(base ^ salt)) * 2.0f - 1.0f;
}

glm::vec4 ApplyMicroColorVariation(const glm::vec4& color, const float node_random, const uint32_t salt,
                                   const float strength) {
  const float amount = std::clamp(strength, 0.0f, 0.25f);
  if (amount <= 1.0e-6f) {
    return color;
  }

  const glm::vec3 rgb = glm::clamp(glm::vec3(color), glm::vec3(0.0f), glm::vec3(1.0f));
  const float value_delta =
      std::clamp(DeterministicNormalFromNodeRandom(node_random, salt ^ 0x6d2b79f5u) * amount, -0.07f, 0.07f);
  const float sat_delta = std::clamp(DeterministicNormalFromNodeRandom(node_random, salt ^ 0x1b873593u) * amount * 0.6f,
                                     -0.04f, 0.04f);
  const float red_delta = DeterministicSignedUnitFromNodeRandom(node_random, salt ^ 0xa24baed5u) * amount * 0.25f;
  const float green_delta = DeterministicSignedUnitFromNodeRandom(node_random, salt ^ 0x9fb21c65u) * amount * 0.25f;
  const float blue_delta = DeterministicSignedUnitFromNodeRandom(node_random, salt ^ 0xc2b2ae35u) * amount * 0.25f;

  const float luma = glm::dot(rgb, glm::vec3(0.2126f, 0.7152f, 0.0722f));
  glm::vec3 varied = glm::vec3(luma) + (rgb - glm::vec3(luma)) * (1.0f + sat_delta);
  varied *= 1.0f + value_delta;
  varied *= glm::vec3(1.0f + red_delta, 1.0f + green_delta, 1.0f + blue_delta);
  return glm::vec4(glm::clamp(varied, glm::vec3(0.0f), glm::vec3(1.0f)), color.a);
}

struct BiologicalMaterialControls {
  float strength = 0.0f;
  float chlorophyll_scale = 1.0f;
  float carotenoid_gold_scale = 1.0f;
  float lignin_bark_scale = 1.0f;
  float senescence_bias = 0.0f;
  float cuticle_wax = 0.0f;
  float individual_variation = 0.0f;
  float facet_contrast = 0.0f;
  float tip_darkening_strength = 0.0f;
  float stem_age_browning_scale = 1.0f;
};

float BiologicalStrength(const BiologicalMaterialControls& controls) {
  return std::clamp(controls.strength, 0.0f, 1.0f);
}

float Smooth01(const float value) {
  const float x = std::clamp(value, 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}

glm::vec4 ClampColor01(const glm::vec4& color) {
  glm::vec4 out = glm::clamp(color, glm::vec4(0.0f), glm::vec4(1.0f));
  out.a = 1.0f;
  return out;
}

glm::vec4 ApplyLiveNeedlePigments(const glm::vec4& color, const BiologicalMaterialControls& controls) {
  const float strength = BiologicalStrength(controls);
  if (strength <= 1.0e-6f) {
    return color;
  }
  glm::vec3 rgb = glm::clamp(glm::vec3(color), glm::vec3(0.0f), glm::vec3(1.0f));
  const float chlorophyll = glm::mix(1.0f, std::clamp(controls.chlorophyll_scale, 0.2f, 2.0f), strength);
  rgb.g *= chlorophyll;
  if (chlorophyll > 1.0f) {
    const float green_push = std::clamp(chlorophyll - 1.0f, 0.0f, 1.0f);
    rgb.r *= 1.0f - 0.10f * green_push;
    rgb.b *= 1.0f - 0.06f * green_push;
  }

  const float wax = strength * std::clamp(controls.cuticle_wax, 0.0f, 1.0f);
  if (wax > 1.0e-6f) {
    const float luma = glm::dot(rgb, glm::vec3(0.2126f, 0.7152f, 0.0722f));
    const glm::vec3 wax_tint(0.58f, 0.66f, 0.58f);
    rgb = glm::mix(rgb, glm::mix(glm::vec3(luma), wax_tint, 0.35f), 0.30f * wax);
    rgb += glm::vec3(0.025f, 0.035f, 0.025f) * wax;
  }
  return ClampColor01(glm::vec4(rgb, color.a));
}

glm::vec4 ApplyOldNeedlePigments(const glm::vec4& color, const BiologicalMaterialControls& controls) {
  const float strength = BiologicalStrength(controls);
  if (strength <= 1.0e-6f) {
    return color;
  }
  glm::vec3 rgb = glm::clamp(glm::vec3(color), glm::vec3(0.0f), glm::vec3(1.0f));
  const float gold = strength * std::clamp(controls.carotenoid_gold_scale, 0.0f, 2.0f);
  const float dryness = strength * std::clamp(std::max(0.0f, controls.senescence_bias), 0.0f, 0.5f) * 2.0f;
  rgb = glm::mix(rgb, glm::vec3(0.78f, 0.56f, 0.16f), std::clamp(0.16f * gold, 0.0f, 0.42f));
  rgb = glm::mix(rgb, glm::vec3(0.40f, 0.24f, 0.10f), std::clamp(0.22f * dryness, 0.0f, 0.42f));
  return ClampColor01(glm::vec4(rgb, color.a));
}

glm::vec4 ApplyStemPigments(const glm::vec4& color, const BiologicalMaterialControls& controls,
                            const float age_weight) {
  const float strength = BiologicalStrength(controls);
  if (strength <= 1.0e-6f) {
    return color;
  }
  const float lignin = strength * std::clamp(controls.lignin_bark_scale, 0.0f, 2.0f);
  const glm::vec3 bark_brown(0.44f, 0.28f, 0.14f);
  const glm::vec3 olive_brown(0.38f, 0.34f, 0.18f);
  glm::vec3 rgb = glm::vec3(color);
  rgb = glm::mix(rgb, olive_brown, std::clamp(0.08f * lignin, 0.0f, 0.22f));
  rgb = glm::mix(rgb, bark_brown, std::clamp(age_weight * 0.18f * lignin, 0.0f, 0.45f));
  return ClampColor01(glm::vec4(rgb, color.a));
}

void ConfigureMaterialModelCurveDefaults(evo_engine::Plot2D<float>& plot, const float y0, const float y1) {
  plot.min_value = 0.0f;
  plot.max_value = 1.0f;
  plot.curve.SetTangent(false);
  auto& values = plot.curve.UnsafeGetValues();
  values.clear();
  const float start = std::clamp(y0, 0.0f, 1.0f);
  const float end = std::clamp(y1, 0.0f, 1.0f);
  for (int i = 0; i < 5; ++i) {
    const float x = static_cast<float>(i) / 4.0f;
    values.emplace_back(x, glm::mix(start, end, x));
  }
}

float EvaluatePositionalMultiplier(const evo_engine::PlottedDistribution<float>& distribution, const float s_norm,
                                   const float node_random, const uint32_t salt) {
  const float x = std::clamp(s_norm, 0.0f, 1.0f);
  const float mean_value = distribution.mean.GetValue(x);
  const float deviation_value = std::max(0.0f, distribution.deviation.GetValue(x));
  if (!(deviation_value > 0.0f)) {
    return std::max(0.0f, mean_value);
  }
  const float z = DeterministicNormalFromNodeRandom(node_random, salt);
  return std::max(0.0f, mean_value + deviation_value * z);
}

constexpr float kNeedleCrossSectionTemporalWindowYears = 2.0f;

float EvaluateTemporalCrossSectionMultiplier(const evo_engine::PlottedDistribution<float>& distribution,
                                             const float needle_age_years, const float node_random,
                                             const uint32_t salt) {
  // Curve x-domain is [0,1]; map 0..2 years of needle age onto that axis.
  const float age_norm =
      std::clamp(needle_age_years / std::max(1.0e-6f, kNeedleCrossSectionTemporalWindowYears), 0.0f, 1.0f);
  return EvaluatePositionalMultiplier(distribution, age_norm, node_random, salt);
}

bool IsInternodeNode(const PineNode& node) {
  return node.data.template Is<PineInternode>();
}

bool IsSheathNode(const PineNode& node) {
  return node.data.template Is<PineNeedleSheath>();
}

LNodeHandle FindParentInternodeNodeHandle(const PineGraph& graph, const LNodeHandle node_handle) {
  auto parent_handle = graph.PeekNode(node_handle).GetParentHandle();
  while (parent_handle >= 0) {
    const auto& parent = graph.PeekNode(parent_handle);
    if (IsInternodeNode(parent))
      return parent_handle;
    parent_handle = parent.GetParentHandle();
  }
  return -1;
}

LNodeHandle FindParentSheathNodeHandle(const PineGraph& graph, const LNodeHandle node_handle) {
  auto parent_handle = graph.PeekNode(node_handle).GetParentHandle();
  while (parent_handle >= 0) {
    const auto& parent = graph.PeekNode(parent_handle);
    if (IsSheathNode(parent))
      return parent_handle;
    parent_handle = parent.GetParentHandle();
  }
  return -1;
}

std::unordered_set<LFlowHandle> CollectInternodeFlowHandles(const PineGraph& graph) {
  std::unordered_set<LFlowHandle> retained;
  for (const auto flow_handle : graph.PeekSortedFlowList()) {
    const auto& flow = graph.PeekFlow(flow_handle);
    const auto& node_handles = flow.PeekNodeHandles();
    if (node_handles.empty())
      continue;
    bool has_internode = false;
    for (const auto h : node_handles) {
      if (IsInternodeNode(graph.PeekNode(h))) {
        has_internode = true;
        break;
      }
    }
    if (has_internode)
      retained.emplace(flow_handle);
  }
  return retained;
}

LFlowHandle FindParentInternodeFlowHandle(const PineGraph& graph, const LFlowHandle flow_handle,
                                          const std::unordered_set<LFlowHandle>& retained) {
  auto parent_flow_handle = graph.PeekFlow(flow_handle).GetParentHandle();
  while (parent_flow_handle >= 0) {
    if (retained.find(parent_flow_handle) != retained.end())
      return parent_flow_handle;
    parent_flow_handle = graph.PeekFlow(parent_flow_handle).GetParentHandle();
  }
  return -1;
}

void AppendParticlesToMesh(const std::shared_ptr<Scene>& scene, const Entity& entity, std::vector<Vertex>& out_vertices,
                           std::vector<glm::uvec3>& out_triangles) {
  if (!scene->IsEntityValid(entity) || !scene->HasPrivateComponent<Particles>(entity))
    return;
  const auto particles = scene->GetOrSetPrivateComponent<Particles>(entity).lock();
  if (!particles)
    return;
  const auto mesh = particles->mesh.Get<Mesh>();
  const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  if (!mesh || !particle_info_list)
    return;
  const auto& source_vertices = mesh->UnsafeGetVertices();
  const auto& source_triangles = mesh->UnsafeGetTriangles();
  const auto& instances = particle_info_list->PeekParticleInfoList();
  if (source_vertices.empty() || source_triangles.empty() || instances.empty())
    return;

  const auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(entity);
  for (const auto& instance : instances) {
    const glm::mat4 world_transform = entity_global_transform.value * instance.instance_matrix.value;
    if (!IsFiniteMat4(world_transform))
      continue;
    const glm::mat3 world_3x3(world_transform);
    glm::mat3 normal_transform(1.0f);
    const float det = glm::determinant(world_3x3);
    if (std::isfinite(det) && std::abs(det) > 1e-8f) {
      normal_transform = glm::transpose(glm::inverse(world_3x3));
    }
    const auto vertex_offset = static_cast<uint32_t>(out_vertices.size());
    out_vertices.reserve(out_vertices.size() + source_vertices.size());
    out_triangles.reserve(out_triangles.size() + source_triangles.size());
    for (const auto& sv : source_vertices) {
      Vertex v = sv;
      v.position = glm::vec3(world_transform * glm::vec4(sv.position, 1.0f));
      const glm::vec3 tn = normal_transform * sv.normal;
      if (IsFiniteVec3(tn) && glm::length(tn) > 1e-8f)
        v.normal = glm::normalize(tn);
      const glm::vec3 tt = normal_transform * sv.tangent;
      if (IsFiniteVec3(tt) && glm::length(tt) > 1e-8f)
        v.tangent = glm::normalize(tt);
      v.color = instance.instance_color;
      out_vertices.emplace_back(v);
    }
    for (const auto& st : source_triangles) {
      out_triangles.emplace_back(vertex_offset + st.x, vertex_offset + st.y, vertex_offset + st.z);
    }
  }
}

void AppendMeshRendererToMesh(const std::shared_ptr<Scene>& scene, const Entity& entity,
                              std::vector<Vertex>& out_vertices, std::vector<glm::uvec3>& out_triangles) {
  if (!scene->IsEntityValid(entity) || !scene->HasPrivateComponent<MeshRenderer>(entity))
    return;
  const auto mesh_renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
  if (!mesh_renderer)
    return;
  const auto mesh = mesh_renderer->mesh.Get<Mesh>();
  if (!mesh)
    return;

  const auto& source_vertices = mesh->UnsafeGetVertices();
  const auto& source_triangles = mesh->UnsafeGetTriangles();
  if (source_vertices.empty() || source_triangles.empty())
    return;

  const auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(entity);
  const glm::mat4 world_transform = entity_global_transform.value;
  if (!IsFiniteMat4(world_transform))
    return;

  const glm::mat3 world_3x3(world_transform);
  glm::mat3 normal_transform(1.0f);
  const float det = glm::determinant(world_3x3);
  if (std::isfinite(det) && std::abs(det) > 1e-8f) {
    normal_transform = glm::transpose(glm::inverse(world_3x3));
  }

  const auto vertex_offset = static_cast<uint32_t>(out_vertices.size());
  out_vertices.reserve(out_vertices.size() + source_vertices.size());
  out_triangles.reserve(out_triangles.size() + source_triangles.size());
  for (const auto& sv : source_vertices) {
    Vertex v = sv;
    v.position = glm::vec3(world_transform * glm::vec4(sv.position, 1.0f));
    const glm::vec3 tn = normal_transform * sv.normal;
    if (IsFiniteVec3(tn) && glm::length(tn) > 1e-8f)
      v.normal = glm::normalize(tn);
    const glm::vec3 tt = normal_transform * sv.tangent;
    if (IsFiniteVec3(tt) && glm::length(tt) > 1e-8f)
      v.tangent = glm::normalize(tt);
    out_vertices.emplace_back(v);
  }
  for (const auto& st : source_triangles) {
    out_triangles.emplace_back(vertex_offset + st.x, vertex_offset + st.y, vertex_offset + st.z);
  }
}

void AppendStrandsRendererToMesh(const std::shared_ptr<Scene>& scene, const Entity& entity,
                                 std::vector<Vertex>& out_vertices, std::vector<glm::uvec3>& out_triangles) {
  if (!scene->IsEntityValid(entity) || !scene->HasPrivateComponent<StrandsRenderer>(entity)) {
    return;
  }
  const auto strands_renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(entity).lock();
  if (!strands_renderer) {
    return;
  }
  const auto strands = strands_renderer->strands.Get<Strands>();
  if (!strands) {
    return;
  }

  const auto& points = strands->UnsafeGetStrandPoints();
  const auto& segments = strands->UnsafeGetSegments();
  if (points.empty() || segments.empty()) {
    return;
  }

  const auto entity_global_transform = scene->GetDataComponent<GlobalTransform>(entity);
  const glm::mat4 world_transform = entity_global_transform.value;
  if (!IsFiniteMat4(world_transform)) {
    return;
  }

  const glm::mat3 world_3x3(world_transform);
  glm::mat3 normal_transform(1.0f);
  const float det = glm::determinant(world_3x3);
  if (std::isfinite(det) && std::abs(det) > 1e-8f) {
    normal_transform = glm::transpose(glm::inverse(world_3x3));
  }

  constexpr int kSegmentSubdivisions = 4;
  constexpr int kRingVertices = 6;
  constexpr float kMinRadius = 0.00002f;

  for (const auto segment_start : segments) {
    const uint32_t i0 = segment_start;
    const uint32_t i1 = segment_start + 1;
    const uint32_t i2 = segment_start + 2;
    const uint32_t i3 = segment_start + 3;
    if (i3 >= points.size()) {
      continue;
    }

    const auto& p0 = points[i0];
    const auto& p1 = points[i1];
    const auto& p2 = points[i2];
    const auto& p3 = points[i3];

    uint32_t previous_ring_start = std::numeric_limits<uint32_t>::max();

    for (int subdivision = 0; subdivision <= kSegmentSubdivisions; ++subdivision) {
      const float u = static_cast<float>(subdivision) / static_cast<float>(kSegmentSubdivisions);

      glm::vec3 local_pos(0.0f);
      glm::vec3 local_tangent(0.0f);
      Strands::CubicInterpolation(p0.position, p1.position, p2.position, p3.position, local_pos, local_tangent, u);
      if (!IsFiniteVec3(local_pos)) {
        continue;
      }
      if (!IsFiniteVec3(local_tangent) || glm::length(local_tangent) <= 1e-8f) {
        local_tangent = p3.position - p0.position;
      }
      if (!IsFiniteVec3(local_tangent) || glm::length(local_tangent) <= 1e-8f) {
        local_tangent = glm::vec3(0.0f, 1.0f, 0.0f);
      }
      local_tangent = glm::normalize(local_tangent);

      glm::vec4 color = Strands::CubicInterpolation(p0.color, p1.color, p2.color, p3.color, u);
      color = SanitizeFiniteColor(color, glm::vec4(1.0f));

      const float thickness = Strands::CubicInterpolation(p0.thickness, p1.thickness, p2.thickness, p3.thickness, u);
      const float radius = std::max(kMinRadius, std::abs(thickness));

      glm::vec3 world_center = glm::vec3(world_transform * glm::vec4(local_pos, 1.0f));
      glm::vec3 world_tangent = normal_transform * local_tangent;
      if (!IsFiniteVec3(world_tangent) || glm::length(world_tangent) <= 1e-8f) {
        world_tangent = glm::vec3(0.0f, 1.0f, 0.0f);
      }
      world_tangent = glm::normalize(world_tangent);

      glm::vec3 ref(0.0f, 1.0f, 0.0f);
      if (std::abs(glm::dot(ref, world_tangent)) > 0.95f) {
        ref = glm::vec3(1.0f, 0.0f, 0.0f);
      }
      glm::vec3 ring_x = glm::cross(ref, world_tangent);
      if (!IsFiniteVec3(ring_x) || glm::length(ring_x) <= 1e-8f) {
        ring_x = glm::vec3(1.0f, 0.0f, 0.0f);
      }
      ring_x = glm::normalize(ring_x);
      glm::vec3 ring_y = glm::normalize(glm::cross(world_tangent, ring_x));

      const uint32_t ring_start = static_cast<uint32_t>(out_vertices.size());
      for (int ring = 0; ring < kRingVertices; ++ring) {
        const float angle = glm::two_pi<float>() * static_cast<float>(ring) / static_cast<float>(kRingVertices);
        const glm::vec3 normal = glm::normalize(std::cos(angle) * ring_x + std::sin(angle) * ring_y);

        Vertex vertex;
        vertex.position = world_center + normal * radius;
        vertex.normal = normal;
        vertex.tangent = glm::normalize(glm::cross(normal, world_tangent));
        vertex.color = color;
        vertex.tex_coord = glm::vec2(static_cast<float>(ring) / static_cast<float>(kRingVertices), u);
        out_vertices.emplace_back(vertex);
      }

      if (previous_ring_start != std::numeric_limits<uint32_t>::max()) {
        for (int ring = 0; ring < kRingVertices; ++ring) {
          const uint32_t curr0 = ring_start + static_cast<uint32_t>(ring);
          const uint32_t curr1 = ring_start + static_cast<uint32_t>((ring + 1) % kRingVertices);
          const uint32_t prev0 = previous_ring_start + static_cast<uint32_t>(ring);
          const uint32_t prev1 = previous_ring_start + static_cast<uint32_t>((ring + 1) % kRingVertices);
          out_triangles.emplace_back(prev0, prev1, curr0);
          out_triangles.emplace_back(curr0, prev1, curr1);
        }
      }

      previous_ring_start = ring_start;
    }
  }
}

// Unit cylinder: radius=1, height=1, along +Y, base at y=0.
void GenerateUnitCylinderMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices,
                              const glm::vec4& bark_color, int segments = 6) {
  vertices.clear();
  indices.clear();
  const float angle_step = glm::two_pi<float>() / static_cast<float>(segments);
  for (int ring = 0; ring <= 1; ring++) {
    const float y = static_cast<float>(ring);
    for (int s = 0; s < segments; s++) {
      const float angle = angle_step * static_cast<float>(s);
      const float cx = std::cos(angle);
      const float cz = std::sin(angle);
      Vertex v;
      v.position = glm::vec3(cx, y, cz);
      v.normal = glm::normalize(glm::vec3(cx, 0.0f, cz));
      v.color = bark_color;
      v.tex_coord = glm::vec2(static_cast<float>(s) / static_cast<float>(segments), y);
      vertices.push_back(v);
    }
  }
  for (int s = 0; s < segments; s++) {
    const unsigned int s0 = static_cast<unsigned int>(s);
    const unsigned int s1 = static_cast<unsigned int>((s + 1) % segments);
    const unsigned int e0 = s0 + static_cast<unsigned int>(segments);
    const unsigned int e1 = s1 + static_cast<unsigned int>(segments);
    indices.push_back(s0);
    indices.push_back(e0);
    indices.push_back(s1);
    indices.push_back(s1);
    indices.push_back(e0);
    indices.push_back(e1);
  }
}

// ---------------------------------------------------------------------------
// Build strands payload for every alive needle in the tree. All positions are
// emitted in world space so the channel owner can keep identity transforms.
// ---------------------------------------------------------------------------
struct NeedleAnchor {
  glm::vec3 base_position;       ///< World-space attachment point.
  glm::quat orientation;         ///< Rotates +Z (needle local forward) to outward direction.
  glm::vec3 base_adaxial_world;  ///< World-space adaxial reference at the base.
};

struct FascicleSheathFrame {
  glm::vec3 base_position = glm::vec3(0.0f);
  glm::vec3 direction = glm::vec3(0.0f, 1.0f, 0.0f);
  glm::vec3 radial = glm::vec3(1.0f, 0.0f, 0.0f);
  glm::vec3 parent_dir = glm::vec3(0.0f, 1.0f, 0.0f);
  glm::quat orientation = glm::quat(1, 0, 0, 0);  ///< Maps local -Z to sheath direction.
};

float ComputeActiveSheathBranchingAngleDeg(const PineGraph& graph, const PineNeedleSheath& sheath) {
  const float sheath_age_years = std::max(0.0f, graph.data.clock.NowYears() - sheath.continuous_growth.t_init_years);
  const float relax_years = std::max(0.0f, sheath.branching_relax_years);
  const float relax_progress =
      (relax_years <= 1.0e-5f) ? 1.0f : std::clamp(sheath_age_years / relax_years, 0.0f, 1.0f);
  return std::clamp(sheath.branching_angle_deg, 0.0f, 179.5f) * relax_progress;
}

FascicleSheathFrame ComputeFascicleSheathFrame(const PineNeedleSheath& sheath,
                                               const PineNode& parent_internode_node,
                                               const float branching_angle_deg) {
  // Parent internode tangent and rolled radial basis in world space.
  // Using the internode's world rotation preserves phyllotactic roll.
  glm::vec3 parent_dir = parent_internode_node.info.GetGlobalDirection();
  if (!IsFiniteVec3(parent_dir) || glm::length(parent_dir) <= 1e-8f) {
    parent_dir = glm::vec3(0.0f, 0.0f, -1.0f);
  } else {
    parent_dir = glm::normalize(parent_dir);
  }

  glm::vec3 perp = parent_internode_node.info.global_rotation * glm::vec3(1.0f, 0.0f, 0.0f);
  if (!IsFiniteVec3(perp) || glm::length(perp) <= 1e-8f) {
    // Fallback keeps anchors valid even if an upstream rotation is degenerate.
    glm::vec3 ref(0.0f, 1.0f, 0.0f);
    if (std::abs(glm::dot(ref, parent_dir)) > 0.95f)
      ref = glm::vec3(1.0f, 0.0f, 0.0f);
    perp = ref - parent_dir * glm::dot(ref, parent_dir);
  }
  const float plen = glm::length(perp);
  perp = (plen > 1e-8f) ? (perp / plen) : glm::vec3(1.0f, 0.0f, 0.0f);

  // Apply per-cluster azimuthal phyllotactic roll around the parent axis so
  // sibling clusters on the same shoot fan out by ~137.5 degrees.
  const glm::quat roll_q = glm::angleAxis(glm::radians(sheath.roll_offset_deg), parent_dir);
  perp = glm::normalize(roll_q * perp);

  const float branching_angle_rad = glm::radians(std::clamp(branching_angle_deg, 0.0f, 179.5f));
  glm::vec3 sheath_dir =
      glm::normalize(std::cos(branching_angle_rad) * parent_dir + std::sin(branching_angle_rad) * perp);
  if (!IsFiniteVec3(sheath_dir) || glm::length(sheath_dir) <= 1.0e-8f) {
    sheath_dir = parent_dir;
  }

  const float s_clamped = std::clamp(sheath.s_along_parent_norm, 0.0f, 1.0f);
  const float parent_length = std::max(0.0f, parent_internode_node.info.length);
  const glm::vec3 anchor_pos = parent_internode_node.info.global_position + parent_dir * (s_clamped * parent_length);

  glm::vec3 z_axis = -sheath_dir;
  glm::vec3 x_axis = perp - z_axis * glm::dot(perp, z_axis);
  if (!IsFiniteVec3(x_axis) || glm::length(x_axis) <= 1.0e-8f) {
    glm::vec3 ref(0.0f, 1.0f, 0.0f);
    if (std::abs(glm::dot(ref, z_axis)) > 0.95f)
      ref = glm::vec3(1.0f, 0.0f, 0.0f);
    x_axis = ref - z_axis * glm::dot(ref, z_axis);
  }
  x_axis = glm::normalize(x_axis);
  glm::vec3 y_axis = glm::cross(z_axis, x_axis);
  if (!IsFiniteVec3(y_axis) || glm::length(y_axis) <= 1.0e-8f) {
    y_axis = glm::vec3(0.0f, 1.0f, 0.0f);
  } else {
    y_axis = glm::normalize(y_axis);
  }
  x_axis = glm::normalize(glm::cross(y_axis, z_axis));

  FascicleSheathFrame frame;
  frame.base_position = anchor_pos;
  frame.direction = sheath_dir;
  frame.radial = perp;
  frame.parent_dir = parent_dir;
  frame.orientation = glm::normalize(glm::quat_cast(glm::mat3(x_axis, y_axis, z_axis)));
  return frame;
}

inline NeedleAnchor ComputeFascicleNeedleAnchor(const PineNeedleSheath& sheath, const PineNode& parent_internode_node,
                                                float branching_angle_deg, int needle_index_in_cluster,
                                                int needle_count_in_cluster) {
  const FascicleSheathFrame sheath_frame = ComputeFascicleSheathFrame(sheath, parent_internode_node, branching_angle_deg);

  // Upward fan around the parent axis. For the common 2-needle fascicle, keep
  // both needles in a narrow V instead of placing them 180 degrees apart
  // (which frequently sends one needle downward and reads as "spaghetti").
  const float kFanHalfAngleRad = glm::radians(12.0f);
  const float kFanJitterRad = glm::radians(4.0f);
  const float fan_jitter = (sheath.node_random - 0.5f) * 2.0f * kFanJitterRad;
  float fan_t = 0.0f;
  if (needle_count_in_cluster > 1) {
    fan_t = static_cast<float>(needle_index_in_cluster) / static_cast<float>(needle_count_in_cluster - 1);
    fan_t = fan_t * 2.0f - 1.0f;
  }
  const float angle = fan_t * kFanHalfAngleRad + fan_jitter;
  const glm::quat about_axis = glm::angleAxis(angle, sheath_frame.parent_dir);
  const glm::vec3 radial = about_axis * sheath_frame.radial;

  // Branching angle is measured from the apical axis (parent_dir). 0 deg
  // means fully apical; increasing angle opens the fascicle toward radial.
  const float branching_angle_rad = glm::radians(std::clamp(branching_angle_deg, 0.0f, 179.5f));
  const glm::vec3 needle_dir = glm::normalize(std::cos(branching_angle_rad) * sheath_frame.parent_dir +
                                              std::sin(branching_angle_rad) * radial);

  // Build a stable local frame instead of a shortest-arc quaternion so the
  // local x-z bending plane is locked to the stem-facing radial plane.
  // +Z = needle forward, +X = adaxial->abaxial direction (outward from stem),
  // +Y = completes a right-handed basis.
  const glm::vec3 z_axis = needle_dir;
  glm::vec3 x_axis = radial - z_axis * glm::dot(radial, z_axis);
  if (!IsFiniteVec3(x_axis) || glm::length(x_axis) <= 1e-8f) {
    glm::vec3 ref(0.0f, 1.0f, 0.0f);
    if (std::abs(glm::dot(ref, z_axis)) > 0.95f)
      ref = glm::vec3(1.0f, 0.0f, 0.0f);
    x_axis = ref - z_axis * glm::dot(ref, z_axis);
  }
  x_axis = glm::normalize(x_axis);
  glm::vec3 y_axis = glm::cross(z_axis, x_axis);
  if (!IsFiniteVec3(y_axis) || glm::length(y_axis) <= 1e-8f) {
    y_axis = glm::vec3(0.0f, 1.0f, 0.0f);
  } else {
    y_axis = glm::normalize(y_axis);
  }
  x_axis = glm::normalize(glm::cross(y_axis, z_axis));
  const glm::mat3 basis_world_from_local(x_axis, y_axis, z_axis);
  const glm::quat orientation = glm::normalize(glm::quat_cast(basis_world_from_local));

  // Use -X as the adaxial reference direction for the ellipsoid profile.
  const glm::vec3 adaxial_world = glm::normalize(orientation * glm::vec3(-1, 0, 0));

  NeedleAnchor anchor;
  anchor.base_position = sheath_frame.base_position + sheath_frame.direction * std::max(0.0f, sheath.length) * 0.92f;
  anchor.orientation = orientation;
  anchor.base_adaxial_world = adaxial_world;
  return anchor;
}

inline StrandPoint MakeEndpointPaddingControl(const StrandPoint& endpoint, const StrandPoint& neighbor,
                                             const float min_strand_thickness_m) {
  StrandPoint padded = endpoint;
  padded.position = endpoint.position * 2.0f - neighbor.position;
  padded.tex_coord = endpoint.tex_coord * 2.0f - neighbor.tex_coord;
  padded.thickness = std::max(min_strand_thickness_m, endpoint.thickness * 2.0f - neighbor.thickness);
  return padded;
}

inline void AppendEndpointPaddedNeedleControls(const std::vector<StrandPoint>& logical_points,
                                               const float min_strand_thickness_m,
                                               std::vector<glm::uint>& out_segments,
                                               std::vector<StrandPoint>& out_points) {
  if (logical_points.size() < 2) {
    return;
  }

  const uint32_t point_start = static_cast<uint32_t>(out_points.size());
  out_points.emplace_back(MakeEndpointPaddingControl(logical_points.front(), logical_points[1], min_strand_thickness_m));
  out_points.insert(out_points.end(), logical_points.begin(), logical_points.end());
  out_points.emplace_back(
      MakeEndpointPaddingControl(logical_points.back(), logical_points[logical_points.size() - 2], min_strand_thickness_m));

  const uint32_t logical_segment_count = static_cast<uint32_t>(logical_points.size() - 1);
  for (uint32_t si = 0; si < logical_segment_count; ++si) {
    out_segments.emplace_back(point_start + si);
  }
}

/// Build strands payload for every alive needle in the tree.
inline void BuildPineNeedleStrandsPayload(
    const PineGraph& graph, const std::vector<LNodeHandle>& sorted_nodes, const ScotsPine::ColorMode color_mode,
    const glm::vec4& needle_young_color, const glm::vec4& needle_old_color, const glm::vec4& needle_tip_color,
    const glm::vec4& node_sheath_brown_color, const evo_engine::Plot2D<float>& needle_axial_color_curve,
    const evo_engine::Plot2D<float>& needle_y_age_color_curve,
    const float needle_tip_color_mix_start, const float needle_tip_color_exponent,
    const float needle_old_thinning_fraction, const float needle_min_strand_thickness_m,
    const float color_micro_variation, const float needle_axial_age_span, const float needle_axial_age_exponent,
    const float sheath_browning_strength, const float needle_twist_turns, const float needle_edge_darkening,
    const BiologicalMaterialControls& biological_material,
    const PlottedDistribution<float>& needle_cross_section_width_profile,
    const PlottedDistribution<float>& needle_cross_section_thickness_profile,
    const PlottedDistribution<float>& needle_cross_section_temporal_maturity_curve,
    const float needle_lignification_factor_year0, const float needle_stomatal_strip_density_year0,
    const float needle_basal_taper_ratio_year0, const float needle_fascicle_sheath_budget_years,
    const float needle_specularity_plasticity_year0,
    std::vector<glm::uint>& out_segments, std::vector<StrandPoint>& out_points, const int logical_station_count,
    std::vector<ScotsPine::NeedleSkeletonLine>* out_needle_skeleton_lines) {
  out_segments.clear();
  out_points.clear();
  if (out_needle_skeleton_lines) {
    out_needle_skeleton_lines->clear();
  }

  // Logical stations are the user-visible needle samples. The render payload
  // pads two hidden cubic controls per needle so EvoEngine strands interpolate
  // the logical base and tip even when the descriptor requests one segment.
  const int kLogicalStations = std::max(2, logical_station_count);
  const int kLogicalSegments = kLogicalStations - 1;
  const float old_thinning_fraction = std::clamp(needle_old_thinning_fraction, 0.0f, 0.95f);
  const float min_strand_thickness_m = std::clamp(needle_min_strand_thickness_m, 0.000001f, 0.002f);
  const float clamped_sheath_browning_strength = std::clamp(sheath_browning_strength, 0.0f, 1.0f);
  const float clamped_twist_turns = std::clamp(needle_twist_turns, -8.0f, 8.0f);
  const float clamped_edge_darkening = std::clamp(needle_edge_darkening, 0.0f, 0.75f);
  (void)needle_fascicle_sheath_budget_years;
  const float biological_strength = BiologicalStrength(biological_material);
  const float biological_senescence_bias =
      biological_strength * std::clamp(biological_material.senescence_bias, -0.5f, 0.5f);
  const float biological_tip_darkening_strength =
      biological_strength * std::clamp(biological_material.tip_darkening_strength, 0.0f, 1.0f);
  const float biological_facet_contrast =
      biological_strength * std::clamp(biological_material.facet_contrast, 0.0f, 1.0f);
  constexpr float kNeedleDefaultWidthRadiusM = 0.0007f;       // ~0.7 mm half-width.
  constexpr float kNeedleDefaultThicknessRadiusM = 0.00045f;  // ~0.45 mm half-thickness.

  for (const auto handle : sorted_nodes) {
    const auto& node = graph.PeekNode(handle);
    if (!node.data.template Is<PineNeedle>())
      continue;
    const auto& needle = node.data.template Get<PineNeedle>();
    if (!needle.alive)
      continue;

    const LNodeHandle sheath_handle = FindParentSheathNodeHandle(graph, handle);
    if (sheath_handle < 0)
      continue;
    const auto& sheath_node = graph.PeekNode(sheath_handle);
    if (!sheath_node.data.template Is<PineNeedleSheath>())
      continue;
    const auto& sheath = sheath_node.data.template Get<PineNeedleSheath>();

    const LNodeHandle parent_handle = FindParentInternodeNodeHandle(graph, sheath_handle);
    if (parent_handle < 0)
      continue;
    const auto& parent_node = graph.PeekNode(parent_handle);

    const float sen = std::clamp(needle.senescence_phase, 0.0f, 1.0f);
    const float needle_age_years =
        std::max(0.0f, graph.data.clock.NowYears() - needle.continuous_growth.t_init_years);
    const float needle_lifespan_years = std::max(0.25f, static_cast<float>(needle.lifespan_years));
    const float needle_age_norm = std::clamp(needle_age_years / needle_lifespan_years, 0.0f, 1.0f);
    const float sheath_branching_angle_deg = ComputeActiveSheathBranchingAngleDeg(graph, sheath);
    const float clamped_axial_exponent = std::max(0.1f, needle_axial_age_exponent);
    const float length_mult = glm::mix(1.0f, 0.82f, sen);
    const bool year0_cohort = needle.initiation_year_index == 0;
    const float cohort_lignification_factor =
        year0_cohort ? std::clamp(needle_lignification_factor_year0, 0.0f, 2.0f) : 1.0f;
    const float cohort_stomatal_strip_density =
        year0_cohort ? std::clamp(needle_stomatal_strip_density_year0, 0.0f, 1.0f) : 1.0f;
    const float cohort_basal_taper_ratio =
        year0_cohort ? std::clamp(needle_basal_taper_ratio_year0, 0.6f, 1.2f) : 1.0f;
    const float cohort_specularity_plasticity =
        year0_cohort ? std::clamp(needle_specularity_plasticity_year0, 0.0f, 1.0f) : 1.0f;
    // Phase 6 seedling realism pass: drive rendered needle thickness from
    // material profile radii when available, with Phase 1 taper as a fallback.
    const float t_now_years = graph.data.clock.NowYears();
    const float maturation_multiplier =
        (needle.continuous_growth.maturation_years > 0.0f) ? needle.continuous_growth.Multiplier(t_now_years) : 1.0f;

    // Phase 4: world-frame gravity vector (-Y world-up convention).
    const float gravity_mag = graph.data.gravity_m_s2;
    const glm::vec3 gravity_world(0.0f, -gravity_mag, 0.0f);

    {
      const int n = std::max(0, needle.needle_index);
      const int needle_count = std::max(1, needle.needle_count);
      const BilateralGrowthField1D& needle_growth_field = needle.growth_field;
      const MaterialProfile1D& needle_material_profile = needle.material_profile;
      MaterialProfile1D matured_material_profile = needle_material_profile;
      const float needle_wave_amplitude_deg = std::clamp(needle.sinusoidal_amplitude_deg, 0.0f, 45.0f);
      const float needle_wave_frequency_cycles = std::clamp(needle.sinusoidal_frequency_cycles, 0.0f, 12.0f);
      const float needle_wave_phase_rad = needle.sinusoidal_phase_rad;
      // Use chronological age so fascicle opening continues through dormant season.
      const float needle_relax_years = std::max(0.0f, needle.branching_relax_years);
      const float needle_relax_progress =
          (needle_relax_years <= 1e-5f) ? 1.0f : std::clamp(needle_age_years / needle_relax_years, 0.0f, 1.0f);
      const float active_branching_angle_deg = sheath_branching_angle_deg * needle_relax_progress;

      const bool has_profile_radii =
          std::isfinite(matured_material_profile.base_radius_m) &&
          std::isfinite(matured_material_profile.tip_radius_m) &&
          (matured_material_profile.base_radius_m > 0.0f || matured_material_profile.tip_radius_m > 0.0f);
      const float fallback_width_radius_m =
          has_profile_radii ? std::max(matured_material_profile.base_radius_m, 0.0f) : kNeedleDefaultWidthRadiusM;
      const float fallback_thickness_radius_m =
          has_profile_radii ? std::max(matured_material_profile.tip_radius_m, 0.0f) : kNeedleDefaultThicknessRadiusM;
      const float raw_cluster_width_radius_m = (needle.cross_section_width_radius_m > 0.0f)
                                                   ? needle.cross_section_width_radius_m
                                                   : fallback_width_radius_m;
      const float raw_cluster_thickness_radius_m = (needle.cross_section_thickness_radius_m > 0.0f)
                                                       ? needle.cross_section_thickness_radius_m
                                                       : fallback_thickness_radius_m;
      const float clamped_cluster_width_radius_m = std::max(raw_cluster_width_radius_m, min_strand_thickness_m);
      const float clamped_cluster_thickness_radius_m = std::max(raw_cluster_thickness_radius_m, min_strand_thickness_m);
      std::vector<float> width_radius_table;
      std::vector<float> thickness_radius_table;
      width_radius_table.reserve(static_cast<size_t>(kLogicalStations));
      thickness_radius_table.reserve(static_cast<size_t>(kLogicalStations));
      const float radius_vigor_scale = std::clamp(needle.render_radius_scale, 0.10f, 4.0f);
      const uint32_t node_hash = static_cast<uint32_t>(node.GetIndex());
      const uint32_t needle_hash = static_cast<uint32_t>(n);
      const uint32_t width_seed = node_hash ^ (needle_hash * 0x9e3779b9u) ^ 0x2d9c8f13u;
      const uint32_t thickness_seed = node_hash ^ (needle_hash * 0x85ebca6bu) ^ 0xa5b35705u;
      const uint32_t temporal_seed = node_hash ^ (needle_hash * 0xc2b2ae35u) ^ 0x4f1bbcdcu;
      const float temporal_cross_section_multiplier = EvaluateTemporalCrossSectionMultiplier(
          needle_cross_section_temporal_maturity_curve, needle_age_years, needle.node_random, temporal_seed);
      for (int i = 0; i < kLogicalStations; ++i) {
        const float s_norm = static_cast<float>(i) / static_cast<float>(kLogicalStations - 1);
        const float width_profile_multiplier = EvaluatePositionalMultiplier(
            needle_cross_section_width_profile, s_norm, needle.node_random, width_seed ^ static_cast<uint32_t>(i));
        const float thickness_profile_multiplier =
            EvaluatePositionalMultiplier(needle_cross_section_thickness_profile, s_norm, needle.node_random,
                                         thickness_seed ^ static_cast<uint32_t>(i));
        const float raw_width_radius =
            clamped_cluster_width_radius_m * width_profile_multiplier * temporal_cross_section_multiplier;
        const float raw_thickness_radius =
            clamped_cluster_thickness_radius_m * thickness_profile_multiplier * temporal_cross_section_multiplier;
        const float scaled_width_radius = raw_width_radius * radius_vigor_scale;
        const float scaled_thickness_radius = raw_thickness_radius * radius_vigor_scale;
        const float safe_width_radius = std::max(scaled_width_radius, min_strand_thickness_m);
        const float safe_thickness_radius = std::max(scaled_thickness_radius, min_strand_thickness_m);
        const float basal_taper_multiplier = glm::mix(cohort_basal_taper_ratio, 1.0f, s_norm);
        const float tapered_width_radius = std::max(safe_width_radius * basal_taper_multiplier, min_strand_thickness_m);
        const float tapered_thickness_radius =
            std::max(safe_thickness_radius * basal_taper_multiplier, min_strand_thickness_m);
        width_radius_table.push_back(tapered_width_radius);
        thickness_radius_table.push_back(tapered_thickness_radius);
      }

      // Phase 3: bent centerline driven by a per-needle bilateral growth
      // field, ramped by the needle's continuous-growth multiplier.
      const float length = std::max(0.001f, needle.length * length_mult);
      OrganCenterline intrinsic_centerline =
          BuildBentNeedleCenterline(length, /*segments=*/kLogicalSegments, needle_growth_field, maturation_multiplier,
                                    needle_wave_amplitude_deg, needle_wave_frequency_cycles, needle_wave_phase_rad);

      const bool mechanics_active = matured_material_profile.IsActive() && gravity_mag > 0.0f;
      const NeedleAnchor anchor =
          ComputeFascicleNeedleAnchor(sheath, parent_node, active_branching_angle_deg, n, needle_count);

      // Phase 4: per-needle elastica solve. The intrinsic centerline gives
      // kappa_intrinsic(s) and the deflection plane (local x-z); gravity is
      // projected from world into needle-local coordinates and decomposed
      // onto that plane. When the material profile is inert, this whole
      // block is skipped and we fall back to the Phase 3 intrinsic shape.
      OrganCenterline centerline = intrinsic_centerline;
      if (mechanics_active) {
        // Project gravity into the needle's local frame.
        const glm::quat q_inv = glm::conjugate(anchor.orientation);
        const glm::vec3 g_local = q_inv * gravity_world;
        // Use only the in-plane (x, z) components - the cross-plane Y
        // component would induce twist; Phase 4 intentionally restricts
        // bending to the same plane the intrinsic curvature lives in.
        const glm::vec2 g_xz(g_local.x, g_local.z);

        // Sample intrinsic curvature from the un-deflected centerline by
        // numerical differentiation of its tangent angle. (Closed-form
        // would require exposing the field math here; this is cheap.)
        std::vector<float> kappa_table(kLogicalStations, 0.0f);
        {
          const float ds = length / static_cast<float>(kLogicalStations - 1);
          float prev_theta = 0.0f;
          for (int i = 0; i < kLogicalStations; ++i) {
            const float s_i = static_cast<float>(i) * ds;
            const auto sample = intrinsic_centerline.Sample(s_i);
            const float theta_i = std::atan2(sample.tangent.x, sample.tangent.z);
            if (i == 0) {
              kappa_table[0] = 0.0f;
            } else {
              kappa_table[i] = (theta_i - prev_theta) / ds;
            }
            prev_theta = theta_i;
          }
        }

        PlanarElasticaInput esi;
        esi.length_m = length;
        esi.station_count = kLogicalStations;
        esi.intrinsic_curvature_per_m = [&kappa_table](float s_norm) {
          const int N = static_cast<int>(kappa_table.size());
          const float idx_f = std::clamp(s_norm, 0.0f, 1.0f) * static_cast<float>(N - 1);
          const int i0 = std::clamp(static_cast<int>(std::floor(idx_f)), 0, N - 1);
          const int i1 = std::min(i0 + 1, N - 1);
          const float u = idx_f - static_cast<float>(i0);
          return kappa_table[i0] * (1.0f - u) + kappa_table[i1] * u;
        };
        esi.bending_stiffness_Pa_m4 = [&matured_material_profile, t_now_years](float s_norm) {
          return matured_material_profile.BendingStiffness_Pa_m4(s_norm, t_now_years);
        };
        esi.mass_per_length_kg_m = [&matured_material_profile](float s_norm) {
          return matured_material_profile.MassPerLength_kg_m(s_norm);
        };
        esi.gravity_acceleration_xz_m_s2 = g_xz;
        esi.relaxation = 0.6f;
        esi.tolerance_rad = 1e-5f;
        esi.max_iterations = 24;
        const PlanarElasticaOutput eso = SolvePlanarElastica(esi);

        // Convert solver positions back into a centerline (planar, x-z).
        OrganCenterline deflected;
        auto& cps = deflected.MutableControlPoints();
        cps.resize(static_cast<size_t>(kLogicalStations));
        for (int i = 0; i < kLogicalStations; ++i) {
          cps[static_cast<size_t>(i)] = glm::vec3(eso.positions_xz[i].x, 0.0f, eso.positions_xz[i].y);
        }
        deflected.Invalidate();
        centerline = std::move(deflected);
      }

      std::vector<float> station_lignification;
      std::vector<float> station_stripe_proxy;
      std::vector<float> station_stripe_scale;
      std::vector<float> station_sheath_visibility;
      station_lignification.reserve(static_cast<size_t>(kLogicalStations));
      station_stripe_proxy.reserve(static_cast<size_t>(kLogicalStations));
      station_stripe_scale.reserve(static_cast<size_t>(kLogicalStations));
      station_sheath_visibility.reserve(static_cast<size_t>(kLogicalStations));

      const float stripe_strength = cohort_stomatal_strip_density * cohort_specularity_plasticity * 0.08f;
      for (int i = 0; i < kLogicalStations; ++i) {
        const float s_norm = static_cast<float>(i) / static_cast<float>(kLogicalStations - 1);
        const float axial = std::pow(s_norm, clamped_axial_exponent);
        const float axial_shift = (axial - 0.5f) * 2.0f;
        const float segment_age_norm = std::clamp(needle_age_norm + needle_axial_age_span * axial_shift, 0.0f, 1.0f);
        float segment_oldness =
            std::clamp(std::max(segment_age_norm, sen) * cohort_lignification_factor + biological_senescence_bias,
                       0.0f, 1.0f);
        // The basal fascicle sheath is now explicit cylinder geometry, so
        // needle strands no longer fake sheath tissue with a base color blend.
        float sheath_visibility = 0.0f;
        const float stripe_phase = (s_norm * 48.0f + needle.node_random * 17.0f) * glm::two_pi<float>();
        const float stripe_wave = 0.5f + 0.5f * std::sin(stripe_phase);
        const float stripe_scale = 1.0f - stripe_strength + stripe_strength * stripe_wave;
        const float stripe_proxy =
            std::clamp(cohort_stomatal_strip_density * cohort_specularity_plasticity * stripe_wave, 0.0f, 1.0f);

        station_lignification.emplace_back(segment_oldness);
        station_stripe_proxy.emplace_back(stripe_proxy);
        station_stripe_scale.emplace_back(stripe_scale);
        station_sheath_visibility.emplace_back(sheath_visibility);
      }

      std::vector<glm::vec4> station_colors;
      station_colors.reserve(static_cast<size_t>(kLogicalStations));
      if (color_mode == ScotsPine::ColorMode::ByNode) {
        const glm::vec4 node_color = HashToColor(static_cast<uint32_t>(node.GetIndex()));
        station_colors.assign(static_cast<size_t>(kLogicalStations), node_color);
      } else if (color_mode == ScotsPine::ColorMode::ByInstance) {
        // ByInstance is driven by the strands material tint.
        station_colors.assign(static_cast<size_t>(kLogicalStations), glm::vec4(1.0f));
      } else {
        for (int i = 0; i < kLogicalStations; ++i) {
          const float s_norm = static_cast<float>(i) / static_cast<float>(kLogicalStations - 1);
          const float segment_oldness = station_lignification[static_cast<size_t>(i)];
          const float stripe_proxy = station_stripe_proxy[static_cast<size_t>(i)];
          const float stripe_scale = station_stripe_scale[static_cast<size_t>(i)];
          const float sheath_visibility = station_sheath_visibility[static_cast<size_t>(i)];
          glm::vec4 segment_color;
          if (color_mode == ScotsPine::ColorMode::NeedleLignification) {
            const glm::vec3 low(0.05f, 0.28f, 0.08f);
            const glm::vec3 high(0.75f, 0.42f, 0.10f);
            segment_color = glm::vec4(glm::mix(low, high, segment_oldness), 1.0f);
          } else if (color_mode == ScotsPine::ColorMode::NeedleStripeProxy) {
            const glm::vec3 low(0.04f, 0.08f, 0.20f);
            const glm::vec3 high(0.80f, 0.95f, 1.00f);
            segment_color = glm::vec4(glm::mix(low, high, stripe_proxy), 1.0f);
          } else if (color_mode == ScotsPine::ColorMode::NeedleSheath) {
            const glm::vec3 low(0.06f, 0.06f, 0.06f);
            const glm::vec3 high(0.95f, 0.75f, 0.20f);
            segment_color = glm::vec4(glm::mix(low, high, sheath_visibility), 1.0f);
          } else if (color_mode == ScotsPine::ColorMode::SyntheticOrganLabels) {
            if (year0_cohort) {
              segment_color = kSyntheticLabelNeedleYear0;
            } else {
              segment_color = s_norm >= needle_tip_color_mix_start ? kSyntheticLabelNeedleLaterTip
                                                                   : kSyntheticLabelNeedleLaterBase;
            }
          } else {
            const float axial_weight = std::clamp(needle_axial_color_curve.GetValue(s_norm), 0.0f, 1.0f);
            const float age_weight = std::clamp(needle_y_age_color_curve.GetValue(needle_age_norm), 0.0f, 1.0f);
            const float live_older_weight =
                year0_cohort ? age_weight * 0.20f : std::max(axial_weight, age_weight * 0.65f);
            const float dry_weight = std::clamp(std::max(segment_oldness, sen) +
                                                    std::max(0.0f, biological_senescence_bias) * 0.45f,
                                                0.0f, 1.0f);
            segment_color = glm::mix(needle_young_color, needle_tip_color, std::clamp(live_older_weight, 0.0f, 1.0f));
            segment_color = glm::mix(segment_color, needle_old_color, dry_weight);
            segment_color =
                glm::mix(segment_color, node_sheath_brown_color, sheath_visibility * clamped_sheath_browning_strength);
            if (biological_strength > 1.0e-6f) {
              if (biological_tip_darkening_strength > 1.0e-6f) {
                const float tip_u = (s_norm - needle_tip_color_mix_start) /
                                    std::max(1.0e-5f, 1.0f - needle_tip_color_mix_start);
                const float tip_weight = std::clamp(Smooth01(tip_u) * biological_tip_darkening_strength *
                                                        (1.0f - dry_weight * 0.65f),
                                                    0.0f, 0.65f);
                segment_color = glm::mix(segment_color, needle_tip_color, tip_weight);
              }
              segment_color = glm::mix(ApplyLiveNeedlePigments(segment_color, biological_material),
                                       ApplyOldNeedlePigments(segment_color, biological_material), dry_weight);
            }
            segment_color.r *= stripe_scale;
            segment_color.g *= stripe_scale;
            segment_color.b *= stripe_scale;
          }
          if (color_mode == ScotsPine::ColorMode::Shaded) {
            const uint32_t color_salt = node_hash ^ (needle_hash * 0x517cc1b7u) ^
                                        (static_cast<uint32_t>(i) * 0x45d9f3bu) ^ 0x2c1b3c6du;
            segment_color = ApplyMicroColorVariation(segment_color, needle.node_random, color_salt,
                                                     color_micro_variation);
            const float edge_darkening =
                std::clamp(clamped_edge_darkening + biological_facet_contrast * 0.16f +
                               sheath_visibility * clamped_sheath_browning_strength * 0.25f,
                           0.0f, 0.75f);
            const float color_scale =
                std::max(0.0f, 1.0f - edge_darkening * 0.25f - biological_facet_contrast * 0.04f);
            segment_color.r *= color_scale;
            segment_color.g *= color_scale;
            segment_color.b *= color_scale;
          }
          station_colors.emplace_back(segment_color);
        }
      }

      ScotsPine::NeedleSkeletonLine line;
      if (out_needle_skeleton_lines) {
        line.sheath_node_handle = static_cast<int>(sheath_handle);
        line.cluster_node_handle = static_cast<int>(sheath_handle);
        line.parent_node_handle = static_cast<int>(parent_handle);
        line.needle_index = n;
        line.initiation_year_index = needle.initiation_year_index;
        line.age_years = needle_age_years;
        line.length_m = length;
        line.target_length_m = needle.target_length;
        line.maturation_years = needle.continuous_growth.maturation_years;
        line.maturity_reached = needle.maturity_reached;
        line.year0_cohort = year0_cohort;
        line.points_world.reserve(static_cast<size_t>(kLogicalStations));
      }

      std::vector<StrandPoint> logical_points;
      logical_points.reserve(static_cast<size_t>(kLogicalStations));
      const float centerline_length = std::max(0.0f, centerline.TotalLength());

      for (int si = 0; si < kLogicalStations; ++si) {
        const float s_norm = static_cast<float>(si) / static_cast<float>(kLogicalStations - 1);
        const float s = centerline_length * s_norm;
        const auto sample = centerline.Sample(s);
        const glm::vec3 world_pos = anchor.base_position + anchor.orientation * sample.position;

        glm::vec3 tangent = sample.tangent;
        if (!IsFiniteVec3(tangent) || glm::length(tangent) <= 1.0e-8f) {
          tangent = glm::vec3(0.0f, 0.0f, 1.0f);
        } else {
          tangent = glm::normalize(tangent);
        }

        glm::vec3 local_normal = glm::vec3(1.0f, 0.0f, 0.0f) - tangent * glm::dot(glm::vec3(1.0f, 0.0f, 0.0f), tangent);
        if (!IsFiniteVec3(local_normal) || glm::length(local_normal) <= 1.0e-8f) {
          glm::vec3 ref(0.0f, 1.0f, 0.0f);
          if (std::abs(glm::dot(ref, tangent)) > 0.95f) {
            ref = glm::vec3(0.0f, 0.0f, 1.0f);
          }
          local_normal = ref - tangent * glm::dot(ref, tangent);
        }
        local_normal = glm::normalize(local_normal);
        if (std::abs(clamped_twist_turns) > 1.0e-5f) {
          const float twist_phase =
              DeterministicSignedUnitFromNodeRandom(needle.node_random, node_hash ^ (needle_hash * 0x9e3779b9u)) *
              glm::pi<float>();
          const glm::quat twist = glm::angleAxis(twist_phase + clamped_twist_turns * glm::two_pi<float>() * s_norm,
                                                 glm::normalize(tangent));
          local_normal = glm::normalize(twist * local_normal);
        }

        const std::size_t width_index = width_radius_table.empty()
                                            ? std::size_t{0}
                                            : std::min(static_cast<std::size_t>(si), width_radius_table.size() - 1);
        const std::size_t thickness_index =
            thickness_radius_table.empty() ? std::size_t{0}
                                           : std::min(static_cast<std::size_t>(si), thickness_radius_table.size() - 1);
        const float width_radius =
            width_radius_table.empty() ? kNeedleDefaultWidthRadiusM : width_radius_table[width_index];
        const float thickness_radius =
            thickness_radius_table.empty() ? kNeedleDefaultThicknessRadiusM : thickness_radius_table[thickness_index];
        const float segment_oldness =
            station_lignification.empty()
                ? 0.0f
                : station_lignification[std::min(static_cast<std::size_t>(si), station_lignification.size() - 1)];
        const float old_thinning_multiplier = glm::mix(1.0f, 1.0f - old_thinning_fraction, segment_oldness);

        StrandPoint point;
        point.position = world_pos;
        const float visual_width_radius = width_radius * old_thinning_multiplier;
        const float visual_thickness_radius = thickness_radius * old_thinning_multiplier;
        point.thickness = std::max(min_strand_thickness_m, std::max(visual_width_radius, visual_thickness_radius));
        point.normal = glm::normalize(anchor.orientation * local_normal);
        point.tex_coord = s_norm;
        point.color = SanitizeFiniteColor(
            station_colors[std::min(static_cast<std::size_t>(si), station_colors.size() - 1)], glm::vec4(1.0f));
        point.material_properties =
            NeedleSemiellipseProfileProperties(visual_width_radius, visual_thickness_radius);
        logical_points.emplace_back(point);

        if (out_needle_skeleton_lines) {
          line.points_world.emplace_back(world_pos);
        }
      }

      if (out_needle_skeleton_lines) {
        out_needle_skeleton_lines->emplace_back(std::move(line));
      }

      AppendEndpointPaddedNeedleControls(logical_points, min_strand_thickness_m, out_segments, out_points);
    }
  }
}

constexpr int kAnnotationNeedleSampleCount = 64;
constexpr int kAnnotationSheathSampleCount = 3;
constexpr int kAnnotationStemSampleCount = 64;

glm::vec3 SafeAnnotationPoint(const glm::vec3& value) {
  return IsFiniteVec3(value) ? value : glm::vec3(0.0f);
}

float SafeAnnotationScalar(const float value) {
  return std::isfinite(value) ? value : 0.0f;
}

float AnnotationPolylineLength(const std::vector<glm::vec3>& points) {
  float length = 0.0f;
  for (size_t i = 1; i < points.size(); ++i) {
    const glm::vec3 a = SafeAnnotationPoint(points[i - 1]);
    const glm::vec3 b = SafeAnnotationPoint(points[i]);
    length += glm::length(b - a);
  }
  return length;
}

glm::vec3 SampleAnnotationPolyline(const std::vector<glm::vec3>& points, float distance) {
  if (points.empty()) {
    return glm::vec3(0.0f);
  }
  if (points.size() == 1) {
    return SafeAnnotationPoint(points.front());
  }

  distance = std::max(0.0f, distance);
  float walked = 0.0f;
  for (size_t i = 1; i < points.size(); ++i) {
    const glm::vec3 a = SafeAnnotationPoint(points[i - 1]);
    const glm::vec3 b = SafeAnnotationPoint(points[i]);
    const float segment_length = glm::length(b - a);
    if (segment_length <= 1.0e-8f) {
      continue;
    }
    if (walked + segment_length >= distance) {
      const float t = std::clamp((distance - walked) / segment_length, 0.0f, 1.0f);
      return glm::mix(a, b, t);
    }
    walked += segment_length;
  }
  return SafeAnnotationPoint(points.back());
}

std::vector<glm::vec3> ResampleAnnotationPolyline(const std::vector<glm::vec3>& points, const int sample_count) {
  const int count = std::max(1, sample_count);
  std::vector<glm::vec3> samples;
  samples.reserve(static_cast<size_t>(count));
  const float length = AnnotationPolylineLength(points);
  for (int i = 0; i < count; ++i) {
    const float t = count == 1 ? 0.0f : static_cast<float>(i) / static_cast<float>(count - 1);
    samples.emplace_back(SampleAnnotationPolyline(points, length * t));
  }
  return samples;
}

glm::vec3 TransformAnnotationPoint(const glm::mat4& transform, const glm::vec3& point) {
  return SafeAnnotationPoint(glm::vec3(transform * glm::vec4(SafeAnnotationPoint(point), 1.0f)));
}

std::vector<glm::vec3> TransformAnnotationPoints(const glm::mat4& transform, const std::vector<glm::vec3>& points) {
  std::vector<glm::vec3> transformed;
  transformed.reserve(points.size());
  for (const auto& point : points) {
    transformed.emplace_back(TransformAnnotationPoint(transform, point));
  }
  return transformed;
}

void WriteAnnotationVec3Json(std::ostream& out, const glm::vec3& value) {
  const glm::vec3 p = SafeAnnotationPoint(value);
  out << '[' << p.x << ',' << p.y << ',' << p.z << ']';
}

void WriteAnnotationPointsJson(std::ostream& out, const std::vector<glm::vec3>& points) {
  out << '[';
  for (size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    WriteAnnotationVec3Json(out, points[i]);
  }
  out << ']';
}

void AppendAnnotationPoints(std::vector<glm::vec3>& out_points, const std::vector<glm::vec3>& points) {
  out_points.insert(out_points.end(), points.begin(), points.end());
}

}  // namespace

// ===========================================================================

float ScotsPine::GetInfancyTargetGDD() const {
  float descriptor_tgt = 18000.0f;
  if (auto desc = const_cast<AssetRef&>(descriptor_ref).Get<ScotsPineDescriptor>()) {
    std::mt19937 rng(seed);
    descriptor_tgt = std::max(0.0f, SampleDistribution(desc->target_gdd, rng));
  }
  if (descriptor_tgt <= 0.0f) {
    return 0.0f;
  }
  constexpr float kWarmStartRatio = 0.20f;
  constexpr float kWarmStartMinGdd = 250.0f;
  constexpr float kWarmStartMaxGdd = 900.0f;
  const float warm_start = std::clamp(descriptor_tgt * kWarmStartRatio, kWarmStartMinGdd, kWarmStartMaxGdd);
  return std::min(warm_start, descriptor_tgt);
}

void ScotsPine::SetGlobalColorMode(const ColorMode mode) {
  g_scots_pine_color_mode = mode;
}
ScotsPine::ColorMode ScotsPine::GetGlobalColorMode() {
  return g_scots_pine_color_mode;
}
void ScotsPine::SetGlobalSeasonalColorTint(const SeasonalColorTint& tint) {
  std::lock_guard<std::mutex> lock(g_seasonal_color_tint_mutex);
  g_scots_pine_seasonal_color_tint = SanitizeSeasonalColorTint(tint);
}
ScotsPine::SeasonalColorTint ScotsPine::GetGlobalSeasonalColorTint() {
  std::lock_guard<std::mutex> lock(g_seasonal_color_tint_mutex);
  return g_scots_pine_seasonal_color_tint;
}

void ScotsPine::SetInternodeVisualRadiusMultiplier(const float multiplier) {
  // Clamp to a sensible non-negative range. Caller may pass 0 to make the
  // visualised trunk vanish; negative values are nonsensical here.
  const float safe = std::isfinite(multiplier) ? std::max(multiplier, 0.0f) : 1.0f;
  g_internode_visual_radius_multiplier.store(safe, std::memory_order_relaxed);
}
float ScotsPine::GetInternodeVisualRadiusMultiplier() {
  return g_internode_visual_radius_multiplier.load(std::memory_order_relaxed);
}

void ScotsPine::SetRenderNeedlesEnabled(const bool enabled) {
  g_render_needles_enabled.store(enabled, std::memory_order_relaxed);
}
bool ScotsPine::IsRenderNeedlesEnabled() {
  return g_render_needles_enabled.load(std::memory_order_relaxed);
}

void ScotsPine::SetLeaderInternodeDebugColor(const glm::vec4& color) {
  std::lock_guard<std::mutex> lock(g_leader_internode_debug_color_mutex);
  g_leader_internode_debug_color = color;
}
glm::vec4 ScotsPine::GetLeaderInternodeDebugColor() {
  std::lock_guard<std::mutex> lock(g_leader_internode_debug_color_mutex);
  return g_leader_internode_debug_color;
}

// ===========================================================================

void ScotsPine::ClearGeometryEntities() const {
  if (render_target_) {
    render_target_->ClearAllChannels();
  }
  last_needle_skeleton_lines.clear();

  // Cleanup current generated child entities.
  const auto scene = GetScene();
  if (!scene) {
    return;
  }
  const auto self = GetOwner();
  if (!scene->IsEntityValid(self)) {
    return;
  }

  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    const auto name = scene->GetEntityName(child);
    if (name == "Pine Internodes" || name == "Pine Fascicle Sheaths" || name == "Pine Needles Strands") {
      scene->DeleteEntity(child);
    }
  }
}

// ===========================================================================
// Generate / Preview / Grow
// ===========================================================================

void ScotsPine::GenerateGeometryEntities(const bool uncapped_growth) {
  (void)uncapped_growth;
  ClearGeometryEntities();
  growth_model.Reset();
  const float requested_target_gdd = std::max(0.0f, target_gdd);
  target_gdd = 0.0f;
  ScotsPineCalendarSettings settings;
  const auto& times = GetApplication().GetTimes();
  const double grow_start = times.Now();
  const auto result = FastForwardScotsPineTemporalGrowth(*this, settings, requested_target_gdd);
  last_grow_seconds = times.Now() - grow_start;
  if (!result.reached_target) {
    EVOENGINE_WARNING("ScotsPine temporal fast-forward stopped before target GDD. target=" +
                      std::to_string(result.requested_target_gdd) + " reached=" +
                      std::to_string(result.target_gdd_after))
  }
}

void ScotsPine::GeneratePreviewGeometryEntities(const float preview_target_gdd,
                                                const uint32_t preview_max_growth_steps) {
  ClearGeometryEntities();
  growth_model.Reset();

  const auto& times = GetApplication().GetTimes();
  const double grow_start = times.Now();
  auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return;
  }
  const float original_target_gdd = target_gdd;
  const float clamped_target_gdd = std::min(original_target_gdd, std::max(0.0f, preview_target_gdd));
  target_gdd = 0.0f;
  ScotsPineCalendarSettings settings;
  const uint32_t calendar_step_cap = std::max<uint32_t>(4096u, std::max(1u, preview_max_growth_steps) * 64u);
  FastForwardScotsPineTemporalGrowth(*this, settings, clamped_target_gdd, 1.0f, calendar_step_cap);
  target_gdd = original_target_gdd;

  last_grow_seconds = times.Now() - grow_start;
}

bool ScotsPine::EnsureGrowthModelInitializedForGrowth() {
  if (growth_model.IsInitialized()) {
    return true;
  }
  auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    return false;
  }
  growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation, true);
  return true;
}

void ScotsPine::GrowToTargetGDD(const bool uncapped_growth, const bool rebuild_geometry) {
  (void)uncapped_growth;
  const auto& times = GetApplication().GetTimes();
  const double grow_start = times.Now();
  auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    last_grow_seconds = 0.0;
    return;
  }
  bool reinitialized = false;
  if (!growth_model.IsInitialized()) {
    if (!EnsureGrowthModelInitializedForGrowth()) {
      last_grow_seconds = 0.0;
      return;
    }
    reinitialized = true;
  }
  // Backward-scrubbing: if the user has dragged target_gdd backward beyond a step,
  // re-init from scratch so geometry shrinks instead of being stuck at the high-water mark.
  const float gdd_step = std::max(1e-5f, growth_model.gdd_per_growth_step);
  if (target_gdd + gdd_step < growth_model.accumulated_gdd) {
    growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation, true);
    reinitialized = true;
  }
  growth_model.GrowToGDD(target_gdd);
  last_grow_seconds = times.Now() - grow_start;

  const float current_internode_visual_radius_multiplier =
      std::max(0.0f, g_internode_visual_radius_multiplier.load(std::memory_order_relaxed));
  const bool current_render_needles_enabled = g_render_needles_enabled.load(std::memory_order_relaxed);
  const glm::vec4 current_leader_debug_color = []() {
    std::lock_guard<std::mutex> lock(g_leader_internode_debug_color_mutex);
    return g_leader_internode_debug_color;
  }();
  const int current_color_mode = static_cast<int>(GetGlobalColorMode());
  const auto approx_equal = [](const float a, const float b) {
    return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1.0e-6f;
  };
  const auto vec4_equal = [&](const glm::vec4& a, const glm::vec4& b) {
    return approx_equal(a.x, b.x) && approx_equal(a.y, b.y) && approx_equal(a.z, b.z) && approx_equal(a.w, b.w);
  };
  const bool visual_settings_changed =
      !approx_equal(last_applied_internode_visual_radius_multiplier, current_internode_visual_radius_multiplier) ||
      last_applied_render_needles_enabled != current_render_needles_enabled ||
      !vec4_equal(last_applied_leader_debug_color, current_leader_debug_color) ||
      last_applied_color_mode != current_color_mode;

  // Auto-grow calls this every frame; when no growth step was taken, a full
  // mesh rebuild is wasted work. Preserve exact behavior on explicit
  // reinitialization (backward scrub), where geometry must always be refreshed.
  // But loaded scenes may deserialize stale particle buffers; if any visual-only
  // knob changed, force one rebuild even when growth itself did not advance.
  if (!rebuild_geometry || (!reinitialized && growth_model.last_growth_steps == 0 && !visual_settings_changed)) {
    return;
  }
  RebuildGeometry();
}

void ScotsPine::SetSeasonalChronologicalMode(const bool enable_independent_chronological_clock) {
  growth_model.SetChronologicalCoupledToThermal(!enable_independent_chronological_clock);
}

bool ScotsPine::AdvanceChronologicalAging(const float delta_years) {
  if (!std::isfinite(delta_years) || delta_years <= 0.0f) {
    return false;
  }

  auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>();
  if (!descriptor) {
    return false;
  }

  if (!growth_model.IsInitialized()) {
    growth_model.Initialize(*descriptor, seed, glm::vec3(0), kDefaultRootRotation, true);
  }

  growth_model.AdvanceChronologicalYears(delta_years);
  growth_model.graph.data.clock.AdvanceYears(delta_years);
  const bool changed = growth_model.AgeOnlyStep();
  (void)changed;
  RebuildGeometry();
  return true;
}

// ===========================================================================
// Rebuild geometry from current growth model state
// ===========================================================================

void ScotsPine::RebuildGeometry() {
  const auto& times = GetApplication().GetTimes();
  const double rebuild_start = times.Now();
  last_invalid_instance_count = 0;
  last_internode_count = 0;
  last_needle_count = 0;
  last_node_count = 0;

  if (!growth_model.IsInitialized()) {
    last_rebuild_seconds = 0.0;
    return;
  }

  const auto scene = GetScene();
  const auto owner = GetOwner();
  if (!scene || !scene->IsEntityValid(owner)) {
    last_rebuild_seconds = 0.0;
    return;
  }

  if (!render_target_ || render_target_->GetRootEntity() != owner) {
    render_target_ = std::make_unique<PlantRenderTarget>(scene, owner);
  }

  const auto color_mode = GetGlobalColorMode();
  const glm::vec4 instance_color = HashToColor(owner.GetIndex());
  const glm::vec4 kDefaultNeedleColor(0.16f, 0.45f, 0.18f, 1.0f);
  const glm::vec4 kDefaultNeedleOldColor(0.42f, 0.27f, 0.10f, 1.0f);
  const glm::vec4 kDefaultNeedleTipColor(0.11f, 0.29f, 0.10f, 1.0f);
  const glm::vec4 kDefaultStemColor(0.83f, 0.72f, 0.50f, 1.0f);
  const glm::vec4 kDefaultStemOldColor(0.45f, 0.30f, 0.20f, 1.0f);
  const glm::vec4 kDefaultNodeSheathBrownColor(0.49f, 0.31f, 0.13f, 1.0f);
  const glm::vec4 kDefaultFascicleSheathColor(0.42f, 0.34f, 0.24f, 1.0f);
  constexpr float kDefaultNeedleTipColorMixStart = 0.35f;
  constexpr float kDefaultNeedleTipColorExponent = 1.35f;
  constexpr float kDefaultNeedleAxialAgeSpan = 0.35f;
  constexpr float kDefaultNeedleAxialAgeExponent = 1.0f;
  constexpr float kDefaultInternodeAgeExponent = 1.0f;

  glm::vec4 needle_base_color = kDefaultNeedleColor;
  glm::vec4 needle_old_color = kDefaultNeedleOldColor;
  glm::vec4 needle_tip_color = kDefaultNeedleTipColor;
  glm::vec4 stem_base_color = kDefaultStemColor;
  glm::vec4 stem_old_color = kDefaultStemOldColor;
  glm::vec4 node_sheath_brown_color = kDefaultNodeSheathBrownColor;
  glm::vec4 fascicle_sheath_color = kDefaultFascicleSheathColor;
  evo_engine::Plot2D<float> needle_axial_color_curve;
  evo_engine::Plot2D<float> needle_y_age_color_curve;
  evo_engine::Plot2D<float> stem_age_gradient_curve;
  ConfigureMaterialModelCurveDefaults(needle_axial_color_curve, 0.0f, 1.0f);
  ConfigureMaterialModelCurveDefaults(needle_y_age_color_curve, 0.0f, 1.0f);
  ConfigureMaterialModelCurveDefaults(stem_age_gradient_curve, 0.0f, 1.0f);
  float needle_tip_color_mix_start = kDefaultNeedleTipColorMixStart;
  float needle_tip_color_exponent = kDefaultNeedleTipColorExponent;
  float needle_old_thinning_fraction = 0.33f;
  float needle_min_strand_thickness_m = 0.00002f;
  float needle_micro_variation = 0.012f;
  float stem_micro_variation = 0.012f;
  float young_needle_roughness = 0.92f;
  float old_needle_roughness = 0.97f;
  float young_needle_specular = 0.09f;
  float old_needle_specular = 0.04f;
  float stem_roughness = 0.86f;
  float stem_specular = 0.12f;
  float node_browning_strength = 0.28f;
  float sheath_browning_strength = 0.40f;
  float node_browning_radius_norm = 0.18f;
  float needle_twist_turns = 0.25f;
  float needle_edge_darkening = 0.08f;
  float needle_axial_age_span = kDefaultNeedleAxialAgeSpan;
  float needle_axial_age_exponent = kDefaultNeedleAxialAgeExponent;
  float internode_age_exponent = kDefaultInternodeAgeExponent;
  BiologicalMaterialControls biological_material{};
  if (const auto descriptor = descriptor_ref.Get<ScotsPineDescriptor>()) {
    needle_base_color = descriptor->young_needle_palette_rgba;
    needle_old_color = descriptor->dry_brown_needle_palette_rgba;
    needle_tip_color = descriptor->older_needle_palette_rgba;
    stem_base_color = descriptor->main_stem_palette_rgba;
    stem_old_color = descriptor->mature_bark_stem_palette_rgba;
    node_sheath_brown_color = descriptor->node_sheath_brown_palette_rgba;
    fascicle_sheath_color = descriptor->fascicle_sheath_palette_rgba;
    needle_axial_color_curve = descriptor->needle_axial_color_curve;
    needle_y_age_color_curve = descriptor->needle_y_age_color_curve;
    stem_age_gradient_curve = descriptor->stem_age_gradient_curve;
    needle_tip_color_mix_start = descriptor->needle_tip_color_mix_start;
    needle_tip_color_exponent = descriptor->needle_tip_color_exponent;
    needle_old_thinning_fraction = descriptor->needle_old_thinning_fraction;
    needle_min_strand_thickness_m = descriptor->needle_min_strand_thickness_m;
    needle_micro_variation = descriptor->needle_micro_variation;
    stem_micro_variation = descriptor->stem_micro_variation;
    young_needle_roughness = descriptor->young_needle_roughness;
    old_needle_roughness = descriptor->old_needle_roughness;
    young_needle_specular = descriptor->young_needle_specular;
    old_needle_specular = descriptor->old_needle_specular;
    stem_roughness = descriptor->stem_roughness;
    stem_specular = descriptor->stem_specular;
    node_browning_strength = descriptor->node_browning_strength;
    sheath_browning_strength = descriptor->sheath_browning_strength;
    node_browning_radius_norm = descriptor->node_browning_radius_norm;
    needle_twist_turns = descriptor->needle_twist_turns;
    needle_edge_darkening = descriptor->needle_edge_darkening;
    needle_axial_age_span = descriptor->needle_axial_age_span;
    needle_axial_age_exponent = descriptor->needle_axial_age_exponent;
    internode_age_exponent = descriptor->internode_age_exponent;
    biological_material.strength = descriptor->biological_material_model_strength;
    biological_material.chlorophyll_scale = descriptor->biological_chlorophyll_scale;
    biological_material.carotenoid_gold_scale = descriptor->biological_carotenoid_gold_scale;
    biological_material.lignin_bark_scale = descriptor->biological_lignin_bark_scale;
    biological_material.senescence_bias = descriptor->biological_senescence_bias;
    biological_material.cuticle_wax = descriptor->biological_cuticle_wax;
    biological_material.individual_variation = descriptor->biological_individual_variation;
    biological_material.facet_contrast = descriptor->biological_facet_contrast;
    biological_material.tip_darkening_strength = descriptor->biological_tip_darkening_strength;
    biological_material.stem_age_browning_scale = descriptor->biological_stem_age_browning_scale;
  }
  needle_base_color = SanitizeFiniteColor(needle_base_color, kDefaultNeedleColor);
  needle_old_color = SanitizeFiniteColor(needle_old_color, kDefaultNeedleOldColor);
  needle_tip_color = SanitizeFiniteColor(needle_tip_color, kDefaultNeedleTipColor);
  stem_base_color = SanitizeFiniteColor(stem_base_color, kDefaultStemColor);
  stem_old_color = SanitizeFiniteColor(stem_old_color, kDefaultStemOldColor);
  node_sheath_brown_color = SanitizeFiniteColor(node_sheath_brown_color, kDefaultNodeSheathBrownColor);
  fascicle_sheath_color = SanitizeFiniteColor(fascicle_sheath_color, kDefaultFascicleSheathColor);
  needle_tip_color_mix_start = std::clamp(needle_tip_color_mix_start, 0.0f, 1.0f);
  needle_tip_color_exponent = std::clamp(needle_tip_color_exponent, 0.1f, 6.0f);
  needle_old_thinning_fraction = std::clamp(needle_old_thinning_fraction, 0.0f, 0.95f);
  needle_min_strand_thickness_m = std::clamp(needle_min_strand_thickness_m, 0.000001f, 0.002f);
  needle_micro_variation = std::clamp(needle_micro_variation, 0.0f, 0.25f);
  stem_micro_variation = std::clamp(stem_micro_variation, 0.0f, 0.25f);
  young_needle_roughness = std::clamp(young_needle_roughness, 0.02f, 1.0f);
  old_needle_roughness = std::clamp(old_needle_roughness, 0.02f, 1.0f);
  young_needle_specular = std::clamp(young_needle_specular, 0.0f, 1.0f);
  old_needle_specular = std::clamp(old_needle_specular, 0.0f, 1.0f);
  stem_roughness = std::clamp(stem_roughness, 0.02f, 1.0f);
  stem_specular = std::clamp(stem_specular, 0.0f, 1.0f);
  node_browning_strength = std::clamp(node_browning_strength, 0.0f, 1.0f);
  sheath_browning_strength = std::clamp(sheath_browning_strength, 0.0f, 1.0f);
  node_browning_radius_norm = std::clamp(node_browning_radius_norm, 0.0f, 1.0f);
  needle_twist_turns = std::clamp(needle_twist_turns, -8.0f, 8.0f);
  needle_edge_darkening = std::clamp(needle_edge_darkening, 0.0f, 0.75f);
  biological_material.strength = std::clamp(biological_material.strength, 0.0f, 1.0f);
  biological_material.chlorophyll_scale = std::clamp(biological_material.chlorophyll_scale, 0.2f, 2.0f);
  biological_material.carotenoid_gold_scale = std::clamp(biological_material.carotenoid_gold_scale, 0.0f, 2.0f);
  biological_material.lignin_bark_scale = std::clamp(biological_material.lignin_bark_scale, 0.0f, 2.0f);
  biological_material.senescence_bias = std::clamp(biological_material.senescence_bias, -0.5f, 0.5f);
  biological_material.cuticle_wax = std::clamp(biological_material.cuticle_wax, 0.0f, 1.0f);
  biological_material.individual_variation = std::clamp(biological_material.individual_variation, 0.0f, 1.0f);
  biological_material.facet_contrast = std::clamp(biological_material.facet_contrast, 0.0f, 1.0f);
  biological_material.tip_darkening_strength = std::clamp(biological_material.tip_darkening_strength, 0.0f, 1.0f);
  biological_material.stem_age_browning_scale =
      std::clamp(biological_material.stem_age_browning_scale, 0.0f, 2.0f);

  const float biological_strength = BiologicalStrength(biological_material);
  if (biological_strength > 1.0e-6f) {
    needle_base_color = ApplyLiveNeedlePigments(needle_base_color, biological_material);
    needle_tip_color = ApplyLiveNeedlePigments(needle_tip_color, biological_material);
    needle_old_color = ApplyOldNeedlePigments(needle_old_color, biological_material);
    stem_base_color = ApplyStemPigments(stem_base_color, biological_material, 0.15f);
    stem_old_color = ApplyStemPigments(stem_old_color, biological_material, 1.0f);
    const float extra_variation = biological_strength * biological_material.individual_variation * 0.08f;
    needle_micro_variation = std::clamp(std::max(needle_micro_variation, extra_variation), 0.0f, 0.25f);
    stem_micro_variation = std::clamp(std::max(stem_micro_variation, extra_variation * 0.75f), 0.0f, 0.25f);
    const float facet = biological_strength * biological_material.facet_contrast;
    needle_edge_darkening = std::clamp(needle_edge_darkening + facet * 0.10f, 0.0f, 0.75f);
    young_needle_specular = std::clamp(young_needle_specular + facet * 0.04f + biological_material.cuticle_wax * 0.03f,
                                       0.0f, 1.0f);
    old_needle_specular = std::clamp(old_needle_specular + facet * 0.02f, 0.0f, 1.0f);
    stem_specular = std::clamp(stem_specular + facet * 0.015f, 0.0f, 1.0f);
  }

  const float needle_color_energy = std::max(needle_base_color.r, std::max(needle_base_color.g, needle_base_color.b));
  const float needle_old_color_energy = std::max(needle_old_color.r, std::max(needle_old_color.g, needle_old_color.b));
  if (needle_color_energy <= 1.0e-4f && needle_old_color_energy <= 1.0e-4f) {
    needle_base_color = kDefaultNeedleColor;
    needle_old_color = kDefaultNeedleOldColor;
  }
  const float stem_color_energy = std::max(stem_base_color.r, std::max(stem_base_color.g, stem_base_color.b));
  const float stem_old_color_energy = std::max(stem_old_color.r, std::max(stem_old_color.g, stem_old_color.b));
  if (stem_color_energy <= 1.0e-4f && stem_old_color_energy <= 1.0e-4f) {
    stem_base_color = kDefaultStemColor;
    stem_old_color = kDefaultStemOldColor;
  }

  const float t_now_years = growth_model.graph.data.clock.NowYears();
  const float max_internode_age_years = std::max(1.0f, static_cast<float>(growth_model.sampled.needle_lifespan_years));
  const SeasonalColorTint seasonal_tint =
      color_mode == ColorMode::Shaded ? GetGlobalSeasonalColorTint() : SeasonalColorTint{};
  needle_base_color = ApplySeasonalColorTint(needle_base_color, seasonal_tint);
  needle_old_color = ApplySeasonalColorTint(needle_old_color, seasonal_tint);
  needle_tip_color = ApplySeasonalColorTint(needle_tip_color, seasonal_tint);
  stem_base_color = ApplySeasonalColorTint(stem_base_color, seasonal_tint);
  stem_old_color = ApplySeasonalColorTint(stem_old_color, seasonal_tint);
  node_sheath_brown_color = ApplySeasonalColorTint(node_sheath_brown_color, seasonal_tint);
  fascicle_sheath_color = ApplySeasonalColorTint(fascicle_sheath_color, seasonal_tint);
  const glm::vec3 instance_albedo = glm::clamp(glm::vec3(instance_color), glm::vec3(0.0f), glm::vec3(1.0f));
  const glm::vec3 shaded_stem_albedo = glm::vec3(glm::mix(stem_base_color, stem_old_color, 0.35f));
  const glm::vec3 shaded_sheath_albedo = glm::vec3(fascicle_sheath_color);
  const glm::vec3 shaded_needle_albedo = glm::vec3(glm::mix(needle_base_color, needle_tip_color, 0.35f));

  static thread_local std::vector<ParticleInfo> internode_infos_cache;

  const auto& sorted = growth_model.graph.PeekSortedNodeList();
  last_node_count = static_cast<uint32_t>(sorted.size());

  LSystemRenderSnapshot render_snapshot;
  render_snapshot.version = g_scots_pine_snapshot_version.fetch_add(1, std::memory_order_relaxed) + 1;
  std::size_t expected_needle_segment_count = 0;
  std::size_t expected_needle_point_count = 0;

  // -- Internodes (instance channel) --
  {
    auto& staged_instances = internode_infos_cache;
    staged_instances.clear();
    render_snapshot.instances.clear();
    render_snapshot.instances.reserve(sorted.size());
    const glm::quat cylinder_axis_fix = glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));

    const float internode_visual_radius_multiplier =
        std::max(0.0f, g_internode_visual_radius_multiplier.load(std::memory_order_relaxed));
    const glm::vec4 leader_debug_color = []() {
      std::lock_guard<std::mutex> lock(g_leader_internode_debug_color_mutex);
      return g_leader_internode_debug_color;
    }();
    const bool leader_debug_color_active = leader_debug_color.a > 0.0f;

    for (const auto handle : sorted) {
      const auto& node = growth_model.graph.PeekNode(handle);
      if (ResolvePineModuleRenderRole(node.data) != PineModuleRenderRole::StemInstance)
        continue;
      const auto& internode = node.data.template Get<PineInternode>();
      if (node.info.length <= 0.0f)
        continue;
      if (!IsFiniteVec3(node.info.global_position) || !std::isfinite(node.info.length) ||
          !std::isfinite(node.info.thickness)) {
        last_invalid_instance_count++;
        continue;
      }
      const float per_node_visual_multiplier = node.info.order == 0 ? internode_visual_radius_multiplier : 1.0f;
      const float half_thick = node.info.thickness * 0.5f * per_node_visual_multiplier;
      if (half_thick <= 0.0f)
        continue;

      const float internode_age_years = std::max(0.0f, t_now_years - internode.continuous_growth.t_init_years);
      float internode_age_norm = std::clamp(internode_age_years / max_internode_age_years, 0.0f, 1.0f);
      internode_age_norm = std::pow(internode_age_norm, std::max(0.1f, internode_age_exponent));
      const bool year0_stem_cohort = internode.year_produced == 0;
      const float stem_curve_weight = std::clamp(stem_age_gradient_curve.GetValue(internode_age_norm), 0.0f, 1.0f);
      float stem_old_weight =
          year0_stem_cohort ? std::min(0.20f, stem_curve_weight * 0.20f) : std::max(0.55f, stem_curve_weight);
      stem_old_weight =
          std::clamp(stem_old_weight *
                         glm::mix(1.0f, biological_material.stem_age_browning_scale, biological_strength),
                     0.0f, 1.0f);
      const float node_kernel =
          std::clamp(node_browning_strength *
                         glm::mix(1.0f, biological_material.lignin_bark_scale, biological_strength) *
                         std::max(stem_old_weight, node.info.order == 0 ? node_browning_radius_norm : 0.0f) *
                         (0.65f + 0.35f * std::clamp(internode.node_random, 0.0f, 1.0f)),
                     0.0f, 1.0f);
      glm::vec4 stem_age_color = glm::mix(stem_base_color, stem_old_color, stem_old_weight);
      stem_age_color = glm::mix(stem_age_color, node_sheath_brown_color, node_kernel);
      stem_age_color = ApplyStemPigments(stem_age_color, biological_material, std::max(stem_old_weight, node_kernel));
      stem_age_color.a = 1.0f;
      stem_age_color = ApplyMicroColorVariation(stem_age_color, internode.node_random,
                                                static_cast<uint32_t>(node.GetIndex()) ^ 0x74a7c15u,
                                                stem_micro_variation);
      const glm::vec4 stem_type_color(stem_base_color.r, stem_base_color.g, stem_base_color.b, 1.0f);

      glm::quat instance_rotation = glm::normalize(node.info.global_rotation * cylinder_axis_fix);
      if (!IsFiniteQuat(instance_rotation)) {
        instance_rotation = glm::quat(1, 0, 0, 0);
        last_invalid_instance_count++;
      }

      const glm::mat4 model = glm::translate(node.info.global_position) * glm::mat4_cast(instance_rotation) *
                              glm::scale(glm::vec3(half_thick, node.info.length, half_thick));
      if (!IsFiniteMat4(model)) {
        last_invalid_instance_count++;
        continue;
      }

      ParticleInfo pi;
      pi.instance_matrix.value = model;
      if (color_mode == ColorMode::ByNode) {
        pi.instance_color = HashToColor(static_cast<uint32_t>(node.GetIndex()));
      } else if (color_mode == ColorMode::ByInstance) {
        pi.instance_color = instance_color;
      } else if (color_mode == ColorMode::ByType) {
        pi.instance_color = stem_type_color;
      } else if (color_mode == ColorMode::SyntheticOrganLabels) {
        pi.instance_color = year0_stem_cohort ? kSyntheticLabelStemYear0 : kSyntheticLabelStemLater;
      } else {
        pi.instance_color = stem_age_color;
      }
      if (leader_debug_color_active && node.info.order == 0) {
        pi.instance_color = leader_debug_color;
      }
      pi.instance_color.a = 1.0f;
      InstancePrimitiveSnapshot primitive_snapshot{};
      primitive_snapshot.key.node_handle = static_cast<int>(handle);
      primitive_snapshot.key.flow_handle = static_cast<int>(node.GetFlowHandle());
      primitive_snapshot.key.symbol_id = static_cast<int>(PineModuleRenderRole::StemInstance);
      primitive_snapshot.instance = ToSnapshotInstance(pi);
      render_snapshot.instances.emplace_back(std::move(primitive_snapshot));
    }

    last_internode_count = static_cast<uint32_t>(render_snapshot.instances.size());
    if (render_snapshot.instances.empty()) {
      render_target_->RemoveInstanceChannel(kChannelInternodes);
    } else {
      BuildInstancePayloadFromSnapshot(render_snapshot.instances, staged_instances);

      std::shared_ptr<Mesh> internode_mesh;
      std::shared_ptr<Material> internode_material;
      const auto& channels = render_target_->GetInstanceChannels();
      if (const auto it = channels.find(kChannelInternodes); it != channels.end()) {
        internode_mesh = it->second->GetInstanceMesh();
        internode_material = it->second->GetInstanceMaterial();
      }
      if (!internode_mesh) {
        internode_mesh = AssetManager::CreateTemporaryAsset<Mesh>();
      }
      if (!internode_material) {
        internode_material = AssetManager::CreateTemporaryAsset<Material>();
      }

      bool needs_mesh_rebuild = true;
      if (internode_mesh) {
        const auto& vertices = internode_mesh->UnsafeGetVertices();
        const auto& triangles = internode_mesh->UnsafeGetTriangles();
        needs_mesh_rebuild = vertices.empty() || triangles.empty();
        if (!needs_mesh_rebuild) {
          for (const auto& vertex : vertices) {
            const auto color = vertex.color;
            const bool near_white = std::abs(color.r - 1.0f) <= 1.0e-3f && std::abs(color.g - 1.0f) <= 1.0e-3f &&
                                    std::abs(color.b - 1.0f) <= 1.0e-3f;
            if (!IsFiniteVec4(color) || !near_white) {
              needs_mesh_rebuild = true;
              break;
            }
          }
        }
      }

      if (needs_mesh_rebuild) {
        std::vector<Vertex> cyl_verts;
        std::vector<unsigned int> cyl_idx;
        GenerateUnitCylinderMesh(cyl_verts, cyl_idx, glm::vec4(1.0f));
        VertexAttributes attrs{};
        attrs.normal = true;
        attrs.color = true;
        attrs.tex_coord = true;
        internode_mesh->SetVertices(attrs, cyl_verts, cyl_idx);
      }

      internode_material->vertex_color_only = true;
      internode_material->SetAlbedoTexture(nullptr);
      internode_material->material_properties.albedo_color =
          color_mode == ColorMode::SyntheticOrganLabels
              ? glm::vec3(kSyntheticLabelStemYear0)
              : color_mode == ColorMode::ByInstance ? instance_albedo : shaded_stem_albedo;
      internode_material->draw_settings.blending = false;
      internode_material->material_properties.metallic = 0.0f;
      internode_material->material_properties.specular = 0.12f;
      internode_material->material_properties.specular_tint = 0.05f;
      internode_material->material_properties.roughness = 0.86f;
      internode_material->material_properties.transmission = 0.0f;
      internode_material->material_properties.subsurface_factor = 0.0f;
      internode_material->material_properties.clear_coat = 0.0f;

      if (auto* channel = render_target_->GetOrCreateInstanceChannel(kChannelInternodes, "Pine Internodes",
                                                                     internode_mesh, internode_material)) {
        channel->Stage(std::move(staged_instances));
      }
    }
  }

  const bool render_needles_enabled = g_render_needles_enabled.load(std::memory_order_relaxed);

  // -- Fascicle sheaths (instance channel) --
  if (!render_needles_enabled) {
    render_target_->RemoveInstanceChannel(kChannelNeedleSheaths);
    last_sheath_count = 0;
  } else {
    static thread_local std::vector<ParticleInfo> sheath_infos_cache;
    auto& staged_sheaths = sheath_infos_cache;
    staged_sheaths.clear();

    std::vector<InstancePrimitiveSnapshot> sheath_snapshots;
    sheath_snapshots.reserve(sorted.size());
    const glm::quat cylinder_axis_fix = glm::angleAxis(-glm::half_pi<float>(), glm::vec3(1.0f, 0.0f, 0.0f));

    for (const auto handle : sorted) {
      const auto& node = growth_model.graph.PeekNode(handle);
      if (ResolvePineModuleRenderRole(node.data) != PineModuleRenderRole::SheathInstance)
        continue;
      const auto& sheath = node.data.template Get<PineNeedleSheath>();
      if (sheath.length <= 0.0f || sheath.width <= 0.0f)
        continue;

      const LNodeHandle parent_handle = FindParentInternodeNodeHandle(growth_model.graph, handle);
      if (parent_handle < 0)
        continue;
      const auto& parent_node = growth_model.graph.PeekNode(parent_handle);
      if (!IsFiniteVec3(parent_node.info.global_position) || !std::isfinite(parent_node.info.length)) {
        last_invalid_instance_count++;
        continue;
      }

      const float active_branching_angle_deg = ComputeActiveSheathBranchingAngleDeg(growth_model.graph, sheath);
      const FascicleSheathFrame sheath_frame = ComputeFascicleSheathFrame(sheath, parent_node, active_branching_angle_deg);
      if (!IsFiniteVec3(sheath_frame.base_position) || !IsFiniteQuat(sheath_frame.orientation)) {
        last_invalid_instance_count++;
        continue;
      }

      const float half_width = std::max(0.00001f, sheath.width * 0.5f);
      glm::quat instance_rotation = glm::normalize(sheath_frame.orientation * cylinder_axis_fix);
      if (!IsFiniteQuat(instance_rotation)) {
        instance_rotation = glm::quat(1, 0, 0, 0);
        last_invalid_instance_count++;
      }

      const glm::mat4 model = glm::translate(sheath_frame.base_position) * glm::mat4_cast(instance_rotation) *
                              glm::scale(glm::vec3(half_width, sheath.length, half_width));
      if (!IsFiniteMat4(model)) {
        last_invalid_instance_count++;
        continue;
      }

      ParticleInfo pi;
      pi.instance_matrix.value = model;
      if (color_mode == ColorMode::ByNode) {
        pi.instance_color = HashToColor(static_cast<uint32_t>(node.GetIndex()));
      } else if (color_mode == ColorMode::ByInstance) {
        pi.instance_color = instance_color;
      } else if (color_mode == ColorMode::SyntheticOrganLabels) {
        pi.instance_color = kSyntheticLabelFascicleSheath;
      } else if (color_mode == ColorMode::NeedleSheath) {
        pi.instance_color = glm::vec4(0.95f, 0.75f, 0.20f, 1.0f);
      } else {
        pi.instance_color = ApplyMicroColorVariation(fascicle_sheath_color, sheath.node_random,
                                                     static_cast<uint32_t>(node.GetIndex()) ^ 0xb5157eafu,
                                                     stem_micro_variation);
      }
      pi.instance_color.a = 1.0f;
      InstancePrimitiveSnapshot primitive_snapshot{};
      primitive_snapshot.key.node_handle = static_cast<int>(handle);
      primitive_snapshot.key.flow_handle = static_cast<int>(node.GetFlowHandle());
      primitive_snapshot.key.symbol_id = static_cast<int>(PineModuleRenderRole::SheathInstance);
      primitive_snapshot.instance = ToSnapshotInstance(pi);
      sheath_snapshots.emplace_back(std::move(primitive_snapshot));
    }

    last_sheath_count = static_cast<uint32_t>(sheath_snapshots.size());
    if (sheath_snapshots.empty()) {
      render_target_->RemoveInstanceChannel(kChannelNeedleSheaths);
    } else {
      BuildInstancePayloadFromSnapshot(sheath_snapshots, staged_sheaths);
      render_snapshot.instances.insert(render_snapshot.instances.end(), sheath_snapshots.begin(), sheath_snapshots.end());

      std::shared_ptr<Mesh> sheath_mesh;
      std::shared_ptr<Material> sheath_material;
      const auto& channels = render_target_->GetInstanceChannels();
      if (const auto it = channels.find(kChannelNeedleSheaths); it != channels.end()) {
        sheath_mesh = it->second->GetInstanceMesh();
        sheath_material = it->second->GetInstanceMaterial();
      }
      if (!sheath_mesh) {
        sheath_mesh = AssetManager::CreateTemporaryAsset<Mesh>();
      }
      if (!sheath_material) {
        sheath_material = AssetManager::CreateTemporaryAsset<Material>();
      }

      bool needs_mesh_rebuild = true;
      if (sheath_mesh) {
        const auto& vertices = sheath_mesh->UnsafeGetVertices();
        const auto& triangles = sheath_mesh->UnsafeGetTriangles();
        needs_mesh_rebuild = vertices.empty() || triangles.empty();
      }
      if (needs_mesh_rebuild) {
        std::vector<Vertex> cyl_verts;
        std::vector<unsigned int> cyl_idx;
        GenerateUnitCylinderMesh(cyl_verts, cyl_idx, glm::vec4(1.0f));
        VertexAttributes attrs{};
        attrs.normal = true;
        attrs.color = true;
        attrs.tex_coord = true;
        sheath_mesh->SetVertices(attrs, cyl_verts, cyl_idx);
      }

      sheath_material->vertex_color_only = true;
      sheath_material->SetAlbedoTexture(nullptr);
      sheath_material->material_properties.albedo_color =
          color_mode == ColorMode::SyntheticOrganLabels
              ? glm::vec3(kSyntheticLabelFascicleSheath)
              : color_mode == ColorMode::ByInstance
                    ? instance_albedo
                    : color_mode == ColorMode::NeedleSheath ? glm::vec3(0.95f, 0.75f, 0.20f) : shaded_sheath_albedo;
      sheath_material->draw_settings.blending = false;
      sheath_material->material_properties.metallic = 0.0f;
      sheath_material->material_properties.specular = std::clamp(stem_specular * 0.65f, 0.0f, 1.0f);
      sheath_material->material_properties.specular_tint = 0.04f;
      sheath_material->material_properties.roughness = std::clamp(stem_roughness + 0.05f, 0.02f, 1.0f);
      sheath_material->material_properties.transmission = 0.0f;
      sheath_material->material_properties.subsurface_factor = 0.0f;
      sheath_material->material_properties.clear_coat = 0.0f;

      if (auto* channel = render_target_->GetOrCreateInstanceChannel(kChannelNeedleSheaths, "Pine Fascicle Sheaths",
                                                                     sheath_mesh, sheath_material)) {
        channel->Stage(std::move(staged_sheaths));
      }
    }
  }

  // -- Needles (strands-only) --
  if (!render_needles_enabled) {
    render_target_->RemoveMeshChannel(kChannelNeedles);
    render_target_->RemoveInstanceChannel(kChannelNeedles);
    render_target_->RemoveStrandsChannel(kChannelNeedles);
    last_needle_skeleton_lines.clear();
    last_needle_count = 0;
  } else {
    static thread_local std::vector<glm::uint> needle_segments;
    static thread_local std::vector<StrandPoint> needle_points;
    static thread_local std::vector<SnapshotStrandPoint> needle_snapshot_points;

    const int needle_logical_station_count = std::max(2, growth_model.sampled.needle_segment_count + 1);

    BuildPineNeedleStrandsPayload(
        growth_model.graph, sorted, color_mode, needle_base_color, needle_old_color, needle_tip_color,
        node_sheath_brown_color, needle_axial_color_curve, needle_y_age_color_curve, needle_tip_color_mix_start,
        needle_tip_color_exponent, needle_old_thinning_fraction, needle_min_strand_thickness_m, needle_micro_variation,
        needle_axial_age_span, needle_axial_age_exponent, sheath_browning_strength, needle_twist_turns,
        needle_edge_darkening, biological_material,
        growth_model.sampled.distributions.needle_cross_section_width_profile,
        growth_model.sampled.distributions.needle_cross_section_thickness_profile,
        growth_model.sampled.distributions.needle_cross_section_temporal_maturity_curve,
        growth_model.sampled.needle_lignification_factor_year0,
        growth_model.sampled.needle_stomatal_strip_density_year0,
        growth_model.sampled.needle_basal_taper_ratio_year0, growth_model.sampled.needle_fascicle_sheath_budget_years,
        growth_model.sampled.needle_specularity_plasticity_year0, needle_segments, needle_points,
        needle_logical_station_count, &last_needle_skeleton_lines);

    last_needle_count = static_cast<uint32_t>(last_needle_skeleton_lines.size());

    render_target_->RemoveMeshChannel(kChannelNeedles);
    render_target_->RemoveInstanceChannel(kChannelNeedles);

    render_snapshot.strands.clear();
    if (!needle_segments.empty() && !needle_points.empty()) {
      StrandsPrimitiveSnapshot needle_snapshot{};
      needle_snapshot.key.node_handle = -1;
      needle_snapshot.key.flow_handle = -1;
      needle_snapshot.key.symbol_id = static_cast<int>(PineModuleRenderRole::NeedleStrands);
      needle_snapshot.attributes.normal = true;
      needle_snapshot.attributes.color = true;
      needle_snapshot.attributes.tex_coord = true;
      needle_snapshot.segments = std::move(needle_segments);

      needle_snapshot_points.clear();
      needle_snapshot_points.reserve(needle_points.size());
      for (const auto& point : needle_points) {
        needle_snapshot_points.emplace_back(ToSnapshotStrandPoint(point));
      }
      needle_snapshot.points = std::move(needle_snapshot_points);
      render_snapshot.strands.emplace_back(std::move(needle_snapshot));
    }

    StrandPointAttributes needle_attributes{};
    BuildStrandsPayloadFromSnapshot(render_snapshot.strands, needle_attributes, needle_segments, needle_points);
    expected_needle_segment_count = needle_segments.size();
    expected_needle_point_count = needle_points.size();

    if (!needle_segments.empty() && !needle_points.empty()) {
      if (auto* strands_channel = render_target_->GetOrCreateStrandsChannel(kChannelNeedles, "Pine Needles Strands")) {
        if (const auto needle_material = strands_channel->GetMaterial()) {
          const float needle_specularity =
              std::clamp(0.5f * (growth_model.sampled.needle_specularity_plasticity_year0 + 1.0f), 0.0f, 1.0f);
          const float stomatal_density =
              std::clamp(0.5f * (growth_model.sampled.needle_stomatal_strip_density_year0 + 1.0f), 0.0f, 1.0f);
          const float needle_lignification =
              std::clamp(0.5f * (growth_model.sampled.needle_lignification_factor_year0 + 1.0f), 0.0f, 2.0f);

          needle_material->vertex_color_only = true;
          needle_material->SetAlbedoTexture(nullptr);
          needle_material->material_properties.albedo_color =
              color_mode == ColorMode::SyntheticOrganLabels
                  ? glm::vec3(kSyntheticLabelNeedleYear0)
                  : color_mode == ColorMode::ByInstance
                        ? instance_albedo
                        : color_mode == ColorMode::NeedleLignification
                              ? glm::vec3(0.40f, 0.32f, 0.09f)
                              : color_mode == ColorMode::NeedleStripeProxy
                                    ? glm::vec3(0.35f, 0.55f, 0.70f)
                                    : color_mode == ColorMode::NeedleSheath ? glm::vec3(0.95f, 0.75f, 0.20f)
                                                                            : shaded_needle_albedo;
          needle_material->draw_settings.blending = false;
          needle_material->draw_settings.cull_mode = VK_CULL_MODE_NONE;
          needle_material->material_properties.metallic = 0.0f;
          needle_material->material_properties.specular =
              std::clamp(0.5f * (young_needle_specular + old_needle_specular), 0.0f, 1.0f);
          needle_material->material_properties.specular_tint =
              std::clamp(0.02f + 0.05f * stomatal_density, 0.0f, 0.10f);
          needle_material->material_properties.roughness =
              std::clamp(0.5f * (young_needle_roughness + old_needle_roughness), 0.02f, 1.0f);
          needle_material->material_properties.subsurface_factor = 0.0f;
          needle_material->material_properties.ior = 1.33f;
          needle_material->material_properties.transmission = 0.0f;
          needle_material->material_properties.transmission_roughness = 1.0f;
          needle_material->material_properties.clear_coat = 0.0f;
          needle_material->material_properties.clear_coat_roughness = 1.0f;
          needle_material->material_properties.emission = 0.0f;
        }

        strands_channel->Stage(needle_attributes, std::move(needle_segments), std::move(needle_points));
      }
    } else {
      render_target_->RemoveStrandsChannel(kChannelNeedles);
    }
  }

  if (render_target_) {
    render_target_->FlushPending();
  }

  const auto raytrace_parity = EvaluateScotsPineRaytraceParity(scene, render_target_.get(), last_internode_count,
                                                               last_sheath_count, expected_needle_segment_count,
                                                               expected_needle_point_count);
  last_render_snapshot_version = render_snapshot.version;
  last_raytrace_internodes_ready = raytrace_parity.internodes_ready;
  last_raytrace_sheaths_ready = raytrace_parity.sheaths_ready;
  last_raytrace_needles_ready = raytrace_parity.needles_ready;
  last_raytrace_internode_instances = static_cast<uint32_t>(
      std::min(raytrace_parity.internode_instances, std::size_t{std::numeric_limits<uint32_t>::max()}));
  last_raytrace_sheath_instances = static_cast<uint32_t>(
      std::min(raytrace_parity.sheath_instances, std::size_t{std::numeric_limits<uint32_t>::max()}));
  last_raytrace_needle_segments = static_cast<uint32_t>(
      std::min(raytrace_parity.needle_segments, std::size_t{std::numeric_limits<uint32_t>::max()}));
  last_raytrace_needle_points =
      static_cast<uint32_t>(std::min(raytrace_parity.needle_points, std::size_t{std::numeric_limits<uint32_t>::max()}));

  last_applied_internode_visual_radius_multiplier =
      std::max(0.0f, g_internode_visual_radius_multiplier.load(std::memory_order_relaxed));
  last_applied_render_needles_enabled = g_render_needles_enabled.load(std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(g_leader_internode_debug_color_mutex);
    last_applied_leader_debug_color = g_leader_internode_debug_color;
  }
  last_applied_color_mode = static_cast<int>(GetGlobalColorMode());
  last_rebuild_seconds = times.Now() - rebuild_start;
}

// ===========================================================================
// Export
// ===========================================================================

void ScotsPine::ExportObj(const std::filesystem::path& path) const {
  const auto scene = GetScene();
  if (!scene)
    return;

  if (!render_target_) {
    EVOENGINE_ERROR("Pine mesh export failed: no render channels available.");
    return;
  }

  std::vector<Vertex> vertices;
  std::vector<glm::uvec3> triangles;

  for (const auto& [channel_id, channel] : render_target_->GetInstanceChannels()) {
    (void)channel_id;
    if (!channel) {
      continue;
    }
    const auto entity = channel->GetEntity();
    if (scene->IsEntityValid(entity)) {
      AppendParticlesToMesh(scene, entity, vertices, triangles);
    }
  }
  for (const auto& [channel_id, channel] : render_target_->GetMeshChannels()) {
    (void)channel_id;
    if (!channel) {
      continue;
    }
    const auto entity = channel->GetEntity();
    if (scene->IsEntityValid(entity)) {
      AppendMeshRendererToMesh(scene, entity, vertices, triangles);
    }
  }
  for (const auto& [channel_id, channel] : render_target_->GetStrandsChannels()) {
    (void)channel_id;
    if (!channel) {
      continue;
    }
    const auto entity = channel->GetEntity();
    if (scene->IsEntityValid(entity)) {
      AppendStrandsRendererToMesh(scene, entity, vertices, triangles);
    }
  }

  if (vertices.empty() || triangles.empty()) {
    EVOENGINE_ERROR("Pine mesh export failed: no pine channel geometry available.");
    return;
  }

  const auto mesh = AssetManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes vertex_attributes{};
  vertex_attributes.normal = true;
  vertex_attributes.tangent = true;
  vertex_attributes.color = true;
  vertex_attributes.tex_coord = true;
  mesh->SetVertices(vertex_attributes, vertices, triangles);

  if (!mesh->Export(path)) {
    EVOENGINE_ERROR("Pine mesh export failed!");
  }
}

void ScotsPine::ExportFlowGraph(YAML::Emitter& out) {
  out << YAML::Key << "Flows" << YAML::Value << YAML::BeginSeq;
  if (!growth_model.IsInitialized()) {
    out << YAML::EndSeq;
    return;
  }
  auto& graph = growth_model.graph;
  graph.SortLists();
  graph.CalculateFlows();
  const auto retained = CollectInternodeFlowHandles(graph);
  for (const auto flow_handle : graph.PeekSortedFlowList()) {
    if (retained.find(flow_handle) == retained.end())
      continue;
    const auto& flow = graph.PeekFlow(flow_handle);
    const auto parent_flow_handle = FindParentInternodeFlowHandle(graph, flow_handle, retained);
    out << YAML::BeginMap;
    out << YAML::Key << "I" << YAML::Value << flow_handle;
    out << YAML::Key << "PI" << YAML::Value << parent_flow_handle;
    out << YAML::Key << "SP" << YAML::Value << flow.info.global_start_position;
    out << YAML::Key << "SD" << YAML::Value << flow.info.global_start_rotation * glm::vec3(0, 0, -1);
    out << YAML::Key << "ST" << YAML::Value << flow.info.start_thickness;
    out << YAML::Key << "EP" << YAML::Value << flow.info.global_end_position;
    out << YAML::Key << "ED" << YAML::Value << flow.info.global_end_rotation * glm::vec3(0, 0, -1);
    out << YAML::Key << "ET" << YAML::Value << flow.info.end_thickness;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void ScotsPine::ExportFlowGraph(const std::filesystem::path& path) {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    ExportFlowGraph(out);
    out << YAML::EndMap;
    std::ofstream output_file(path.string());
    output_file << out.c_str();
    output_file.flush();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(std::string("Failed to save: ") + e.what());
  }
}

void ScotsPine::ExportNodeGraph(YAML::Emitter& out) {
  out << YAML::Key << "Nodes" << YAML::Value << YAML::BeginSeq;
  if (!growth_model.IsInitialized()) {
    out << YAML::EndSeq;
    return;
  }
  auto& graph = growth_model.graph;
  graph.SortLists();
  graph.CalculateFlows();
  const auto retained = CollectInternodeFlowHandles(graph);
  for (const auto node_handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(node_handle);
    if (!IsInternodeNode(node))
      continue;
    const auto parent_node_handle = FindParentInternodeNodeHandle(graph, node_handle);
    auto flow_handle = node.GetFlowHandle();
    while (flow_handle >= 0 && retained.find(flow_handle) == retained.end()) {
      flow_handle = graph.PeekFlow(flow_handle).GetParentHandle();
    }
    out << YAML::BeginMap;
    out << YAML::Key << "I" << YAML::Value << node_handle;
    out << YAML::Key << "PI" << YAML::Value << parent_node_handle;
    out << YAML::Key << "FI" << YAML::Value << flow_handle;
    out << YAML::Key << "SP" << YAML::Value << node.info.global_position;
    out << YAML::Key << "EP" << YAML::Value << node.info.GetGlobalEndPosition();
    out << YAML::Key << "D" << YAML::Value << node.info.GetGlobalDirection();
    out << YAML::Key << "T" << YAML::Value << node.info.thickness;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void ScotsPine::ExportNodeGraph(const std::filesystem::path& path) {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    ExportNodeGraph(out);
    out << YAML::EndMap;
    std::ofstream output_file(path.string());
    output_file << out.c_str();
    output_file.flush();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(std::string("Failed to save: ") + e.what());
  }
}

void ScotsPine::ExportNeedleSkeleton(YAML::Emitter& out) {
  const int segment_count = growth_model.IsInitialized() ? std::max(1, growth_model.sampled.needle_segment_count) : 1;

  out << YAML::Key << "NeedleSegmentCount" << YAML::Value << segment_count;
  out << YAML::Key << "NeedleStationCount" << YAML::Value << (segment_count + 1);
  out << YAML::Key << "Needles" << YAML::Value << YAML::BeginSeq;
  for (const auto& line : last_needle_skeleton_lines) {
    out << YAML::BeginMap;
    out << YAML::Key << "SI" << YAML::Value << line.sheath_node_handle;
    out << YAML::Key << "CI" << YAML::Value << line.cluster_node_handle;
    out << YAML::Key << "PI" << YAML::Value << line.parent_node_handle;
    out << YAML::Key << "NI" << YAML::Value << line.needle_index;
    out << YAML::Key << "IY" << YAML::Value << line.initiation_year_index;
    out << YAML::Key << "AgeY" << YAML::Value << line.age_years;
    out << YAML::Key << "LengthM" << YAML::Value << line.length_m;
    out << YAML::Key << "TargetLengthM" << YAML::Value << line.target_length_m;
    out << YAML::Key << "MaturationY" << YAML::Value << line.maturation_years;
    out << YAML::Key << "Mature" << YAML::Value << line.maturity_reached;
    out << YAML::Key << "Year0" << YAML::Value << line.year0_cohort;
    out << YAML::Key << "P" << YAML::Value << YAML::BeginSeq;
    for (const auto& point : line.points_world) {
      out << point;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
}

void ScotsPine::ExportNeedleSkeleton(const std::filesystem::path& path) {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    ExportNeedleSkeleton(out);
    out << YAML::EndMap;
    std::ofstream output_file(path.string());
    output_file << out.c_str();
    output_file.flush();
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(std::string("Failed to save: ") + e.what());
  }
}

void ScotsPine::WriteAnnotationSkeletonTreeJson(std::ostream& out, const int tree_index) {
  glm::mat4 world_from_tree(1.0f);
  if (const auto scene = GetScene()) {
    const auto owner = GetOwner();
    if (scene->IsEntityValid(owner)) {
      world_from_tree = scene->GetDataComponent<GlobalTransform>(owner).value;
    }
  }

  out << std::fixed << std::setprecision(9);
  out << "{";
  out << "\"tree_index\":" << tree_index << ",";
  out << "\"organs\":{";

  out << "\"needles\":[";
  for (size_t i = 0; i < last_needle_skeleton_lines.size(); ++i) {
    const auto& line = last_needle_skeleton_lines[i];
    if (i > 0) {
      out << ",";
    }
    const auto points =
        TransformAnnotationPoints(world_from_tree, ResampleAnnotationPolyline(line.points_world, kAnnotationNeedleSampleCount));
    out << "{";
    out << "\"tree_index\":" << tree_index << ",";
    out << "\"needle_index\":" << line.needle_index << ",";
    out << "\"sheath_node_handle\":" << line.sheath_node_handle << ",";
    out << "\"parent_internode_handle\":" << line.parent_node_handle << ",";
    out << "\"parent_internode_node_handle\":" << line.parent_node_handle << ",";
    out << "\"initiation_year\":" << line.initiation_year_index << ",";
    out << "\"age_years\":" << SafeAnnotationScalar(line.age_years) << ",";
    out << "\"length_m\":" << SafeAnnotationScalar(AnnotationPolylineLength(points)) << ",";
    out << "\"target_length_m\":" << SafeAnnotationScalar(line.target_length_m) << ",";
    out << "\"maturation_years\":" << SafeAnnotationScalar(line.maturation_years) << ",";
    out << "\"mature\":" << (line.maturity_reached ? "true" : "false") << ",";
    out << "\"points_world\":";
    WriteAnnotationPointsJson(out, points);
    out << "}";
  }
  out << "],";

  out << "\"sheaths\":[";
  bool first_sheath = true;
  if (growth_model.IsInitialized()) {
    auto& graph = growth_model.graph;
    graph.SortLists();
    graph.CalculateFlows();
    for (const auto sheath_handle : graph.PeekSortedNodeList()) {
      const auto& node = graph.PeekNode(sheath_handle);
      if (ResolvePineModuleRenderRole(node.data) != PineModuleRenderRole::SheathInstance) {
        continue;
      }
      const auto& sheath = node.data.template Get<PineNeedleSheath>();
      if (sheath.length <= 0.0f || sheath.width <= 0.0f) {
        continue;
      }
      const LNodeHandle parent_handle = FindParentInternodeNodeHandle(graph, sheath_handle);
      if (parent_handle < 0) {
        continue;
      }
      const auto& parent_node = graph.PeekNode(parent_handle);
      const float active_branching_angle_deg = ComputeActiveSheathBranchingAngleDeg(graph, sheath);
      const FascicleSheathFrame sheath_frame =
          ComputeFascicleSheathFrame(sheath, parent_node, active_branching_angle_deg);
      const glm::vec3 base = SafeAnnotationPoint(sheath_frame.base_position);
      const glm::vec3 tip = SafeAnnotationPoint(base + SafeAnnotationPoint(sheath_frame.direction) * sheath.length);
      const auto points = TransformAnnotationPoints(world_from_tree, {base, glm::mix(base, tip, 0.5f), tip});
      if (!first_sheath) {
        out << ",";
      }
      first_sheath = false;
      out << "{";
      out << "\"tree_index\":" << tree_index << ",";
      out << "\"sheath_node_handle\":" << sheath_handle << ",";
      out << "\"parent_internode_handle\":" << parent_handle << ",";
      out << "\"parent_internode_node_handle\":" << parent_handle << ",";
      out << "\"length_m\":" << SafeAnnotationScalar(AnnotationPolylineLength(points)) << ",";
      out << "\"points_world\":";
      WriteAnnotationPointsJson(out, points);
      out << "}";
    }
  }
  out << "],";

  out << "\"stems\":[";
  bool first_stem = true;
  if (growth_model.IsInitialized()) {
    auto& graph = growth_model.graph;
    graph.SortLists();
    graph.CalculateFlows();
    const auto retained = CollectInternodeFlowHandles(graph);
    for (const auto flow_handle : graph.PeekSortedFlowList()) {
      if (retained.find(flow_handle) == retained.end()) {
        continue;
      }
      const auto& flow = graph.PeekFlow(flow_handle);
      std::vector<glm::vec3> polyline;
      std::vector<LNodeHandle> node_handles;
      for (const auto node_handle : flow.PeekNodeHandles()) {
        const auto& node = graph.PeekNode(node_handle);
        if (!IsInternodeNode(node) || node.info.length <= 0.0f) {
          continue;
        }
        node_handles.emplace_back(node_handle);
        if (polyline.empty()) {
          polyline.emplace_back(node.info.global_position);
        }
        polyline.emplace_back(node.info.GetGlobalEndPosition());
      }
      if (polyline.empty()) {
        continue;
      }
      const auto points =
          TransformAnnotationPoints(world_from_tree, ResampleAnnotationPolyline(polyline, kAnnotationStemSampleCount));
      if (!first_stem) {
        out << ",";
      }
      first_stem = false;
      out << "{";
      out << "\"tree_index\":" << tree_index << ",";
      out << "\"flow_handle\":" << flow_handle << ",";
      out << "\"parent_flow_handle\":" << FindParentInternodeFlowHandle(graph, flow_handle, retained) << ",";
      out << "\"order\":" << flow.info.order << ",";
      out << "\"node_handles\":[";
      for (size_t i = 0; i < node_handles.size(); ++i) {
        if (i > 0) {
          out << ",";
        }
        out << node_handles[i];
      }
      out << "],";
      out << "\"length_m\":" << SafeAnnotationScalar(AnnotationPolylineLength(polyline)) << ",";
      out << "\"flow_length_m\":" << SafeAnnotationScalar(flow.info.flow_length) << ",";
      out << "\"points_world\":";
      WriteAnnotationPointsJson(out, points);
      out << "}";
    }
  }
  out << "]";
  out << "}}";
}

void ScotsPine::CollectAnnotationSkeletonPoints(std::vector<glm::vec3>& points) {
  glm::mat4 world_from_tree(1.0f);
  if (const auto scene = GetScene()) {
    const auto owner = GetOwner();
    if (scene->IsEntityValid(owner)) {
      world_from_tree = scene->GetDataComponent<GlobalTransform>(owner).value;
    }
  }

  for (const auto& line : last_needle_skeleton_lines) {
    AppendAnnotationPoints(points, TransformAnnotationPoints(
                                       world_from_tree,
                                       ResampleAnnotationPolyline(line.points_world, kAnnotationNeedleSampleCount)));
  }

  if (!growth_model.IsInitialized()) {
    return;
  }

  auto& graph = growth_model.graph;
  graph.SortLists();
  graph.CalculateFlows();
  for (const auto sheath_handle : graph.PeekSortedNodeList()) {
    const auto& node = graph.PeekNode(sheath_handle);
    if (ResolvePineModuleRenderRole(node.data) != PineModuleRenderRole::SheathInstance) {
      continue;
    }
    const auto& sheath = node.data.template Get<PineNeedleSheath>();
    if (sheath.length <= 0.0f || sheath.width <= 0.0f) {
      continue;
    }
    const LNodeHandle parent_handle = FindParentInternodeNodeHandle(graph, sheath_handle);
    if (parent_handle < 0) {
      continue;
    }
    const auto& parent_node = graph.PeekNode(parent_handle);
    const float active_branching_angle_deg = ComputeActiveSheathBranchingAngleDeg(graph, sheath);
    const FascicleSheathFrame sheath_frame =
        ComputeFascicleSheathFrame(sheath, parent_node, active_branching_angle_deg);
    const glm::vec3 base = SafeAnnotationPoint(sheath_frame.base_position);
    const glm::vec3 tip = SafeAnnotationPoint(base + SafeAnnotationPoint(sheath_frame.direction) * sheath.length);
    AppendAnnotationPoints(points, TransformAnnotationPoints(world_from_tree, {base, glm::mix(base, tip, 0.5f), tip}));
  }

  const auto retained = CollectInternodeFlowHandles(graph);
  for (const auto flow_handle : graph.PeekSortedFlowList()) {
    if (retained.find(flow_handle) == retained.end()) {
      continue;
    }
    const auto& flow = graph.PeekFlow(flow_handle);
    std::vector<glm::vec3> polyline;
    for (const auto node_handle : flow.PeekNodeHandles()) {
      const auto& node = graph.PeekNode(node_handle);
      if (!IsInternodeNode(node) || node.info.length <= 0.0f) {
        continue;
      }
      if (polyline.empty()) {
        polyline.emplace_back(node.info.global_position);
      }
      polyline.emplace_back(node.info.GetGlobalEndPosition());
    }
    if (!polyline.empty()) {
      AppendAnnotationPoints(points, TransformAnnotationPoints(world_from_tree,
                                                              ResampleAnnotationPolyline(polyline, kAnnotationStemSampleCount)));
    }
  }
}

// ===========================================================================
// Component lifecycle / inspector
// ===========================================================================

void ScotsPine::OnDestroy() {
  ClearGeometryEntities();
  render_target_.reset();
}

bool l_system_package::InspectScotsPine(InspectorContext& context, ScotsPine& pine) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;

  if (editor_layer->DragAndDropButton<ScotsPineDescriptor>(pine.descriptor_ref, "Descriptor"))
    changed = true;

  int seed_int = static_cast<int>(pine.seed);
  if (ImGui::DragInt("Seed", &seed_int, 1, 0, 999999)) {
    pine.seed = static_cast<unsigned int>(seed_int);
    changed = true;
  }

  if (ImGui::DragFloat("Target GDD", &pine.target_gdd, 1.0f, 0.0f, 200000.0f, "%.1f"))
    changed = true;

  if (ImGui::Button("Generate")) {
    pine.GenerateGeometryEntities();
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear")) {
    pine.ClearGeometryEntities();
    changed = true;
  }

  if (pine.growth_model.IsInitialized()) {
    ImGui::Separator();
    ImGui::Text("GDD: %.1f", pine.growth_model.accumulated_gdd);
    ImGui::Text("Topology: %s", pine.growth_model.IsTopologyComplete() ? "Complete" : "Pending");
    const auto& sorted = pine.growth_model.graph.PeekSortedNodeList();
    ImGui::Text("Nodes: %d", static_cast<int>(sorted.size()));
    ImGui::Text("Internodes: %u   Needles: %u", pine.last_internode_count, pine.last_needle_count);
    ImGui::Text("Sheaths: %u", pine.last_sheath_count);
    ImGui::Text("Render Snapshot Version: %llu", static_cast<unsigned long long>(pine.last_render_snapshot_version));
    ImGui::Text("Raytrace Internodes: %s (%u instances)",
                pine.last_raytrace_internodes_ready ? "Ready" : "Not Ready",
                pine.last_raytrace_internode_instances);
    ImGui::Text("Raytrace Sheaths: %s (%u instances)", pine.last_raytrace_sheaths_ready ? "Ready" : "Not Ready",
                pine.last_raytrace_sheath_instances);
    ImGui::Text("Raytrace Needles: %s (%u segments, %u points)",
                pine.last_raytrace_needles_ready ? "Ready" : "Not Ready", pine.last_raytrace_needle_segments,
                pine.last_raytrace_needle_points);
  }

  return changed;
}

void l_system_package::SerializeScotsPine(YAML::Emitter& out, const ScotsPine& target) {
  target.descriptor_ref.Save("descriptor_ref", out);
  out << YAML::Key << "seed" << YAML::Value << target.seed;
  out << YAML::Key << "target_gdd" << YAML::Value << target.target_gdd;
}

void l_system_package::DeserializeScotsPine(const YAML::Node& in, ScotsPine& target) {
  target.descriptor_ref.Load("descriptor_ref", in);
  if (in["seed"])
    target.seed = in["seed"].as<unsigned int>();
  if (in["target_gdd"]) {
    target.target_gdd = in["target_gdd"].as<float>();
  }
}

void ScotsPine::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(descriptor_ref);
}
