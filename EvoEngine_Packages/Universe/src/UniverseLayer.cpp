#include "UniverseLayer.hpp"

#include <chrono>
#include "Application.hpp"
#include "AssetManager.hpp"
#include "ComputePipeline.hpp"
#include "EditorLayer.hpp"
#include "GpuProfiler.hpp"
#include "GraphicsPipeline.hpp"
#include "PostProcessingStack.hpp"
#include "ProjectManager.hpp"
#include "RenderTexture.hpp"
#include "Shader.hpp"
#include "StarHover.hpp"
#include "Times.hpp"
#include "UniverseProfiler.hpp"
#include "WindowLayer.hpp"

using namespace universe_package;

namespace {
constexpr uint32_t kWorkgroupSize = 256;

uint64_t SplitMix64(uint64_t value) {
  value += 0x9e3779b97f4a7c15ull;
  value = (value ^ (value >> 30u)) * 0xbf58476d1ce4e5b9ull;
  value = (value ^ (value >> 27u)) * 0x94d049bb133111ebull;
  return value ^ (value >> 31u);
}

double UnitSample(const uint64_t seed, const uint64_t id, const uint64_t stream) {
  const uint64_t bits = SplitMix64(seed ^ SplitMix64(id) ^ SplitMix64(stream));
  return static_cast<double>(bits >> 11u) * (1.0 / 9007199254740992.0);
}

std::pair<double, double> GaussianPair(const uint64_t seed, const uint64_t id, const uint64_t stream) {
  const double u1 = (std::max)(UnitSample(seed, id, stream), std::numeric_limits<double>::min());
  const double u2 = UnitSample(seed, id, stream + 1u);
  const double radius = std::sqrt(-2.0 * std::log(u1));
  const double angle = glm::two_pi<double>() * u2;
  return {radius * std::cos(angle), radius * std::sin(angle)};
}

uint64_t BucketIndex(const uint64_t seed, const uint64_t ordinal, const uint64_t stream, const uint64_t count) {
  uint64_t value = SplitMix64(seed ^ SplitMix64(ordinal) ^ SplitMix64(stream));
  const uint64_t threshold = -count % count;
  while (value < threshold)
    value = SplitMix64(value);
  return value % count;
}

std::shared_ptr<Buffer> CreateDeviceBuffer(const VkDeviceSize size, const VkBufferUsageFlags usage) {
  VkBufferCreateInfo create_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  create_info.size = (std::max)(size, VkDeviceSize{1});
  create_info.usage = usage;
  create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  return std::make_shared<Buffer>(create_info, allocation_info);
}

struct StarRenderPushConstant {
  int32_t camera_index = 0;
  float subpixel_brightness_limit = 1.0f;
  glm::vec2 viewport_size = glm::vec2(1.0f);
  float fade_strength = 1.0f;
  glm::vec2 distance_compression{};
};
static_assert(sizeof(StarRenderPushConstant) == 28);

bool IsProceduralGalaxyProjectPath() {
  const auto project_path = ProjectManager::GetProjectPath();
  return project_path.filename() == "ProceduralGalaxy.eveproj" && project_path.parent_path().filename() == "Universe" &&
         project_path.parent_path().parent_path().filename() == "EvoEngine-DemoProjects";
}

void CheckLod(std::mutex& mutex, const std::shared_ptr<TerrainChunk>& chunk, const PlanetInfo& info,
              const GlobalTransform& planet_transform, const GlobalTransform& camera_transform) {
  if (glm::distance(glm::dvec3(chunk->ChunkCenterPosition(planet_transform.GetPosition(), info.radius,
                                                          planet_transform.GetRotation())),
                    glm::dvec3(camera_transform.GetPosition())) <
      info.lod_distance * info.radius / glm::pow(2, chunk->detail_level + 1)) {
    if (chunk->detail_level < info.max_lod_level)
      chunk->Expand(mutex);
  }
  if (chunk->c0)
    CheckLod(mutex, chunk->c0, info, planet_transform, camera_transform);
  if (chunk->c1)
    CheckLod(mutex, chunk->c1, info, planet_transform, camera_transform);
  if (chunk->c2)
    CheckLod(mutex, chunk->c2, info, planet_transform, camera_transform);
  if (chunk->c3)
    CheckLod(mutex, chunk->c3, info, planet_transform, camera_transform);
  if (glm::distance(glm::dvec3(chunk->ChunkCenterPosition(planet_transform.GetPosition(), info.radius,
                                                          planet_transform.GetRotation())),
                    glm::dvec3(camera_transform.GetPosition())) >
      info.lod_distance * info.radius / glm::pow(2, chunk->detail_level + 1))
    chunk->Collapse();
}

void RenderChunk(const std::shared_ptr<TerrainChunk>& chunk, const std::shared_ptr<Material>& material,
                 const GlobalTransform& transform) {
  if (chunk->active)
    ApplicationContext::Get().GetLayer<RenderLayer>()->DrawMesh(chunk->mesh, material, transform, true);
  if (chunk->children_active) {
    RenderChunk(chunk->c0, material, transform);
    RenderChunk(chunk->c1, material, transform);
    RenderChunk(chunk->c2, material, transform);
    RenderChunk(chunk->c3, material, transform);
  }
}
}  // namespace

StarBaseSample universe_package::GenerateStarBaseSample(const uint64_t seed, const uint32_t ordinal) {
  const uint64_t id = static_cast<uint64_t>(ordinal) + 1;
  const auto xy = GaussianPair(seed, id, 1u);
  const auto z = GaussianPair(seed, id, 3u);
  return {UnitSample(seed, id, 0u), xy.first, xy.second, z.first, z.second};
}

bool StarOrbitLayout::Update(const StarCluster& cluster) {
  const std::array<double, 7> next{cluster.star_minimum_distance, cluster.center_diameter,   cluster.disk_diameter,
                                   cluster.center_eccentricity,   cluster.disk_eccentricity, cluster.core_proportion,
                                   cluster.core_eccentricity};
  if (revision && std::memcmp(next.data(), geometry.data(), sizeof(next)) == 0)
    return false;
  geometry = next;
  ++revision;
  orbits.clear();
  capacity = 0;
  radial_spacing = 0;
  status =
      "Invalid orbit settings: require finite positive spacing/diameters, disk >= center and eccentricities/core "
      "proportion in (0,1).";
  for (const double value : next)
    if (!std::isfinite(value))
      return true;
  if (next[0] <= 0 || next[1] <= 0 || next[2] < next[1])
    return true;
  for (size_t i = 3; i < next.size(); ++i)
    if (next[i] <= 0 || next[i] >= 1)
      return true;
  const double gaps = std::floor(((next[2] - next[1]) / 40) / next[0]);
  // Bound edit-time table memory; never silently change the requested spacing.
  if (gaps >= 32768) {
    status = "Orbit layout exceeds 32768 orbits; increase minimum distance.";
    return true;
  }
  const uint32_t gap_count = static_cast<uint32_t>(gaps);
  radial_spacing = gap_count ? (next[2] - next[1]) / (40 * gap_count) : 0;
  const auto p = BuildStarClusterParameters(cluster, glm::dmat4(1), 0);
  orbits.reserve(gap_count + 1);
  constexpr uint32_t segments = 2048;
  for (uint32_t i = 0; i <= gap_count; ++i) {
    StarOrbit orbit;
    orbit.proportion = gap_count ? double(i) / gap_count : 0;
    const bool disk = orbit.proportion > p.ellipse1.z;
    const double t = disk ? (orbit.proportion - p.ellipse1.z) / (1 - p.ellipse1.z) : orbit.proportion / p.ellipse1.z;
    const double a = (disk ? glm::mix(p.ellipse0.z, p.ellipse0.x, t) : glm::mix(p.ellipse1.x, p.ellipse0.z, t)) / 20;
    const double b = (disk ? glm::mix(p.ellipse0.w, p.ellipse0.y, t) : glm::mix(p.ellipse1.y, p.ellipse0.w, t)) / 20;
    if (a == b) {
      orbit.length = glm::two_pi<double>() * a;
    } else {
      orbit.arc_lengths.resize(segments + 1);
      glm::dvec2 previous(0, b);
      for (uint32_t j = 1; j <= segments; ++j) {
        const double angle = glm::two_pi<double>() * j / segments;
        const glm::dvec2 point(a * std::sin(angle), b * std::cos(angle));
        orbit.arc_lengths[j] = orbit.arc_lengths[j - 1] + glm::distance(previous, point);
        previous = point;
      }
      orbit.length = orbit.arc_lengths.back();
    }
    const double slots = std::floor(orbit.length / next[0]);
    if (!std::isfinite(slots) || slots > UINT32_MAX) {
      orbits.clear();
      capacity = 0;
      status = "Orbit slot count exceeds 32-bit capacity; increase minimum distance.";
      return true;
    }
    orbit.capacity = static_cast<uint64_t>(slots);
    capacity += orbit.capacity;
    orbits.push_back(std::move(orbit));
  }
  status = capacity ? "Orbit buckets ready" : "No orbit has enough circumference for one slot";
  return true;
}

std::vector<StarBaseSample> StarOrbitLayout::Allocate(const uint64_t seed, const uint32_t count) {
  std::vector<uint32_t> available;
  std::vector<std::unordered_map<uint64_t, uint64_t>> slots(orbits.size());
  for (uint32_t i = 0; i < orbits.size(); ++i) {
    orbits[i].occupied = 0;
    if (orbits[i].capacity)
      available.push_back(i);
  }
  std::vector<StarBaseSample> result;
  const uint32_t active = static_cast<uint32_t>((std::min)(uint64_t(count), capacity));
  result.reserve(active);
  // Replay only on population edits: deterministic prefixes survive shrink/regrowth without retaining unused slots.
  for (uint32_t ordinal = 0; ordinal < active; ++ordinal) {
    const auto orbit_index = BucketIndex(seed, ordinal, 100, available.size());
    const auto index = available[orbit_index];
    auto& orbit = orbits[index];
    auto& remaining_slots = slots[index];
    const uint64_t remaining = orbit.capacity - orbit.occupied;
    const auto selected = BucketIndex(seed, ordinal, 101, remaining);
    const auto lookup = [&](const uint64_t slot) {
      const auto found = remaining_slots.find(slot);
      return found == remaining_slots.end() ? slot : found->second;
    };
    const uint64_t slot = lookup(selected), last = lookup(remaining - 1);
    remaining_slots[selected] = last;
    remaining_slots.erase(remaining - 1);
    if (++orbit.occupied == orbit.capacity) {
      available[orbit_index] = available.back();
      available.pop_back();
    }
    const double fraction = (slot + UnitSample(seed, ordinal, 102)) / orbit.capacity;
    auto sample = GenerateStarBaseSample(seed, ordinal);
    sample.orbital_proportion = orbit.proportion;
    sample.orbital_phase = fraction * glm::two_pi<double>();
    if (!orbit.arc_lengths.empty()) {
      const double arc = fraction * orbit.length;
      const auto end = std::upper_bound(orbit.arc_lengths.begin(), orbit.arc_lengths.end(), arc);
      const size_t upper = (std::min)(size_t(end - orbit.arc_lengths.begin()), orbit.arc_lengths.size() - 1);
      const double t = (arc - orbit.arc_lengths[upper - 1]) / (orbit.arc_lengths[upper] - orbit.arc_lengths[upper - 1]);
      sample.orbital_phase = (upper - 1 + t) * glm::two_pi<double>() / (orbit.arc_lengths.size() - 1);
    }
    result.push_back(sample);
  }
  return result;
}

StarClusterGpuParameters universe_package::BuildStarClusterParameters(const StarCluster& cluster,
                                                                      const glm::dmat4& world_transform,
                                                                      const double simulation_time) {
  const double disk_a = cluster.disk_diameter * cluster.disk_eccentricity;
  const double disk_b = cluster.disk_diameter * (1.0 - cluster.disk_eccentricity);
  const double center_a = cluster.center_diameter * cluster.center_eccentricity;
  const double center_b = cluster.center_diameter * (1.0 - cluster.center_eccentricity);
  const double core_diameter =
      cluster.center_diameter + (disk_a + disk_b - cluster.center_diameter) * cluster.core_proportion;
  const double core_a = core_diameter * cluster.core_eccentricity;
  const double core_b = core_diameter * (1.0 - cluster.core_eccentricity);
  StarClusterGpuParameters result{};

  result.star_count = cluster.GetStarCount();
  result.ellipse0 = {disk_a, disk_b, core_a, core_b};
  result.ellipse1 = {center_a, center_b, cluster.core_proportion, cluster.twist};
  result.spread_speed = {cluster.y_spread, cluster.xz_spread, cluster.disk_speed, cluster.core_speed};
  result.speed_tilt = {cluster.center_speed, cluster.disk_tilt_x, cluster.disk_tilt_z, cluster.core_tilt_x};
  result.tilt_radius = {cluster.core_tilt_z, cluster.center_tilt_x, cluster.center_tilt_z, 0.0};
  result.center_offset = glm::dvec4(cluster.center_offset, (std::max)(0.0, cluster.radius_deviation));
  result.center_position = glm::dvec4(cluster.center_position, 0.0);
  result.world0 = world_transform[0];
  result.world1 = world_transform[1];
  result.world2 = world_transform[2];
  result.world3 = world_transform[3];
  result.disk_color_intensity = glm::vec4(cluster.disk_color, cluster.disk_emission_intensity);
  result.core_color_intensity = glm::vec4(cluster.core_color, cluster.core_emission_intensity);
  result.center_color_intensity = glm::vec4(cluster.center_color, cluster.center_emission_intensity);
  const double minimum_radius = (std::max)(0.0, cluster.radius_min);
  result.time_padding = {simulation_time, static_cast<double>(cluster.alpha), minimum_radius,
                         (std::max)(minimum_radius, cluster.radius_max)};
  return result;
}

void universe_package::ApplyStarRadiusScale(StarClusterGpuParameters& parameters, const double scale) {
  parameters.time_padding.z *= scale;
  parameters.time_padding.w *= scale;
}

void universe_package::ApplyStarDisplayFrame(StarClusterGpuParameters& parameters, const glm::dmat4& display_transform,
                                             const double radius_scale) {
  ApplyStarRadiusScale(parameters, radius_scale);
  const auto transform =
      display_transform * glm::dmat4(parameters.world0, parameters.world1, parameters.world2, parameters.world3);
  parameters.world0 = transform[0];
  parameters.world1 = transform[1];
  parameters.world2 = transform[2];
  parameters.world3 = transform[3];
}

glm::vec2 universe_package::StarDistanceCompression(const float far_distance) {
  if (!std::isfinite(far_distance) || far_distance <= 0)
    return {};
  const glm::vec2 result{far_distance * 0.7f, far_distance * 0.99f};
  return result.y > result.x ? result : glm::vec2{};
}

double universe_package::CompressStarDistance(const double distance, const double start, const double limit) {
  if (!std::isfinite(distance) || !std::isfinite(start) || !std::isfinite(limit) || distance <= start || start < 0 ||
      limit <= start)
    return distance;
  return (std::min)(start + (limit - start) * (1 - std::exp(-(distance - start) / (limit - start))),
                    std::nextafter(limit, start));
}

bool StarClusterBatch::Update(const std::vector<StarClusterInput>& inputs, const double global_time,
                              const uint64_t maximum_stars) {
  std::vector<StarClusterRange> next_ranges;
  std::unordered_set<const StarCluster*> live;
  parameters.clear();
  uint64_t total = 0;
  for (const auto& input : inputs) {
    const auto& cluster = *input.cluster;
    const auto handle = cluster.GetHandle().GetValue();
    live.insert(input.cluster.get());
    auto& clock = clocks[input.cluster.get()];
    if (clock.component.lock() != input.cluster || clock.component_handle != handle) {
      clock = {input.cluster, handle, next_identity++, 0.0, global_time};
    } else {
      if (!cluster.paused)
        clock.elapsed += (global_time - clock.last_global_time) * cluster.time_scale;
      clock.last_global_time = global_time;
    }
    clock.layout.Update(cluster);
    const auto count = static_cast<uint32_t>((std::min)(uint64_t(cluster.GetStarCount()), clock.layout.capacity));
    clock.active_count = input.enabled ? count : 0;
    if (!input.enabled || count == 0)
      continue;
    if (total + count > (std::min)(maximum_stars, uint64_t(UINT32_MAX)))
      throw std::length_error("Active star batch exceeds GPU storage or draw limits; reduce star count.");
    next_ranges.push_back({clock.identity, cluster.seed, static_cast<uint32_t>(total), count, clock.layout.revision});
    total += count;
    parameters.emplace_back(BuildStarClusterParameters(cluster, input.world_transform, cluster.phase + clock.elapsed));
    parameters.back().star_count = count;
  }
  for (auto it = clocks.begin(); it != clocks.end();) {
    if (live.find(it->first) == live.end())
      it = clocks.erase(it);
    else
      ++it;
  }
  const bool changed = ranges != next_ranges;
  if (changed) {
    std::vector<StarBaseSample> next_samples;
    next_samples.reserve(static_cast<size_t>(total));
    for (const auto& range : next_ranges) {
      const auto previous = std::find_if(ranges.begin(), ranges.end(), [&](const auto& old_range) {
        return old_range.identity == range.identity && old_range.seed == range.seed;
      });
      if (previous != ranges.end() && previous->count == range.count &&
          previous->layout_revision == range.layout_revision) {
        next_samples.insert(next_samples.end(), samples.begin() + previous->offset,
                            samples.begin() + previous->offset + range.count);
      } else {
        auto clock = std::find_if(clocks.begin(), clocks.end(), [&](const auto& item) {
          return item.second.identity == range.identity;
        });
        const auto allocated = clock->second.layout.Allocate(range.seed, range.count);
        next_samples.insert(next_samples.end(), allocated.begin(), allocated.end());
      }
    }
    samples = std::move(next_samples);
    ranges = std::move(next_ranges);
    gaussian_bounds.assign(ranges.size(), glm::dvec3(0));
    for (size_t i = 0; i < ranges.size(); ++i)
      for (uint32_t j = 0; j < ranges[i].count; ++j) {
        const auto& sample = samples[ranges[i].offset + j];
        gaussian_bounds[i] =
            glm::max(gaussian_bounds[i], glm::abs(glm::dvec3(sample.gaussian_x, sample.gaussian_y, sample.gaussian_z)));
      }
    ++population_revision;
  }
  for (auto& parameter : parameters)
    parameter.population_revision = population_revision;
  return changed;
}

std::vector<double> universe_package::SelectStarOrbitProportions(const StarOrbitLayout& layout,
                                                                 const StarOrbitDisplay mode,
                                                                 const StarPickSnapshot& selected,
                                                                 const StarClusterRange& range,
                                                                 const std::vector<StarBaseSample>& samples) {
  if (!range.count)
    return {};
  if (mode == StarOrbitDisplay::Selected) {
    if (!selected.result.valid || selected.cluster.expired() || selected.identity != range.identity ||
        selected.seed != range.seed || selected.ordinal >= range.count)
      return {};
    return {samples[range.offset + selected.ordinal].orbital_proportion};
  }
  std::vector<double> result;
  for (const auto& orbit : layout.orbits)
    if (mode == StarOrbitDisplay::All || orbit.occupied)
      result.push_back(orbit.proportion);
  return result;
}

bool StarOrbitStrandCache::Update(const StarClusterGpuParameters& parameters, const std::vector<double>& orbits,
                                  const float radius) {
  auto next = parameters;
  next.population_revision = 0;
  next.star_count = 0;
  next.spread_speed = {};
  next.speed_tilt.x = 0;
  next.tilt_radius.w = radius;
  next.center_offset.w = 0;
  next.world0 = {1, 0, 0, 0};
  next.world1 = {0, 1, 0, 0};
  next.world2 = {0, 0, 1, 0};
  next.world3 = {0, 0, 0, 1};
  next.disk_color_intensity.w = next.core_color_intensity.w = next.center_color_intensity.w = 0;
  next.time_padding = {};
  if (proportions == orbits && std::memcmp(&key, &next, sizeof(key)) == 0)
    return false;
  key = next;
  proportions = orbits;
  points.clear();
  starts.clear();
  constexpr uint32_t segments = 256;
  points.reserve(orbits.size() * (segments + 3));
  for (const double proportion : orbits) {
    starts.push_back(static_cast<uint32_t>(points.size()));
    StarBaseSample sample{};
    sample.orbital_proportion = proportion;
    const auto point_at = [&](const double phase) {
      sample.orbital_phase = phase;
      return glm::dvec3(EvaluateStar(key, sample).world_position_radius);
    };
    const auto a = point_at(0), b = point_at(glm::half_pi<double>()), c = point_at(glm::pi<double>());
    const glm::vec3 normal(glm::normalize(glm::cross(b - a, c - a)));
    for (uint32_t j = 0; j < segments + 3; ++j) {
      StrandPoint point;
      point.position = glm::vec3(point_at(glm::two_pi<double>() * ((j + segments - 1) % segments) / segments));
      point.normal = normal;
      point.thickness = radius;
      point.tex_coord = float(j) / segments;
      point.color = glm::vec4(glm::vec3(EvaluateStar(key, sample).color_emission), 1);
      points.push_back(point);
    }
  }
  starts.push_back(static_cast<uint32_t>(points.size()));
  return true;
}

void UniverseLayer::RenderOrbitStrands(const std::vector<StarClusterGpuParameters>& parameters) {
  const ProfilerScope profiler_scope("Universe Orbit Strands", "Universe");
  displayed_orbits_ = orbit_draws_ = 0;
  std::unordered_set<uint64_t> live;
  for (const auto& [component, clock] : batch_.clocks)
    live.insert(clock.identity);
  for (auto it = orbit_strands_.begin(); it != orbit_strands_.end();) {
    if (live.find(it->first) == live.end())
      it = orbit_strands_.erase(it);
    else
      ++it;
  }
  if (!show_orbit_strands) {
    orbit_status_ = "Disabled";
    return;
  }
  if (!Platform::MeshShaderEnabled()) {
    orbit_status_ = "Native orbit strands require mesh shader support and enablement.";
    return;
  }
  if (!std::isfinite(orbit_strand_radius) || orbit_strand_radius <= 0) {
    orbit_status_ = "Orbit strand radius must be finite and positive.";
    return;
  }
  const auto start = std::chrono::steady_clock::now();
  std::vector<StarOrbitStrandCache*> changed;
  std::vector<std::pair<StarOrbitStrandCache*, GlobalTransform>> draws;
  for (size_t i = 0; i < batch_.ranges.size(); ++i) {
    const auto& range = batch_.ranges[i];
    const auto clock = std::find_if(batch_.clocks.begin(), batch_.clocks.end(), [&](const auto& item) {
      return item.second.identity == range.identity;
    });
    const auto orbits = SelectStarOrbitProportions(clock->second.layout, orbit_display, star_picker_.state.selected,
                                                   range, batch_.samples);
    if (orbits.empty())
      continue;
    auto& cache = orbit_strands_[range.identity];
    if (cache.Update(parameters[i], orbits, orbit_strand_radius) || !cache.asset)
      changed.push_back(&cache);
    GlobalTransform model;
    const auto& p = parameters[i];
    model.value = glm::mat4(glm::dmat4(p.world0, p.world1, p.world2, p.world3));
    draws.emplace_back(&cache, model);
    displayed_orbits_ += orbits.size();
  }
  if (!changed.empty()) {
    orbit_rebuild_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    const auto upload_start = std::chrono::steady_clock::now();
    Platform::WaitForFrameSubmissions("Publish Universe orbit strands");
    for (auto* cache : changed) {
      if (!cache->asset)
        cache->asset = AssetManager::CreateTemporaryAsset<Strands>();
      StrandPointAttributes attributes;
      attributes.normal = attributes.color = true;
      cache->asset->SetStrands(attributes, cache->starts, cache->points);
    }
    GeometryStorage::WaitForPendingUploads();
    orbit_upload_ms_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - upload_start).count();
  }
  if (!draws.empty() && !orbit_material_) {
    orbit_material_ = AssetManager::CreateTemporaryAsset<Material>();
    auto data = orbit_material_->BuildGltfMaterialData();
    data.shade_material.unlit = 1;
    data.shade_material.pbr_base_color_factor = glm::vec4(1);
    orbit_material_->SetGltfMaterialData(data);
  }
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  for (const auto& [cache, model] : draws)
    orbit_draws_ += render_layer->DrawStrands(cache->asset, orbit_material_, model, false);
  orbit_status_ = draws.empty()                  ? "No matching active orbits"
                  : orbit_draws_ == draws.size() ? "Native strands submitted"
                                                 : "Waiting for strand geometry publication";
}

void UniverseLayer::RegisterTypes(Application&) {
}

bool universe_package::InspectUniverseLayer(InspectorContext&, UniverseLayer& layer) {
  const auto window_title = layer.GetLayerName();
  bool open = layer.enable_inspection;
  if (!ImGui::Begin(window_title.c_str(), &open)) {
    ImGui::End();
    layer.enable_inspection = open;
    return false;
  }
  ImGui::Text("Global simulation time: %.3f", layer.global_simulation_time_);
  ImGui::Text("Frame: %llu", static_cast<unsigned long long>(layer.frame_number_));
  ImGui::Text("FP64 compute: %s", layer.fp64_supported_ ? "supported" : "unavailable");
  ImGui::Text("Compute pipeline: %s", layer.compute_ready_ ? "ready" : "unavailable");
  ImGui::Text("Forward pipeline: %s", layer.render_ready_ ? "ready" : "unavailable");
  ImGui::Text("Registered clusters: %u, stars: %u", layer.registered_render_cluster_count_,
              layer.registered_render_star_count_);
  ImGui::Checkbox("Write star depth", &layer.depth_write);
  ImGui::Checkbox("Show orbit strands", &layer.show_orbit_strands);
  if (layer.show_orbit_strands) {
    int mode = static_cast<int>(layer.orbit_display);
    if (ImGui::Combo("Orbit display", &mode, "All\0Occupied\0Selected\0"))
      layer.orbit_display = static_cast<StarOrbitDisplay>(mode);
    ImGui::DragFloat("Orbit strand radius", &layer.orbit_strand_radius, 0.01f, 0.001f, 10000, "%.3f",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::Text("Orbit strands: %zu loops, %zu segments, %u cluster submissions", layer.displayed_orbits_,
                layer.displayed_orbits_ * 256, layer.orbit_draws_);
    ImGui::Text("Last rebuild/upload: %.3f / %.3f ms", layer.orbit_rebuild_ms_, layer.orbit_upload_ms_);
    ImGui::TextWrapped("%s", layer.orbit_status_.c_str());
  }
  ImGui::SliderFloat("Star-view fade strength", &layer.star_fade_strength, 0.0f, 4.0f, "%.2f",
                     ImGuiSliderFlags_AlwaysClamp);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip(
        "Locked-view target for stars enlarged to one pixel. Galaxy view uses 0. The value transitions with the view. "
        "1: area compensation; higher: stronger fade. Bloom suppression remains enabled.");
  ImGui::TextUnformatted("Star compression: 70%-99% of each camera far distance");
  const double coordinate_scale = layer.star_follow_.following ? 1.0 : kGalaxyDisplayScale;
  ImGui::Text("Universe coordinate scale: %.3f", coordinate_scale);
  if (layer.batch_.parameters.size() == 1)
    ImGui::Text("Effective displayed disk diameter: %.1f",
                (layer.batch_.parameters[0].ellipse0.x + layer.batch_.parameters[0].ellipse0.y) * coordinate_scale);
  ImGui::Text("Capacity: %zu stars, %zu clusters", layer.star_capacity_, layer.cluster_capacity_);
  ImGui::Text("Population / computed / rendered revision: %llu / %llu / %llu",
              static_cast<unsigned long long>(layer.batch_.population_revision),
              static_cast<unsigned long long>(layer.computed_revision_),
              static_cast<unsigned long long>(layer.rendered_revision_));
  ImGui::Text("Render slot: %u; draws across cameras this frame: %u", layer.render_slot_, layer.draws_this_frame_);
  if (ImGui::TreeNode("Packed cluster ranges")) {
    for (const auto& range : layer.batch_.ranges)
      ImGui::Text("%llu: offset %u, count %u", static_cast<unsigned long long>(range.identity), range.offset,
                  range.count);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Orbit buckets")) {
    for (const auto& [component, clock] : layer.batch_.clocks) {
      const auto cluster = clock.component.lock();
      if (!cluster)
        continue;
      ImGui::PushID(component);
      if (ImGui::TreeNode("Cluster", "Cluster %llu", static_cast<unsigned long long>(clock.identity))) {
        const auto& layout = clock.layout;
        ImGui::Text("Requested %u, active %u; %zu orbits, %llu slots", cluster->GetStarCount(), clock.active_count,
                    layout.orbits.size(), static_cast<unsigned long long>(layout.capacity));
        ImGui::Text("Nominal radial spacing %.6f; layout revision %llu", layout.radial_spacing,
                    static_cast<unsigned long long>(layout.revision));
        ImGui::TextWrapped("%s", layout.status.c_str());
        if (cluster->GetStarCount() > layout.capacity)
          ImGui::TextUnformatted("Capacity limit: excess requested stars are not simulated or rendered.");
        if (ImGui::TreeNode("Orbit occupancy")) {
          ImGuiListClipper clipper;
          clipper.Begin(static_cast<int>(layout.orbits.size()));
          while (clipper.Step())
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
              const auto& orbit = layout.orbits[i];
              ImGui::Text("%d: proportion %.6f, occupied %u / %llu", i, orbit.proportion,
                          clock.active_count ? orbit.occupied : 0, static_cast<unsigned long long>(orbit.capacity));
            }
          ImGui::TreePop();
        }
        ImGui::TreePop();
      }
      ImGui::PopID();
    }
    ImGui::TreePop();
  }
  ImGui::TextUnformatted("One population draw per camera, plus an optional hover ring. Picking reads back 48 bytes.");
  if (ImGui::TreeNodeEx("Star picking", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Active: %s%s", layer.pick_camera_name_.c_str(),
                layer.star_picker_.state.current.valid ? "" : " (inactive cursor)");
    ImGui::DragFloat("Minimum picking radius (display pixels)", &layer.pick_minimum_radius_, 0.1f, 0.0f, 20.0f, "%.1f",
                     ImGuiSliderFlags_AlwaysClamp);
    const auto show_hit = [&](const char* label, const StarPickSnapshot& hit) {
      if (!hit.result.valid) {
        ImGui::Text("%s: none", label);
        return;
      }
      const auto cluster = hit.cluster.lock();
      const auto scene = layer.GetScene();
      const std::string name = cluster && scene && scene->IsEntityValid(cluster->GetOwner())
                                   ? scene->GetEntityName(cluster->GetOwner())
                                   : "Deleted cluster";
      ImGui::Text("%s: %s / star %u (cluster %llu)", label, name.c_str(), hit.ordinal,
                  static_cast<unsigned long long>(hit.identity));
      ImGui::Text("Sampled render-space center: %.6f, %.6f, %.6f; ray distance: %.6f", hit.result.position_radius.x,
                  hit.result.position_radius.y, hit.result.position_radius.z, hit.result.distance);
      ImGui::Text("Sample age: %llu frames", static_cast<unsigned long long>(layer.frame_number_ - hit.frame));
    };
    show_hit("Hover", layer.star_picker_.state.hovered);
    show_hit("Selected", layer.star_picker_.state.selected);
    ImGui::Text("Readback: %s; click: %s", layer.star_picker_.Pending() ? "pending" : "idle",
                layer.star_picker_.PendingClick() ? "pending" : "idle");
    ImGui::Text("Selection: %s; following: %s (Space toggles)",
                layer.star_follow_.available ? "available" : "unavailable",
                layer.star_follow_.following ? "yes" : "no");
    ImGui::TextWrapped("%s", layer.star_follow_.status.c_str());
    ImGui::Text("View radius scale: %.3f -> %.0f; fade: %.3f -> %.2f", layer.star_view_.radius_scale,
                layer.star_view_.target_scale, layer.star_view_.fade_strength, layer.star_view_.target_fade_strength);
    if (layer.star_follow_.following)
      ImGui::TextUnformatted("Star-local view: hover and selection locked; Space exits.");
    if (layer.star_follow_.available) {
      const auto& position = layer.star_follow_.selected_world_position;
      ImGui::Text("CPU world position: %.6f, %.6f, %.6f", position.x, position.y, position.z);
      const auto& frame = layer.star_follow_.selected_frame;
      const auto rotation = glm::degrees(glm::eulerAngles(glm::quat_cast(glm::dmat3(frame))));
      ImGui::Text("Reference origin: %.6f, %.6f, %.6f", frame[3].x, frame[3].y, frame[3].z);
      ImGui::Text("Reference rotation (degrees): %.3f, %.3f, %.3f", rotation.x, rotation.y, rotation.z);
    }
    ImGui::TextWrapped("%s", layer.star_picker_.status.c_str());
    if (layer.pick_benchmark_)
      ImGui::TextUnformatted("Benchmark cursor override: center of active camera");
    ImGui::TreePop();
  }
  ImGui::TextWrapped("GPU status: %s", layer.gpu_status_.c_str());
  ImGui::End();
  layer.enable_inspection = open;
  return false;
}

void UniverseLayer::OnCreate() {
  show_orbit_strands = false;
  orbit_display = StarOrbitDisplay::All;
  orbit_strand_radius = 0.1f;
  if (const auto mode = std::getenv("EVOENGINE_UNIVERSE_ORBIT_BENCHMARK")) {
    show_orbit_strands = true;
    orbit_display = std::string(mode) == "selected"   ? StarOrbitDisplay::Selected
                    : std::string(mode) == "occupied" ? StarOrbitDisplay::Occupied
                                                      : StarOrbitDisplay::All;
  }
  depth_write = true;
  star_fade_strength = 0.5f;
  global_simulation_time_ = 0.0;
  frame_number_ = 0;
  pick_minimum_radius_ = 3.0f;
  last_viewport_click_ = 0;
  pick_benchmark_ = std::getenv("EVOENGINE_UNIVERSE_PICK_BENCHMARK") != nullptr;
  follow_benchmark_ = pick_benchmark_ && std::string(std::getenv("EVOENGINE_UNIVERSE_PICK_BENCHMARK")) == "follow";
  registered_render_cluster_count_ = 0;
  registered_render_star_count_ = 0;
  procedural_galaxy_scene_.reset();
  parameter_upload_arena_ = std::make_unique<BufferUploadArena>();
  InitializeGpuResources();
}

void UniverseLayer::InitializeGpuResources() {
  compute_ready_ = false;
  render_ready_ = false;
  const auto physical_device = Platform::GetSelectedPhysicalDevice();
  fp64_supported_ = physical_device && physical_device->features.shaderFloat64 == VK_TRUE;
  if (!fp64_supported_) {
    gpu_status_ = "Disabled: Vulkan shaderFloat64 is unavailable; FP32 fallback is intentionally unsupported.";
    EVOENGINE_ERROR(gpu_status_);
    return;
  }

  star_cluster_layout_ = std::make_shared<DescriptorSetLayout>();
  star_cluster_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  star_cluster_layout_->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  star_cluster_layout_->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  star_cluster_layout_->Initialize();

  const auto shader = std::make_shared<Shader>();
  const auto shader_path = std::filesystem::path("./UniverseResources/Shaders/Compute/StarCluster.slang");
  if (!shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(), shader_path)) {
    gpu_status_ = "Disabled: failed to compile " + shader_path.string();
    EVOENGINE_ERROR(gpu_status_);
    star_cluster_layout_.reset();
    return;
  }
  star_cluster_pipeline_ = std::make_shared<ComputePipeline>();
  star_cluster_pipeline_->compute_shader = shader;
  star_cluster_pipeline_->descriptor_set_layouts.emplace_back(star_cluster_layout_);
  star_cluster_pipeline_->push_constant_ranges.push_back(
      {VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(StarClusterComputePushConstant)});
  star_cluster_pipeline_->Initialize();

  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer) {
    gpu_status_ = "Disabled: RenderLayer is unavailable.";
    EVOENGINE_ERROR(gpu_status_);
    return;
  }
  star_render_layout_ = std::make_shared<DescriptorSetLayout>();
  star_render_layout_->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0);
  star_render_layout_->Initialize();

  const auto vertex_path = std::filesystem::path("./UniverseResources/Shaders/Graphics/StarCluster.vert.slang");
  const auto fragment_path = std::filesystem::path("./UniverseResources/Shaders/Graphics/StarCluster.frag.slang");
  auto vertex_shader = std::make_shared<Shader>();
  auto fragment_shader = std::make_shared<Shader>();
  if (!vertex_shader->TryCompile(ShaderType::Vertex, Platform::GetShaderGlobalDefines(), vertex_path) ||
      !fragment_shader->TryCompile(ShaderType::Fragment, Platform::GetShaderGlobalDefines(), fragment_path)) {
    gpu_status_ = "Disabled: failed to compile Star Cluster forward shaders.";
    EVOENGINE_ERROR(gpu_status_);
    star_render_layout_.reset();
    return;
  }
  star_render_pipeline_ = std::make_shared<GraphicsPipeline>();
  star_render_pipeline_->view_mask = 0;
  star_render_pipeline_->vertex_shader = vertex_shader;
  star_render_pipeline_->fragment_shader = fragment_shader;
  star_render_pipeline_->geometry_type = GeometryType::Mesh;
  star_render_pipeline_->vertex_input_enabled = false;
  star_render_pipeline_->primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  star_render_pipeline_->descriptor_set_layouts.emplace_back(render_layer->GetPerFrameDescriptorSetLayout());
  star_render_pipeline_->descriptor_set_layouts.emplace_back(star_render_layout_);
  star_render_pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
  star_render_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
  star_render_pipeline_->color_attachment_formats = {Platform::Constants::render_texture_color};
  auto& push_constant = star_render_pipeline_->push_constant_ranges.emplace_back();
  push_constant.size = sizeof(StarRenderPushConstant);
  push_constant.offset = 0;
  push_constant.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
  star_render_pipeline_->Initialize();
  const auto hover_vertex = std::make_shared<Shader>();
  const auto hover_fragment = std::make_shared<Shader>();
  if (hover_vertex->TryCompile(ShaderType::Vertex, Platform::GetShaderGlobalDefines(),
                               "./UniverseResources/Shaders/Graphics/StarHover.vert.slang") &&
      hover_fragment->TryCompile(ShaderType::Fragment, Platform::GetShaderGlobalDefines(),
                                 "./UniverseResources/Shaders/Graphics/StarHover.frag.slang")) {
    star_hover_pipeline_ = std::make_shared<GraphicsPipeline>();
    star_hover_pipeline_->view_mask = 0;
    star_hover_pipeline_->vertex_shader = hover_vertex;
    star_hover_pipeline_->fragment_shader = hover_fragment;
    star_hover_pipeline_->geometry_type = GeometryType::Mesh;
    star_hover_pipeline_->vertex_input_enabled = false;
    star_hover_pipeline_->primitive_topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    star_hover_pipeline_->descriptor_set_layouts = star_render_pipeline_->descriptor_set_layouts;
    star_hover_pipeline_->depth_attachment_format = Platform::Constants::render_texture_depth;
    star_hover_pipeline_->stencil_attachment_format = VK_FORMAT_UNDEFINED;
    star_hover_pipeline_->color_attachment_formats = {Platform::Constants::render_texture_color};
    star_hover_pipeline_->push_constant_ranges.push_back(
        {VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(StarHoverPushConstant)});
    star_hover_pipeline_->Initialize();
  } else {
    EVOENGINE_ERROR("Star hover shader compilation failed; hover highlighting is unavailable.");
  }
  compute_ready_ = star_cluster_pipeline_->Initialized();
  render_ready_ = star_render_pipeline_->Initialized();
  if (compute_ready_ && render_ready_)
    star_picker_.Initialize(render_layer->GetPerFrameDescriptorSetLayout());
  gpu_status_ = compute_ready_ && render_ready_ ? "FP64 compute and direct forward pipelines ready"
                                                : "Disabled: Star Cluster GPU pipeline initialization failed.";
}

void UniverseLayer::OnDestroy() {
  ResetSimulation();
  star_picker_ = {};
  star_cluster_pipeline_.reset();
  star_cluster_layout_.reset();
  star_render_pipeline_.reset();
  star_hover_pipeline_.reset();
  star_render_layout_.reset();
  parameter_upload_arena_.reset();
  compute_ready_ = false;
  render_ready_ = false;
  procedural_galaxy_scene_.reset();
}

void UniverseLayer::ConfigureProceduralGalaxyDemoIfNeeded() {
  const auto scene = GetScene();
  if (!scene || !IsProceduralGalaxyProjectPath()) {
    demo_main_camera_.Restore();
    demo_scene_camera_.Restore();
    return;
  }
  auto owners = scene->UnsafeGetPrivateComponentOwnersList<StarCluster>();
  if (procedural_galaxy_scene_.lock() != scene) {
    procedural_galaxy_scene_ = scene;
    if (!owners || owners->empty()) {
      const auto first_entity = scene->CreateEntity("Star Cluster");
      const auto first = scene->GetOrSetPrivateComponent<StarCluster>(first_entity).lock();
      first->seed = 1;
      first->SetStarCount(500000);
      demo_needs_framing_ = true;
    }
  }
  demo_main_camera_.Apply(scene->main_camera.Get<Camera>());
  if (const auto editor = ApplicationContext::Get().GetLayer<EditorLayer>())
    demo_scene_camera_.Apply(editor->GetSceneCamera());
  else
    demo_scene_camera_.Restore();
}

void UniverseLayer::ResetSimulation() {
  displayed_orbits_ = orbit_draws_ = 0;
  orbit_rebuild_ms_ = orbit_upload_ms_ = 0;
  orbit_status_ = "Not submitted";
  demo_main_camera_.Restore();
  demo_scene_camera_.Restore();
  if (star_follow_.following)
    if (const auto editor = ApplicationContext::Get().GetLayer<EditorLayer>())
      editor->RebaseSceneCamera(glm::scale(glm::dmat4(1), glm::dvec3(kGalaxyDisplayScale)) *
                                star_follow_.reference_to_world);
  star_follow_ = {};
  star_view_ = {};
  demo_needs_framing_ = false;
  last_follow_toggle_ = 0;
  if (base_sample_buffer_ || !orbit_strands_.empty())
    Platform::WaitForFrameSubmissions("Reset Universe star batch");
  orbit_strands_.clear();
  orbit_material_.reset();
  frame_slots_.clear();
  star_picker_.Reset();
  last_viewport_click_ = 0;
  base_sample_buffer_.reset();
  batch_ = {};
  star_capacity_ = cluster_capacity_ = 0;
  global_simulation_time_ = 0.0;
  computed_revision_ = rendered_revision_ = 0;
  render_slot_ = draws_this_frame_ = 0;
  simulation_scene_.reset();
}

void UniverseLayer::EnsureBatchResources(const bool population_changed) {
  const size_t frame_count = static_cast<size_t>((std::max)(Platform::GetMaxFramesInFlight(), 1));
  const bool slots_changed = frame_slots_.size() != frame_count;
  if (!population_changed && !slots_changed)
    return;
  if (base_sample_buffer_)
    Platform::WaitForFrameSubmissions("Edit Universe star batch");
  const size_t max_range = Platform::GetSelectedPhysicalDevice()->properties.limits.maxStorageBufferRange;
  const auto grow = [](size_t capacity, const size_t required, const size_t maximum) {
    capacity = (std::max)(capacity, size_t{1});
    while (capacity < required)
      capacity = (std::min)(capacity * 2, maximum);
    return capacity;
  };
  const size_t stars = grow(star_capacity_, batch_.samples.size(), max_range / sizeof(StarClusterGpuResult));
  const size_t clusters = grow(cluster_capacity_, batch_.ranges.size(), max_range / sizeof(StarClusterGpuParameters));
  if (slots_changed || stars != star_capacity_ || clusters != cluster_capacity_) {
    star_capacity_ = stars;
    cluster_capacity_ = clusters;
    base_sample_buffer_ = CreateDeviceBuffer(stars * sizeof(StarBaseSample),
                                             VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
    frame_slots_.assign(frame_count, {});
    for (auto& slot : frame_slots_) {
      slot.parameter_buffer = CreateDeviceBuffer(clusters * sizeof(StarClusterGpuParameters),
                                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
      slot.result_buffer = CreateDeviceBuffer(stars * sizeof(StarClusterGpuResult), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
      slot.compute_descriptor_set = std::make_shared<DescriptorSet>(star_cluster_layout_);
      slot.compute_descriptor_set->UpdateBufferDescriptorBinding(0, slot.parameter_buffer);
      slot.compute_descriptor_set->UpdateBufferDescriptorBinding(1, base_sample_buffer_);
      slot.compute_descriptor_set->UpdateBufferDescriptorBinding(2, slot.result_buffer);
      slot.render_descriptor_set = std::make_shared<DescriptorSet>(star_render_layout_);
      slot.render_descriptor_set->UpdateBufferDescriptorBinding(0, slot.result_buffer);
    }
  }
  if (!batch_.samples.empty())
    base_sample_buffer_->UploadData(batch_.samples.size() * sizeof(StarBaseSample), batch_.samples.data());
  star_picker_.EnsureResources(frame_count, star_capacity_);
}

void universe_package::ConfigureStarRenderStates(GraphicsPipeline& pipeline, const glm::ivec4& viewport,
                                                 const bool depth_write) {
  pipeline.states.ResetAllStates(1);
  pipeline.states.SetViewportScissor(viewport);
  pipeline.states.cull_mode = VK_CULL_MODE_NONE;
  pipeline.states.depth_test = true;
  pipeline.states.depth_write = depth_write;
  pipeline.states.depth_compare = VK_COMPARE_OP_LESS_OR_EQUAL;
  auto& blend = pipeline.states.color_blend_attachment_states[0];
  blend.blendEnable = VK_FALSE;
  blend.colorWriteMask =
      VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
}

void UniverseLayer::RegisterForwardRendering(StarBatchRenderPacket packet) {
  if (packet.star_count == 0 || !render_ready_ || !star_render_pipeline_)
    return;
  registered_render_cluster_count_ = static_cast<uint32_t>(batch_.ranges.size());
  registered_render_star_count_ = packet.star_count;
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  if (!render_layer)
    return;
  const ProfilerScope registration_scope(universe_profiler::GetItems().render_registration);
  const auto pipeline = star_render_pipeline_;
  const std::weak_ptr<UniverseLayer> self = std::static_pointer_cast<UniverseLayer>(GetSelf());
  const auto picking_request = star_picker_.state.current;
  const auto star_results = frame_slots_.at(packet.frame_slot).result_buffer;
  uint32_t hover_index = UINT32_MAX;
  if (!star_follow_.following && star_picker_.state.hovered.result.valid) {
    const auto& hovered = star_picker_.state.hovered;
    for (const auto& range : picking_request.ranges)
      if (range.identity == hovered.identity && range.seed == hovered.seed && hovered.ordinal < range.count &&
          range.cluster.lock() == hovered.cluster.lock())
        hover_index = range.offset + hovered.ordinal;
  }
  const auto hover_pipeline = star_hover_pipeline_;
  render_layer->ForwardRenderingAllCameras(
      [pipeline, packet = std::move(packet), self, picking_request, star_results, hover_index, hover_pipeline](
          const VkCommandBuffer command_buffer, const std::shared_ptr<Camera>& camera,
          const RenderLayer::ForwardRenderingView& view) -> uint32_t {
        if (!camera || !camera->GetRenderTexture() || !RenderLayer::GetPerFrameDescriptorSet())
          return 0;
        if (const auto layer = self.lock())
          layer->star_picker_.Record(command_buffer, packet.frame_slot, star_results, camera, view.camera_index,
                                     RenderLayer::GetPerFrameDescriptorSet(), picking_request);
        ConfigureStarRenderStates(*pipeline, view.viewport, packet.depth_write);

        // ForwardExternal already transitions both graph resources to attachment layouts.
        std::vector<VkRenderingAttachmentInfo> colors;
        camera->GetRenderTexture()->AppendColorAttachmentInfos(colors, VK_ATTACHMENT_LOAD_OP_LOAD,
                                                               VK_ATTACHMENT_STORE_OP_STORE);
        const auto depth = camera->GetRenderTexture()->GetDepthAttachmentInfo(VK_ATTACHMENT_LOAD_OP_LOAD,
                                                                              VK_ATTACHMENT_STORE_OP_STORE);
        VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};
        rendering.renderArea = {{view.viewport.x, view.viewport.y},
                                {static_cast<uint32_t>(view.viewport.z), static_cast<uint32_t>(view.viewport.w)}};
        rendering.layerCount = 1;
        rendering.colorAttachmentCount = static_cast<uint32_t>(colors.size());
        rendering.pColorAttachments = colors.data();
        rendering.pDepthAttachment = &depth;
        Platform::BeginRendering(command_buffer, rendering);
        {
          const GpuProfilerCommandScope render_scope(command_buffer, universe_profiler::GetItems().forward_render);
          pipeline->states.ApplyAllStates(command_buffer);
          pipeline->Bind(command_buffer);
          pipeline->BindDescriptorSet(command_buffer, 0, RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
          StarRenderPushConstant constants;
          constants.camera_index = view.camera_index;
          constants.viewport_size = {view.viewport.z, view.viewport.w};
          constants.fade_strength = packet.fade_strength;
          constants.distance_compression = StarDistanceCompression(camera->camera_settings.far_distance);
          if (const auto stack = camera->post_processing_stack_ref.Get<PostProcessingStack>();
              stack && stack->enable_bloom && stack->bloom) {
            // Leave a rounding margin below the bloom soft-knee onset, including FP16 targets.
            constants.subpixel_brightness_limit =
                (std::max)(0.0f, stack->bloom->threshold - (std::max)(stack->bloom->knee, 0.0001f)) * 0.999f;
          }
          pipeline->PushConstant(command_buffer, 0, constants);
          pipeline->BindDescriptorSet(command_buffer, 1, packet.descriptor_set->GetVkDescriptorSet());
          Platform::Draw(command_buffer, 6u, packet.star_count);
          if (const auto layer = self.lock()) {
            layer->rendered_revision_ = packet.population_revision;
            layer->render_slot_ = packet.frame_slot;
            ++layer->draws_this_frame_;
          }
        }
        if (hover_index != UINT32_MAX && picking_request.valid && picking_request.camera.lock() == camera &&
            hover_pipeline && hover_pipeline->Initialized()) {
          const GpuProfilerCommandScope hover_scope(command_buffer, universe_profiler::GetItems().hover_render);
          ConfigureStarRenderStates(*hover_pipeline, view.viewport, false);
          auto& blend = hover_pipeline->states.color_blend_attachment_states[0];
          blend.blendEnable = VK_TRUE;
          blend.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
          blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
          blend.colorBlendOp = VK_BLEND_OP_ADD;
          blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
          blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
          blend.alphaBlendOp = VK_BLEND_OP_ADD;
          hover_pipeline->states.ApplyAllStates(command_buffer);
          hover_pipeline->Bind(command_buffer);
          hover_pipeline->BindDescriptorSet(command_buffer, 0,
                                            RenderLayer::GetPerFrameDescriptorSet()->GetVkDescriptorSet());
          hover_pipeline->BindDescriptorSet(command_buffer, 1, packet.descriptor_set->GetVkDescriptorSet());
          StarHoverPushConstant constants;
          constants.camera_index = view.camera_index;
          constants.viewport_size = {view.viewport.z, view.viewport.w};
          constants.display_size = picking_request.display_size;
          constants.minimum_radius = picking_request.minimum_radius;
          constants.star_index = hover_index;
          constants.brightness_limit = 1.0f;
          if (const auto stack = camera->post_processing_stack_ref.Get<PostProcessingStack>();
              stack && stack->enable_bloom && stack->bloom)
            constants.brightness_limit =
                (std::max)(0.0f, stack->bloom->threshold - (std::max)(stack->bloom->knee, 0.0001f)) * 0.999f;
          hover_pipeline->PushConstant(command_buffer, 0, constants);
          Platform::Draw(command_buffer, 6u, 1u);
        }
        Platform::EndRendering(command_buffer);
        return packet.star_count * 6u;
      });
}

void UniverseLayer::UpdatePlanetTerrain(const std::shared_ptr<Scene>& scene) const {
  const auto terrains = scene->UnsafeGetPrivateComponentOwnersList<PlanetTerrain>();
  const auto main_camera = scene->main_camera.Get<Camera>();
  if (!terrains || !main_camera)
    return;
  std::mutex mesh_gen_lock;
  const auto camera_transform = scene->GetDataComponent<GlobalTransform>(main_camera->GetOwner());
  for (const auto entity : *terrains) {
    const auto terrain = scene->GetOrSetPrivateComponent<PlanetTerrain>(entity).lock();
    if (!scene->IsEntityEnabled(entity) || !terrain || !terrain->IsEnabled())
      continue;
    const auto planet_transform = scene->GetDataComponent<GlobalTransform>(entity);
    for (auto& chunk : terrain->chunks_)
      CheckLod(mesh_gen_lock, chunk, terrain->info_, planet_transform, camera_transform);
    GlobalTransform render_transform;
    render_transform.value =
        glm::translate(glm::mat4_cast(planet_transform.GetRotation()), glm::vec3(planet_transform.GetPosition()));
    if (const auto material = terrain->surface_material.Get<Material>()) {
      for (const auto& chunk : terrain->chunks_)
        RenderChunk(chunk, material, render_transform);
    }
  }
}

void UniverseLayer::UpdatePickingInput(const std::shared_ptr<Scene>& scene) {
  const ProfilerScope scope(universe_profiler::GetItems().pick_input);
  StarPickRequest request;
  auto camera = scene->main_camera.Get<Camera>();
  pick_camera_name_ = "Main camera";
  bool clicked = false;
  if (const auto editor = ApplicationContext::Get().GetLayer<EditorLayer>()) {
    const bool use_scene = editor->GetSceneViewportInput().focused;
    const auto& input = use_scene ? editor->GetSceneViewportInput() : editor->GetMainCameraViewportInput();
    pick_camera_name_ = use_scene ? "Scene camera" : "Main camera";
    if (use_scene)
      camera = editor->GetSceneCamera();
    request.cursor_uv = input.cursor_uv;
    request.display_size = input.image_size;
    request.image_origin = input.image_origin;
    request.valid = input.visible && input.cursor_valid && input.scene.lock() == scene && input.camera.lock() == camera;
    clicked = request.valid && input.click_sequence != 0 && input.click_sequence != last_viewport_click_;
    if (clicked)
      last_viewport_click_ = input.click_sequence;
  } else if (const auto window = ApplicationContext::Get().GetLayer<WindowLayer>(); window && camera) {
    int width = 0, height = 0;
    glfwGetWindowSize(window->GetGlfwWindow(), &width, &height);
    const auto mouse = Input::GetMousePosition();
    // The non-editor presenter aspect-fits the main camera to the window.
    const auto size = glm::vec2(camera->GetSize());
    const float scale = (std::min)(width / size.x, height / size.y);
    request.display_size = size * scale;
    const glm::vec2 origin = (glm::vec2(width, height) - request.display_size) * 0.5f;
    request.valid = EditorLayer::MapViewportCursor(origin, request.display_size, mouse, request.cursor_uv);
    clicked = request.valid && scene->GetKey(GLFW_MOUSE_BUTTON_LEFT) == Input::KeyActionType::Press;
  }
  if (pick_benchmark_ && camera) {
    request.cursor_uv = glm::vec2(0.5f);
    request.display_size = glm::vec2(camera->GetSize());
    request.valid = true;
    clicked = false;
  }
  request.camera = camera;
  request.camera_handle = camera ? camera->GetHandle().GetValue() : 0;
  request.frame = frame_number_;
  request.population_revision = batch_.population_revision;
  request.reference_generation = star_follow_.generation;
  request.minimum_radius = pick_minimum_radius_;
  for (const auto& range : batch_.ranges) {
    std::weak_ptr<StarCluster> cluster;
    for (const auto& [pointer, clock] : batch_.clocks)
      if (clock.identity == range.identity) {
        cluster = clock.component;
        break;
      }
    request.ranges.push_back({range.identity, range.seed, range.offset, range.count, cluster});
  }
  request.valid = request.valid && camera && !request.ranges.empty();
  star_picker_.state.Update(std::move(request), clicked);
}

void UniverseLayer::Update() {
  const auto scene = GetScene();
  registered_render_cluster_count_ = registered_render_star_count_ = draws_this_frame_ = 0;
  if (!scene) {
    demo_main_camera_.Restore();
    demo_scene_camera_.Restore();
    if (base_sample_buffer_ || !batch_.clocks.empty())
      ResetSimulation();
    return;
  }
  if (simulation_scene_.lock() != scene) {
    ResetSimulation();
    simulation_scene_ = scene;
  }
  ConfigureProceduralGalaxyDemoIfNeeded();
  UpdatePlanetTerrain(scene);
  global_simulation_time_ += ApplicationContext::Get().GetTimes().DeltaTime();
  ++frame_number_;
  if (!compute_ready_ || !render_ready_)
    return;

  std::vector<StarClusterInput> inputs;
  const size_t max_range = Platform::GetSelectedPhysicalDevice()->properties.limits.maxStorageBufferRange;
  if (const auto owners = scene->UnsafeGetPrivateComponentOwnersList<StarCluster>()) {
    inputs.reserve(owners->size());
    for (const auto entity : *owners) {
      const auto cluster = scene->GetOrSetPrivateComponent<StarCluster>(entity).lock();
      if (!cluster)
        continue;
      const bool enabled = scene->IsEntityEnabled(entity) && cluster->IsEnabled();
      inputs.push_back({cluster, glm::dmat4(scene->GetDataComponent<GlobalTransform>(entity).value), enabled});
    }
  }
  {
    const ProfilerScope parameters_scope(universe_profiler::GetItems().parameters);
    star_view_.Update(ApplicationContext::Get().GetTimes().Now());
    try {
      const bool changed = batch_.Update(inputs, global_simulation_time_, max_range / sizeof(StarClusterGpuResult));
      if (batch_.ranges.size() > max_range / sizeof(StarClusterGpuParameters)) {
        gpu_status_ = "Active cluster table exceeds Vulkan storage-buffer limits.";
        return;
      }
      EnsureBatchResources(changed);
    } catch (const std::length_error& error) {
      gpu_status_ = error.what();
      return;
    }
  }
  const auto overview_pose = [&](const std::shared_ptr<Camera>& camera, const glm::dvec3& position,
                                 const glm::dquat& rotation) {
    double radius = 0;
    for (size_t i = 0; i < batch_.ranges.size(); ++i) {
      const auto& range = batch_.ranges[i];
      for (const auto& input : inputs) {
        const auto& clock = batch_.clocks.at(input.cluster.get());
        if (clock.identity == range.identity) {
          auto parameters = BuildStarClusterParameters(*input.cluster, input.world_transform, 0);
          ApplyStarDisplayFrame(parameters, glm::scale(glm::dmat4(1), glm::dvec3(kGalaxyDisplayScale)),
                                30 * kGalaxyDisplayScale);
          radius = (std::max)(radius, StarClusterBoundingRadius(parameters, batch_.gaussian_bounds[i]));
          break;
        }
      }
    }
    return CalculateStarOverviewCameraPose(position, rotation, radius, glm::dmat4(camera->GetProjection()),
                                           camera->camera_settings.near_distance);
  };
  if (demo_needs_framing_ && !batch_.ranges.empty()) {
    if (const auto camera = scene->main_camera.Get<Camera>()) {
      auto transform = scene->GetDataComponent<Transform>(camera->GetOwner());
      const auto pose = overview_pose(camera, glm::dvec3(transform.GetPosition()), glm::dquat(transform.GetRotation()));
      if (pose.valid) {
        transform.SetPosition(glm::vec3(pose.position));
        transform.SetRotation(glm::quat(pose.rotation));
        scene->SetDataComponent(camera->GetOwner(), transform);
      }
    }
    if (const auto editor = ApplicationContext::Get().GetLayer<EditorLayer>()) {
      const auto pose = overview_pose(editor->GetSceneCamera(), glm::dvec3(editor->GetSceneCameraPosition()),
                                      glm::dquat(editor->GetSceneCameraRotation()));
      if (pose.valid)
        editor->MoveCamera(glm::quat(pose.rotation), glm::vec3(pose.position));
    }
    demo_needs_framing_ = false;
  }
  UpdatePickingInput(scene);
  star_picker_.Consume(Platform::GetCurrentFrameIndex());
  {
    const ProfilerScope follow_scope(universe_profiler::GetItems().follow_cpu);
    const auto editor = ApplicationContext::Get().GetLayer<EditorLayer>();
    bool toggle = false;
    if (editor) {
      const auto& input = editor->GetSceneViewportInput().focused ? editor->GetSceneViewportInput()
                                                                  : editor->GetMainCameraViewportInput();
      toggle = input.follow_toggle_sequence != 0 && input.follow_toggle_sequence != last_follow_toggle_;
      if (toggle)
        last_follow_toggle_ = input.follow_toggle_sequence;
    }
    if (follow_benchmark_ && editor && star_follow_.generation == 0 && star_picker_.state.hovered.result.valid) {
      star_picker_.state.selected = star_picker_.state.hovered;
      toggle = true;
      EVOENGINE_LOG("Universe follow benchmark: selected hovered star and entered star-local frame.");
    }
    const bool was_following = star_follow_.following;
    auto change = star_follow_.Update(star_picker_.state.selected, batch_, toggle);
    if (change.rebase) {
      const auto galaxy_to_world = glm::scale(glm::dmat4(1), glm::dvec3(1.0 / kGalaxyDisplayScale));
      const auto world_to_galaxy = glm::scale(glm::dmat4(1), glm::dvec3(kGalaxyDisplayScale));
      if (!was_following && star_follow_.following)
        change.camera_transform *= galaxy_to_world;
      else if (was_following && !star_follow_.following)
        change.camera_transform = world_to_galaxy * change.camera_transform;
      star_view_.SetLocked(star_follow_.following, ApplicationContext::Get().GetTimes().Now(), star_fade_strength);
      if (editor) {
        editor->RebaseSceneCamera(change.camera_transform);
        const auto pose = star_follow_.following
                              ? CalculateStarFollowCameraPose(star_follow_.selected_radius)
                              : overview_pose(editor->GetSceneCamera(), glm::dvec3(editor->GetSceneCameraPosition()),
                                              glm::dquat(editor->GetSceneCameraRotation()));
        if (pose.valid)
          editor->MoveCamera(glm::quat(pose.rotation), glm::vec3(pose.position));
        if (IsProceduralGalaxyProjectPath())
          demo_scene_camera_.Apply(editor->GetSceneCamera());
      }
      auto request = star_picker_.state.current;
      request.reference_generation = star_follow_.generation;
      star_picker_.state.Update(std::move(request), false);
    }
    star_picker_.state.SetInteractionLocked(star_follow_.following);
  }
  if (batch_.samples.empty()) {
    RenderOrbitStrands({});
    gpu_status_ = "Empty star batch; no compute or draw";
    return;
  }

  const uint32_t frame_index = Platform::GetCurrentFrameIndex();
  const auto& slot = frame_slots_.at(frame_index);
  BufferUploadBatch uploads;
  auto parameters = batch_.parameters;
  const double coordinate_scale = star_follow_.following ? 1.0 : kGalaxyDisplayScale;
  const auto display_transform = star_follow_.following ? glm::inverse(star_follow_.reference_to_world)
                                                        : glm::scale(glm::dmat4(1), glm::dvec3(kGalaxyDisplayScale));
  for (auto& parameter : parameters)
    ApplyStarDisplayFrame(parameter, display_transform, star_view_.radius_scale * coordinate_scale);
  RenderOrbitStrands(parameters);
  uploads.AddVector(slot.parameter_buffer, parameters);
  uploads.Record(*parameter_upload_arena_);
  const auto pipeline = star_cluster_pipeline_;
  const auto descriptor = slot.compute_descriptor_set;
  const auto results = slot.result_buffer;
  Platform::RecordCommandsMainQueue(
      [pipeline, descriptor, results, ranges = batch_.ranges](const VkCommandBuffer command_buffer) {
        {
          const GpuProfilerCommandScope compute_scope(command_buffer, universe_profiler::GetItems().compute);
          pipeline->Bind(command_buffer);
          pipeline->BindDescriptorSet(command_buffer, 0, descriptor->GetVkDescriptorSet());
          for (uint32_t index = 0; index < ranges.size(); ++index) {
            const auto& range = ranges[index];
            pipeline->PushConstant(command_buffer, 0, StarClusterComputePushConstant{index, range.offset});
            pipeline->Dispatch(command_buffer, Platform::DivUp(range.count, kWorkgroupSize), 1, 1);
          }
        }
        Platform::BufferMemoryBarrier(command_buffer, *results, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                      VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT,
                                      VK_ACCESS_2_SHADER_READ_BIT);
      });
  computed_revision_ = batch_.population_revision;
  gpu_status_ = "Batched FP64 compute submitted; direct forward rendering registered";
  RegisterForwardRendering({slot.render_descriptor_set, batch_.population_revision, frame_index,
                            static_cast<uint32_t>(batch_.samples.size()), depth_write,
                            static_cast<float>(star_view_.fade_strength)});
}
