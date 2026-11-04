#include "GpuTreePhysicsLod.hpp"
#include <cstring>
#include <glm/gtc/quaternion.hpp>
#include "GpuTreePhysicsBuffers.hpp"

using namespace eco_sys_lab_package;
using namespace eco_sys_lab_package::tree_physics_buffers;

GpuTreePhysicsAddresses eco_sys_lab_package::GetGpuTreePhysicsAddresses(const GpuTreePhysicsState& t) {
  GpuTreePhysicsAddresses a;
  for (uint32_t i = 0; i < 2; ++i) {
    a.vertices[i] = t.vertices[t.vertex_index ^ i]->GetDeviceAddress();
    a.rods[i] = t.rods[t.rod_index ^ i]->GetDeviceAddress();
  }
  const auto address = [](const auto& buffer) {
    return buffer ? buffer->GetDeviceAddress() : uint64_t(0);
  };
  a.angles = address(t.angles);
  a.vertex_rods = address(t.vertex_rods);
  a.rod_angles = address(t.rod_angles);
  a.node_rods = address(t.node_rods);
  a.predicted = address(t.predicted);
  a.previous = address(t.previous);
  a.vertex_coefficients = address(t.vertex_coefficients);
  a.rod_coefficients = address(t.rod_coefficients);
  a.angle_coefficients = address(t.angle_coefficients);
  a.counts = {t.vertex_count, t.rod_count, t.angle_count, t.attribute_arrays ? 1u : 0u};
  return a;
}
std::shared_ptr<GpuTreePhysicsState> eco_sys_lab_package::MakeGpuTreePhysicsState(
    const DynamicTreeSkeleton::GpuRestState& rest, bool arrays, bool immutable) {
  auto state = std::make_shared<GpuTreePhysicsState>();
  state->attribute_arrays = arrays;
  state->reference_density = rest.density;
  BufferUploadBatch upload;
  const auto make_buffer = [&](const auto& source) {
    using T = typename std::decay_t<decltype(source)>::value_type;
    static const T empty{};
    auto buffer = Allocate(std::max<size_t>(1, source.size()) * sizeof(T));
    upload.Add(buffer, source.empty() ? &empty : source.data(), std::max<size_t>(1, source.size()) * sizeof(T));
    return buffer;
  };
  const auto pack = [](const auto& source) {
    using T = typename std::decay_t<decltype(source)>::value_type;
    static_assert(sizeof(T) % 16 == 0);
    constexpr size_t fields = sizeof(T) / 16;
    std::vector<glm::uvec4> packed(source.size() * fields);
    for (size_t i = 0; i < source.size(); ++i)
      for (size_t field = 0; field < fields; ++field)
        std::memcpy(&packed[field * source.size() + i], reinterpret_cast<const char*>(&source[i]) + field * 16, 16);
    return packed;
  };
  const auto vertices = arrays ? pack(rest.vertices) : std::vector<glm::uvec4>{};
  const auto rods = arrays ? pack(rest.rods) : std::vector<glm::uvec4>{};
  for (uint32_t i = 0; i < (immutable ? 1u : 2u); ++i) {
    state->vertices[i] = arrays ? make_buffer(vertices) : make_buffer(rest.vertices);
    state->rods[i] = arrays ? make_buffer(rods) : make_buffer(rest.rods);
  }
  if (immutable) {
    state->vertices[1] = state->vertices[0];
    state->rods[1] = state->rods[0];
  }
  state->angles = make_buffer(rest.angles);
  state->vertex_rods = make_buffer(rest.vertex_rods);
  state->rod_angles = make_buffer(rest.rod_angles);
  state->node_rods = make_buffer(rest.node_rods);
  upload.SubmitImmediate();
  state->vertex_count = static_cast<uint32_t>(rest.vertices.size());
  state->rod_count = static_cast<uint32_t>(rest.rods.size());
  state->angle_count = static_cast<uint32_t>(rest.angles.size());
  state->node_count = static_cast<uint32_t>(rest.node_rods.size());
  if (!immutable) {
    state->predicted = Allocate(rest.vertices.size() * sizeof(glm::vec4));
    state->previous = Allocate(rest.vertices.size() * sizeof(glm::vec4));
    state->vertex_coefficients = Allocate(rest.vertices.size() * sizeof(glm::vec2));
    state->rod_coefficients = Allocate(rest.rods.size() * sizeof(glm::vec2));
    state->angle_coefficients = Allocate(rest.angles.size() * sizeof(float));
  }
  state->previous_model = rest.initial_model;
  GlobalTransform transform;
  transform.value = rest.initial_model;
  state->initial_rotation = transform.GetRotation();
  return state;
}
std::shared_ptr<GpuTreePhysicsLodStorage> eco_sys_lab_package::BuildGpuTreePhysicsLod(
    const DynamicTreeSkeleton::GpuRestState& rest, const std::shared_ptr<GpuTreePhysicsState>& full) {
  auto hierarchy = BuildTreePhysicsLod(rest);
  auto lod = std::make_shared<GpuTreePhysicsLodStorage>();
  lod->level_count = static_cast<uint32_t>(hierarchy.size());
  lod->error_group_count = ((std::max(full->vertex_count, full->rod_count) + 63) / 64) * (lod->level_count - 1);
  lod->scratch = std::make_shared<GpuTreePhysicsState>(*full);
  lod->scratch->lod.reset();
  if (lod->level_count > 1) {
    for (uint32_t i = 0; i < 2; ++i) {
      lod->scratch->vertices[i] = Allocate(rest.vertices.size() * sizeof(DynamicTreeSkeleton::GpuRestState::Vertex));
      lod->scratch->rods[i] = Allocate(rest.rods.size() * sizeof(DynamicTreeSkeleton::GpuRestState::Rod));
    }
    lod->scratch->predicted = Allocate(rest.vertices.size() * sizeof(glm::vec4));
    lod->scratch->previous = Allocate(rest.vertices.size() * sizeof(glm::vec4));
    lod->scratch->vertex_coefficients = Allocate(rest.vertices.size() * sizeof(glm::vec2));
    lod->scratch->rod_coefficients = Allocate(rest.rods.size() * sizeof(glm::vec2));
    lod->scratch->angle_coefficients = Allocate(rest.angles.size() * sizeof(float));
  }
  std::vector<GpuTreePhysicsLodLevel> levels(lod->level_count);
  levels.front().rest = GetGpuTreePhysicsAddresses(*full);
  const auto map_buffer = [&](const auto& values) {
    auto buffer = MakeBuffer(values);
    const auto address = buffer->GetDeviceAddress();
    lod->maps.push_back(std::move(buffer));
    return address;
  };
  struct VelocityWeight {
    uint32_t vertex;
    float weight;
  };
  for (uint32_t index = 1; index < lod->level_count; ++index) {
    auto& source = hierarchy[index];
    auto& level = levels[index];
    std::vector<uint32_t> members;
    for (uint32_t rod = 0; rod < source.groups.size(); ++rod) {
      source.rest.rods[rod].incident_angles.z = static_cast<uint32_t>(members.size());
      source.rest.rods[rod].incident_angles.w = static_cast<uint32_t>(source.groups[rod].size());
      members.insert(members.end(), source.groups[rod].begin(), source.groups[rod].end());
    }
    auto prototype = MakeGpuTreePhysicsState(source.rest, true, true);
    level.rest = GetGpuTreePhysicsAddresses(*prototype);
    lod->templates.push_back(std::move(prototype));
    std::vector<std::vector<VelocityWeight>> contributions(source.rest.vertices.size());
    for (uint32_t vertex = 0; vertex < rest.vertices.size(); ++vertex) {
      const float inverse_mass = rest.vertices[vertex].position_inverse_mass.w;
      if (inverse_mass <= 0)
        continue;
      const auto& mapping = source.vertices[vertex];
      if (mapping.links.y != UINT32_MAX)
        contributions[mapping.links.y].push_back({vertex, 1 / inverse_mass});
      else {
        const auto& rod = source.rest.rods[mapping.links.x];
        contributions[rod.links.x].push_back({vertex, (1 - mapping.offset_t.w) / inverse_mass});
        contributions[rod.links.y].push_back({vertex, mapping.offset_t.w / inverse_mass});
      }
    }
    std::vector<glm::uvec2> ranges;
    std::vector<VelocityWeight> weights;
    for (auto& values : contributions) {
      float mass = 0;
      for (const auto& value : values)
        mass += value.weight;
      ranges.emplace_back(static_cast<uint32_t>(weights.size()), static_cast<uint32_t>(values.size()));
      for (auto value : values) {
        value.weight /= std::max(mass, 1e-20f);
        weights.push_back(value);
      }
    }
    level.vertex_map = map_buffer(source.vertices);
    level.rod_map = map_buffer(source.rods);
    level.original_vertices = map_buffer(source.original_vertices);
    level.members = map_buffer(members);
    level.velocity_ranges = map_buffer(ranges);
    level.velocity_weights = map_buffer(weights);
    level.metrics = {source.approximation_error, glm::radians(10.0f), source.span_error, 0};
  }
  std::vector<uint32_t> state(8 + lod->level_count, 0);
  state[2] = UINT32_MAX;
  lod->state = MakeBuffer(state);
  lod->levels = MakeBuffer(levels);
  lod->header = Allocate(sizeof(GpuTreePhysicsLodHeader));
  return lod;
}
void eco_sys_lab_package::UpdateGpuTreePhysicsLodHeader(GpuTreePhysicsLodStorage& lod, const GpuTreePhysicsState& full,
                                                        BufferUploadBatch& upload) {
  lod.scratch->vertex_index = full.vertex_index;
  lod.scratch->rod_index = full.rod_index;
  GpuTreePhysicsLodHeader header;
  header.full = GetGpuTreePhysicsAddresses(full);
  header.scratch = GetGpuTreePhysicsAddresses(*lod.scratch);
  header.levels = lod.levels->GetDeviceAddress();
  header.state = lod.state->GetDeviceAddress();
  header.counts.x = lod.level_count;
  if (std::memcmp(&header, &lod.cached_header, sizeof(header)) == 0)
    return;
  lod.cached_header = header;
  upload.Add(lod.header, lod.cached_header, {BufferUploadUsage::StorageRead, BufferUploadCapacityPolicy::GrowIfNeeded});
}
