#pragma once
#include "GpuTreePhysicsState.hpp"
#include "TreePhysicsLod.hpp"

namespace eco_sys_lab_package {
struct GpuTreePhysicsAddresses {
  std::array<uint64_t, 2> vertices{}, rods{};
  uint64_t angles = 0, vertex_rods = 0, rod_angles = 0, node_rods = 0, predicted = 0, previous = 0;
  uint64_t vertex_coefficients = 0, rod_coefficients = 0, angle_coefficients = 0, reserved = 0;
  glm::uvec4 counts{};
};
struct GpuTreePhysicsLodLevel {
  GpuTreePhysicsAddresses rest;
  uint64_t vertex_map = 0, rod_map = 0, original_vertices = 0, velocity_ranges = 0, velocity_weights = 0, members = 0;
  glm::vec4 metrics{};
};
struct GpuTreePhysicsLodHeader {
  GpuTreePhysicsAddresses full, scratch;
  uint64_t levels = 0, state = 0;
  glm::uvec4 counts{};
};
struct GpuTreePhysicsLodStorage {
  std::shared_ptr<GpuTreePhysicsState> scratch;
  std::vector<std::shared_ptr<GpuTreePhysicsState>> templates;
  std::vector<std::shared_ptr<Buffer>> maps;
  std::shared_ptr<Buffer> levels, header, state;
  GpuTreePhysicsLodHeader cached_header{};
  uint32_t level_count = 0, error_group_count = 0;
};
static_assert(sizeof(GpuTreePhysicsAddresses) == 128);
static_assert(sizeof(GpuTreePhysicsLodLevel) == 192);
static_assert(sizeof(GpuTreePhysicsLodHeader) == 288);
GpuTreePhysicsAddresses GetGpuTreePhysicsAddresses(const GpuTreePhysicsState& state);
std::shared_ptr<GpuTreePhysicsState> MakeGpuTreePhysicsState(const DynamicTreeSkeleton::GpuRestState& rest, bool arrays,
                                                             bool immutable = false);
std::shared_ptr<GpuTreePhysicsLodStorage> BuildGpuTreePhysicsLod(const DynamicTreeSkeleton::GpuRestState& rest,
                                                                 const std::shared_ptr<GpuTreePhysicsState>& full);
void UpdateGpuTreePhysicsLodHeader(GpuTreePhysicsLodStorage& lod, const GpuTreePhysicsState& full,
                                   BufferUploadBatch& upload);
}  // namespace eco_sys_lab_package
