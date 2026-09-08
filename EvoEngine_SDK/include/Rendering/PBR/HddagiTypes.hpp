#pragma once

#include <array>
#include <cstdint>
#include <glm/glm.hpp>

namespace evo_engine {
// Matches HddagiTrace.slang and HddagiRegionStore.slang.
struct alignas(16) HddagiCascadeData {
  glm::vec3 offset{0};
  float to_cell = 1;
  glm::ivec3 region_world_offset{0};
  float padding = 0;
};
struct alignas(16) HddagiCascadeBlock {
  std::array<HddagiCascadeData, 8> data{};
};
struct alignas(16) HddagiRegionParams {
  glm::ivec3 grid{0};
  uint32_t cascade = 0;
  glm::ivec3 offset{0};
  uint32_t version = 1;
  glm::ivec3 region_world_offset{0};
  uint32_t padding = 0;
};
struct alignas(16) HddagiRay {
  glm::vec3 origin{0};
  uint32_t cascade = 0;
  glm::vec3 direction{0};
  float distance = 0;
};
struct alignas(16) HddagiTraceParams {
  glm::ivec3 grid{0};
  uint32_t ray_count = 0;
  uint32_t cascade_count = 1;
  uint32_t fractional_bits = 8;
  glm::uvec2 padding{0};
};
struct alignas(16) HddagiLightStoreParams {
  glm::ivec3 grid{0};
  uint32_t capacity = 0;
  glm::ivec3 offset{0};
  uint32_t cascade = 0;
  glm::ivec3 limit{0};
  uint32_t region_index = 0;
  glm::ivec3 region_world_offset{0};
  uint32_t padding2 = 0;
};
struct alignas(16) HddagiUpdateBounds {
  glm::ivec3 begin{0};
  uint32_t cascade = 0;
  glm::ivec3 end{0};
  uint32_t padding = 0;
};
struct alignas(16) HddagiScrollParams {
  glm::ivec3 grid{0};
  uint32_t capacity = 0;
  glm::ivec3 scroll{0};
  uint32_t cascade = 0;
  uint32_t region_count = 0;
  glm::uvec3 padding{0};
};
static_assert(sizeof(HddagiUpdateBounds) == 32 && sizeof(HddagiScrollParams) == 48);
struct alignas(16) HddagiResetParams {
  glm::ivec3 probe_size{0};
  uint32_t cascade = 0;
  glm::ivec3 region_offset{0};
  uint32_t history_size = 0;
  glm::ivec3 scroll{0};
  uint32_t reset_all = 0;
};
static_assert(sizeof(HddagiResetParams) == 48);
struct HddagiProcessVoxel {
  uint32_t position = 0;
  uint32_t albedo_normal = 0;
  uint32_t emission = 0;
  uint32_t occlusion = 0;
};
static_assert(sizeof(HddagiLightStoreParams) == 64 && sizeof(HddagiProcessVoxel) == 16);
static_assert(sizeof(HddagiCascadeData) == 32 && sizeof(HddagiCascadeBlock) == 256);
static_assert(sizeof(HddagiRegionParams) == 48 && sizeof(HddagiRay) == 32 && sizeof(HddagiTraceParams) == 32);
}  // namespace evo_engine
