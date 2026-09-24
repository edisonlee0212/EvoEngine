#pragma once

#include <cstddef>
#include <cstdint>

namespace evo_engine {

enum class RestirPtPathType : uint32_t {
  Invalid = 0u,
  PrimaryEmission = 1u,
  BsdfEmission = 2u,
  BsdfEnvironment = 3u,
  NeeEmission = 4u,
  NeeEnvironment = 5u,
  PrimaryBackground = 6u,
};

// Byte-addressed GPU layout; keep in sync with EvoEngine.RestirPt.
struct RestirPtPathReservoir {
  static constexpr uint32_t kVersion = 1u;
  static constexpr uint32_t kInvalidInstance = UINT32_MAX;
  static constexpr uint32_t kDeltaPrefix = 1u;

  uint32_t version = kVersion;
  uint32_t path_type = 0u;
  uint32_t sample_seed = 0u;
  uint32_t path_length = 0u;

  uint32_t reconnection_length = 0u;
  uint32_t instance_id = kInvalidInstance;
  uint32_t primitive_id = 0u;
  uint32_t flags = 0u;

  float barycentrics[2]{};
  float light_pdf = 0.0f;
  float bsdf_pdf = 0.0f;

  float contribution[3]{};
  float target_density = 0.0f;

  float reconnection_direction[3]{};
  float jacobian = 0.0f;

  float reconnection_position[3]{};
  float effective_count = 0.0f;

  float reconnection_normal[3]{};
  float weight_sum = 0.0f;

  float primary_position[3]{};
  float selected_weight = 0.0f;
};

static_assert(sizeof(RestirPtPathReservoir) == 128u);
static_assert(offsetof(RestirPtPathReservoir, barycentrics) == 32u);
static_assert(offsetof(RestirPtPathReservoir, contribution) == 48u);
static_assert(offsetof(RestirPtPathReservoir, reconnection_direction) == 64u);
static_assert(offsetof(RestirPtPathReservoir, reconnection_position) == 80u);
static_assert(offsetof(RestirPtPathReservoir, reconnection_normal) == 96u);
static_assert(offsetof(RestirPtPathReservoir, primary_position) == 112u);

struct RestirPtPrimarySurface {
  float position[3]{};
  float depth = 0.0f;
  float normal[3]{};
  uint32_t material_id = UINT32_MAX;
};

static_assert(sizeof(RestirPtPrimarySurface) == 32u);

struct RestirPtSpatialShift {
  float contribution[3]{};
  float weight_sum = 0.0f;
  uint32_t partner_index = UINT32_MAX;
  uint32_t status = 0u;
  uint32_t ray_count = 0u;
  uint32_t reserved = 0u;
};

static_assert(sizeof(RestirPtSpatialShift) == 32u);

}  // namespace evo_engine
