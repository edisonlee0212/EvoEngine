#pragma once

#include <cstddef>
#include <cstdint>

namespace evo_engine {

// Logical shader sample; its first 96 bytes match the Candidate Only GPU record.
struct RestirSssDirectSample {
  uint32_t receiver_instance = UINT32_MAX;
  uint32_t receiver_primitive = 0;
  float receiver_barycentrics[2]{};
  uint32_t sample_instance = UINT32_MAX;
  uint32_t sample_primitive = 0;
  float sample_barycentrics[2]{};
  uint32_t sampling_seed = 0;
  uint32_t axis_channel = 0;
  uint32_t intersection_index = 0;
  uint32_t intersection_count = 0;
  float probe_direction[3]{};
  float area_pdf = 0;
  uint32_t light_technique = UINT32_MAX;
  uint32_t light_seed = 0;
  uint32_t bsdf_seed = 0;
  uint32_t shadow_seed = 0;
  // Candidate: weighted throughput/local RIS weight. History: receiver factor/GRIS weight.
  float throughput[3]{};
  float resolve_weight = 0;
  float receiver_position[3]{};
  float confidence = 0;
  float receiver_normal[3]{};
  uint32_t receiver_material = UINT32_MAX;
  float receiver_tangent[3]{};
  float target_density = 0;
  uint32_t origin_pixel = UINT32_MAX;
  uint32_t origin_frame = 0;
};

static_assert(sizeof(RestirSssDirectSample) == 152u);
inline constexpr uint32_t kRestirSssCandidateBytes = 96u;
inline constexpr uint32_t kRestirSssHistoryBytes = 112u;
inline constexpr uint32_t kRestirSssResidualBytes = 64u;
static_assert(offsetof(RestirSssDirectSample, sample_instance) == 16u);
static_assert(offsetof(RestirSssDirectSample, sampling_seed) == 32u);
static_assert(offsetof(RestirSssDirectSample, probe_direction) == 48u);
static_assert(offsetof(RestirSssDirectSample, light_technique) == 64u);
static_assert(offsetof(RestirSssDirectSample, throughput) == 80u);
static_assert(offsetof(RestirSssDirectSample, receiver_position) == kRestirSssCandidateBytes);
static_assert(offsetof(RestirSssDirectSample, receiver_normal) == 112u);
static_assert(offsetof(RestirSssDirectSample, receiver_tangent) == 128u);

}  // namespace evo_engine
