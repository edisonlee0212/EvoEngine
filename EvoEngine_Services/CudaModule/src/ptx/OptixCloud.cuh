#pragma once
#include <cuda_runtime.h>
#include <cstdint>
#include <cstdio>

// The generated Slang prelude and the OptiX SDK each define CUDA vector helpers.
namespace cloud_shader {
using ::__ldg;
using ::atomicAdd;
using ::atomicCAS;
using ::atomicExch;
#define IMPORT_CUDA_MAKE(T) using ::make_##T##2; using ::make_##T##3; using ::make_##T##4
IMPORT_CUDA_MAKE(char);
IMPORT_CUDA_MAKE(uchar);
IMPORT_CUDA_MAKE(short);
IMPORT_CUDA_MAKE(ushort);
IMPORT_CUDA_MAKE(int);
IMPORT_CUDA_MAKE(uint);
IMPORT_CUDA_MAKE(longlong);
IMPORT_CUDA_MAKE(ulonglong);
IMPORT_CUDA_MAKE(float);
IMPORT_CUDA_MAKE(double);
#undef IMPORT_CUDA_MAKE
#include "CameraCloud.generated.cuh"
}

#include "RayTracerUtilities.cuh"

namespace evo_engine {
static __forceinline__ __device__ float3 CloudFloat3(glm::vec3 v) {
  return make_float3(v.x, v.y, v.z);
}
template <typename LaunchParams>
static __forceinline__ __device__ glm::vec4 EvaluateCloud(
    const LaunchParams& params, float3 rayOrigin, float3 rayDirection, float distance,
    float jitter, glm::vec4 color, bool hit, bool use_cache, glm::vec4 cached, float& depth,
    bool prepass, int debug_mode, float shadow_strength, bool transmittance_only = false) {
  const auto& s = params.clouds;
  const auto& env = params.ray_tracer_properties.environment;
  const float4 result = cloud_shader::marchCameraCloud(
      params.cloud_noise[0], params.cloud_noise[1], params.cloud_noise[2], params.cloud_noise[3],
      rayOrigin, rayDirection, distance, CloudFloat3(env.sun_direction), CloudFloat3(env.sun_color * env.sun_intensity),
      CloudFloat3(env.color * env.skylight_intensity), jitter,
      make_float4(s.bottom_altitude, s.top_altitude, s.coverage, s.density),
      make_float4(s.base_noise_scale, s.detail_noise_scale, s.extinction_scale, s.max_march_distance),
      make_float4(float(s.use_spherical_atmosphere), s.atmosphere_radius, s.cloud_type, s.curl_strength),
      make_float4(s.wind_direction.x, s.wind_direction.y, s.wind_speed, 0.0f),
      make_float4(s.lighting_intensity, s.ambient_lighting_strength, s.phase_anisotropy, 0.0f),
      make_float2(s.coarse_step_fraction, s.fine_step_scale),
      make_int3(s.primary_step_count, s.light_step_count, s.empty_step_fallback_count),
      make_float4(color.r, color.g, color.b, color.a), hit,
      shadow_strength, s.cloud_shadow_step_count,
      debug_mode, prepass, use_cache, transmittance_only,
      make_float4(cached.x, cached.y, cached.z, cached.w), &depth);
  return glm::vec4(result.x, result.y, result.z, result.w);
}

}  // namespace evo_engine
