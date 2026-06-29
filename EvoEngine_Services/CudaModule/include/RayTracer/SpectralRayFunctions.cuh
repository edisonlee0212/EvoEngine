#pragma once

#include "BSSDF.cuh"
#include "RayTracerUtilities.cuh"

namespace evo_engine {

static __forceinline__ __device__ void SpectralAnyHitFunc() {
  // TODO(illumination-spectral): Add wavelength-aware alpha/transmission handling.
}

static __forceinline__ __device__ void SpectralClosestHitFunc() {
  const float3 ray_direction_internal = optixGetWorldRayDirection();
  glm::vec3 ray_direction = glm::vec3(ray_direction_internal.x, ray_direction_internal.y, ray_direction_internal.z);

  const auto &sbt_data = *(const SBT *)optixGetSbtDataPointer();
  const auto hit_info = sbt_data.GetHitInfo(ray_direction);
  auto &per_ray_data = *GetRayDataPointer<PerRayData<glm::vec3>>();

  per_ray_data.hit_count += 1;
  per_ray_data.energy = glm::vec3(0.0f);

  if (per_ray_data.hit_count == 1) {
    per_ray_data.normal = hit_info.normal;
    per_ray_data.albedo = hit_info.color;
    per_ray_data.position = hit_info.position;
  }

  // TODO(illumination-spectral): Replace RGB placeholders with spectral transport.
}

static __forceinline__ __device__ glm::vec3 SampleSpectralEnvironment(const EnvironmentProperties &environment,
                                                                      const glm::vec3 &ray_direction) {
  if (environment.use_environmental_map && environment.environmental_map) {
    const float4 color =
        texCubemap<float4>(environment.environmental_map, ray_direction.x, ray_direction.y, ray_direction.z);
    return glm::vec3(color.x, color.y, color.z);
  }
  return environment.color;
}

static __forceinline__ __device__ void SpectralMissFunc(const EnvironmentProperties &environment) {
  const float3 ray_direction_internal = optixGetWorldRayDirection();
  const glm::vec3 ray_direction =
      glm::vec3(ray_direction_internal.x, ray_direction_internal.y, ray_direction_internal.z);
  auto &per_ray_data = *GetRayDataPointer<PerRayData<glm::vec3>>();
  per_ray_data.energy = SampleSpectralEnvironment(environment, ray_direction);
  per_ray_data.albedo = per_ray_data.energy;
}

}  // namespace evo_engine
