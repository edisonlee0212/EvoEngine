#include "BSSDF.cuh"
#include "RayTracerUtilities.cuh"
#include "SpectralRayFunctions.cuh"

namespace evo_engine {
extern "C" __constant__ IlluminationEstimationSpectralLaunchParams illuminationEstimationSpectralLaunchParams;

static __forceinline__ __device__ void SampleProbePoint(const Vertex &a, const Vertex &b, const Vertex &c,
                                                        Random &random, glm::vec3 &position, glm::vec3 &normal) {
  float coord_a = random();
  float coord_b = random();
  if (coord_a + coord_b > 1.0f) {
    coord_a = 1.0f - coord_a;
    coord_b = 1.0f - coord_b;
  }
  const float coord_c = 1.0f - coord_a - coord_b;
  position = coord_c * a.position + coord_a * b.position + coord_b * c.position;
  normal = glm::normalize(coord_c * a.normal + coord_a * b.normal + coord_b * c.normal);
}

#pragma region Closest hit functions
extern "C" __global__ void __closesthit__IES_R() {
  SpectralClosestHitFunc();
}

extern "C" __global__ void __closesthit__IES_SS() {
  SSHit();
}
#pragma endregion

#pragma region Any hit functions
extern "C" __global__ void __anyhit__IES_R() {
  SpectralAnyHitFunc();
}

extern "C" __global__ void __anyhit__IES_SS() {
  SSAnyHit();
}
#pragma endregion

#pragma region Miss functions
extern "C" __global__ void __miss__IES_R() {
  SpectralMissFunc(illuminationEstimationSpectralLaunchParams.ray_tracer_properties.environment);
}

extern "C" __global__ void __miss__IES_SS() {
}
#pragma endregion

#pragma region Main ray generation
extern "C" __global__ void __raygen__IES() {
  const unsigned ix = optixGetLaunchIndex().x;
  const auto num_point_samples = illuminationEstimationSpectralLaunchParams.ray_tracer_properties.ray_properties.samples;
  auto &probe = illuminationEstimationSpectralLaunchParams.light_probes[ix];
  const auto &a = probe.v_0;
  const auto &b = probe.v_1;
  const auto &c = probe.v_2;
  const auto push_distance = illuminationEstimationSpectralLaunchParams.push_normal_distance;

  glm::vec3 point_energy(0.0f);
  glm::vec3 point_direction(0.0f);

  PerRayData<glm::vec3> per_ray_data;
  per_ray_data.random.Init(ix, illuminationEstimationSpectralLaunchParams.seed);
  per_ray_data.hit_count = 0;
  per_ray_data.energy = glm::vec3(0.0f);
  per_ray_data.normal = glm::vec3(0.0f);
  per_ray_data.albedo = glm::vec3(0.0f);
  per_ray_data.position = glm::vec3(0.0f);

  uint32_t u0, u1;
  PackRayDataPointer(&per_ray_data, u0, u1);

  int sample_size = 0;
  if (probe.front_face) {
    for (int sample_id = 0; sample_id < num_point_samples; sample_id++) {
      per_ray_data.energy = glm::vec3(0.0f);
      per_ray_data.hit_count = 0;

      glm::vec3 position;
      glm::vec3 normal;
      SampleProbePoint(a, b, c, per_ray_data.random, position, normal);
      const glm::vec3 ray_dir = RandomSampleHemisphere(per_ray_data.random, normal);
      const glm::vec3 ray_origin = position + normal * push_distance;

      optixTrace(illuminationEstimationSpectralLaunchParams.traversable,
                 make_float3(ray_origin.x, ray_origin.y, ray_origin.z), make_float3(ray_dir.x, ray_dir.y, ray_dir.z),
                 1e-3f, 1e20f, 0.0f, static_cast<OptixVisibilityMask>(255), OPTIX_RAY_FLAG_NONE,
                 static_cast<int>(RayType::Radiance), static_cast<int>(RayType::RayTypeCount),
                 static_cast<int>(RayType::Radiance), u0, u1);

      const glm::vec3 energy = per_ray_data.energy * glm::abs(glm::dot(normal, ray_dir));
      point_energy += energy;
      point_direction += ray_dir * glm::length(energy);
    }
    sample_size += num_point_samples;
  }
  if (probe.back_face) {
    for (int sample_id = 0; sample_id < num_point_samples; sample_id++) {
      per_ray_data.energy = glm::vec3(0.0f);
      per_ray_data.hit_count = 0;

      glm::vec3 position;
      glm::vec3 normal;
      SampleProbePoint(a, b, c, per_ray_data.random, position, normal);
      normal = -normal;
      const glm::vec3 ray_dir = RandomSampleHemisphere(per_ray_data.random, normal);
      const glm::vec3 ray_origin = position + normal * push_distance;

      optixTrace(illuminationEstimationSpectralLaunchParams.traversable,
                 make_float3(ray_origin.x, ray_origin.y, ray_origin.z), make_float3(ray_dir.x, ray_dir.y, ray_dir.z),
                 1e-3f, 1e20f, 0.0f, static_cast<OptixVisibilityMask>(255), OPTIX_RAY_FLAG_NONE,
                 static_cast<int>(RayType::Radiance), static_cast<int>(RayType::RayTypeCount),
                 static_cast<int>(RayType::Radiance), u0, u1);

      const glm::vec3 energy = per_ray_data.energy * glm::abs(glm::dot(normal, ray_dir));
      point_energy += energy;
      point_direction += ray_dir * glm::length(energy);
    }
    sample_size += num_point_samples;
  }

  if (sample_size > 0) {
    probe.energy = point_energy / static_cast<float>(sample_size);
    probe.direction = glm::normalize(point_direction);
  }

  // TODO(illumination-spectral): Replace RGB placeholder accumulation with spectral irradiance estimation.
}
#pragma endregion
}  // namespace evo_engine
