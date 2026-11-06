#include "OptixCloud.cuh"
#include "RayFunctions.cuh"

namespace evo_engine {
extern "C" __constant__ IlluminationEstimationLaunchParams illuminationEstimationLaunchParams;
// Integrate the current world-space segment, without consuming the probe's direction-sampling RNG.
static __forceinline__ __device__ void ApplyProbeCloud(float distance, bool visibility_ray) {
  const auto &params = illuminationEstimationLaunchParams;
  if (!params.clouds.enabled || distance <= 0.f)
    return;
  auto &prd = *GetRayDataPointer<PerRayData<glm::vec3>>();
  auto jitter = prd.random;
  float cloud_depth;
  const auto cloud = EvaluateCloud(params, optixGetWorldRayOrigin(), optixGetWorldRayDirection(), distance,
                                   jitter(), glm::vec4(0.f), false, false, glm::vec4(0.f), cloud_depth,
                                   true, 0, 0.f, visibility_ray);
  prd.energy *= cloud.w;
  if (visibility_ray)
    prd.visibility = cloud.w;
  else
    prd.energy += glm::vec3(cloud);
}

#pragma region Closest hit functions
extern "C" __global__ void __closesthit__IE_R() {
  ClosestHitFunc(illuminationEstimationLaunchParams.ray_tracer_properties,
                 illuminationEstimationLaunchParams.traversable);
  ApplyProbeCloud(optixGetRayTmax(), false);
}
extern "C" __global__ void __closesthit__IE_SS() {
  SSHit();
}
#pragma endregion
#pragma region Any hit functions
extern "C" __global__ void __anyhit__IE_R() {
  AnyHitFunc();
}
extern "C" __global__ void __anyhit__IE_SS() {
  SSAnyHit();
}
#pragma endregion
#pragma region Miss functions
extern "C" __global__ void __miss__IE_R() {
  MissFunc(illuminationEstimationLaunchParams.ray_tracer_properties);
  ApplyProbeCloud(illuminationEstimationLaunchParams.clouds.max_march_distance,
                  (optixGetRayFlags() & OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT) != 0);
}
extern "C" __global__ void __miss__IE_SS() {
}
#pragma endregion

static __forceinline__ __device__ void SampleTriangleSurface(const Vertex &a, const Vertex &b, const Vertex &c,
                                                             Random &random, const float normal_sign,
                                                             glm::vec3 &position, glm::vec3 &normal) {
  // Square-root barycentric sampling is uniform over the triangle and never leaves its boundary.
  const float r1 = random();
  const float r2 = random();
  const float sqrt_r1 = glm::sqrt(r1);
  const float w0 = 1.0f - sqrt_r1;
  const float w1 = sqrt_r1 * (1.0f - r2);
  const float w2 = sqrt_r1 * r2;
  position = w0 * a.position + w1 * b.position + w2 * c.position;
  normal = normal_sign * glm::normalize(w0 * a.normal + w1 * b.normal + w2 * c.normal);
}

static __forceinline__ __device__ glm::vec3 EstimateDirectSun(PerRayData<glm::vec3> &per_ray_data,
                                                              const glm::vec3 &ray_origin, const glm::vec3 &normal,
                                                              uint32_t u0, uint32_t u1, glm::vec3 &sun_direction) {
  const auto &params = illuminationEstimationLaunchParams;
  const auto &environment = params.ray_tracer_properties.environment;
  sun_direction = glm::vec3(0.0f);
  if (environment.environmental_lighting_type != EnvironmentalLightingType::Skydome ||
      environment.sun_intensity <= 0.0f)
    return glm::vec3(0.0f);

  const float sun_radius = 0.5f * environment.sun_angular_diameter_radians;
  const float sample_alpha = 1.0f - glm::sqrt(glm::max(1.0f - glm::cos(sun_radius), 0.0f));
  sun_direction = RandomSampleHemisphere(per_ray_data.random, environment.sun_direction, sample_alpha);
  const float n_dot_l = glm::dot(normal, sun_direction);
  if (n_dot_l <= 0.0f)
    return glm::vec3(0.0f);

  // Keep any-hit enabled so alpha-masked leaf pixels do not cast opaque rectangular shadows.
  per_ray_data.energy = glm::vec3(0.0f);
  per_ray_data.visibility = 0.f;
  optixTrace(params.traversable, make_float3(ray_origin.x, ray_origin.y, ray_origin.z),
             make_float3(sun_direction.x, sun_direction.y, sun_direction.z), 1e-4f, 1e20f, 0.0f,
             static_cast<OptixVisibilityMask>(255),
             OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT | OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT,
             static_cast<int>(RayType::Radiance), static_cast<int>(RayType::RayTypeCount),
             static_cast<int>(RayType::Radiance), u0, u1);
  return per_ray_data.visibility * environment.sun_color * environment.sun_intensity * n_dot_l;
}

#pragma region Main ray generation
extern "C" __global__ void __raygen__IE() {
  unsigned ix = optixGetLaunchIndex().x;
  const auto numPointSamples = illuminationEstimationLaunchParams.ray_tracer_properties.ray_properties.samples;
  auto &probe = illuminationEstimationLaunchParams.light_probes[ix];
  const auto &a = probe.v_0;
  const auto &b = probe.v_1;
  const auto &c = probe.v_2;
  const auto &pushDistance = illuminationEstimationLaunchParams.push_normal_distance;
  const auto frontFace = probe.front_face;
  const auto backFace = probe.back_face;

  glm::vec3 direct_energy(0.f);
  auto pointEnergy = glm::vec3(0.0f);
  auto pointDirection = glm::vec3(0.0f);

  PerRayData<glm::vec3> perRayData;
  perRayData.random.Init(ix, illuminationEstimationLaunchParams.seed);
  uint32_t u0, u1;
  PackRayDataPointer(&perRayData, u0, u1);
  int sampleSize = 0;
  const float pdf = 1.0f / (2.0f * glm::pi<float>());

  for (int face = 0; face < 2; ++face) {
    if (!(face == 0 ? frontFace : backFace))
      continue;
    for (int sampleID = 0; sampleID < numPointSamples; sampleID++) {
      perRayData.energy = glm::vec3(0.0f);
      perRayData.hit_count = 0;
      perRayData.diffuse_indirect_path = true;
      glm::vec3 rayDir, rayOrigin;
      glm::vec3 position, normal;
      SampleTriangleSurface(a, b, c, perRayData.random, face == 0 ? 1.f : -1.f, position, normal);
      rayDir = RandomSampleHemisphere(perRayData.random, normal);
      rayOrigin = position + normal * pushDistance;
      float3 rayOriginInternal = make_float3(rayOrigin.x, rayOrigin.y, rayOrigin.z);
      float3 rayDirection = make_float3(rayDir.x, rayDir.y, rayDir.z);
      optixTrace(illuminationEstimationLaunchParams.traversable, rayOriginInternal, rayDirection,
                 1e-3f,  // tmin
                 1e20f,  // tmax
                 0.0f,   // rayTime
                 static_cast<OptixVisibilityMask>(255),
                 OPTIX_RAY_FLAG_NONE,                      // OPTIX_RAY_FLAG_NONE,
                 static_cast<int>(RayType::Radiance),      // SBT offset
                 static_cast<int>(RayType::RayTypeCount),  // SBT stride
                 static_cast<int>(RayType::Radiance),      // missSBTIndex
                 u0, u1);
      auto energy = perRayData.energy * glm::abs(glm::dot(normal, rayDir)) / pdf;
      pointDirection += rayDir * glm::length(energy);
      glm::vec3 sun_direction;
      const auto direct_sun = EstimateDirectSun(perRayData, rayOrigin, normal, u0, u1, sun_direction);
      direct_energy += direct_sun;
      energy += direct_sun;
      pointEnergy += energy;
      pointDirection += sun_direction * glm::length(direct_sun);
    }
    sampleSize += numPointSamples;
  }
  if (sampleSize != 0) {
    if (illuminationEstimationLaunchParams.direct_sun)
      illuminationEstimationLaunchParams.direct_sun[ix] = direct_energy / float(sampleSize);
    probe.energy = pointEnergy / ((float)sampleSize);
    probe.direction = glm::length(pointDirection) > 0.0f ? glm::normalize(pointDirection) : glm::vec3(0.0f);
  }
}
#pragma endregion
}  // namespace evo_engine
