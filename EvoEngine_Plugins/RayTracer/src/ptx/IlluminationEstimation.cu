#include "RayFunctions.cuh"

namespace evo_engine {
extern "C" __constant__ IlluminationEstimationLaunchParams illuminationEstimationLaunchParams;
#pragma region Closest hit functions
extern "C" __global__ void __closesthit__IE_R() {
  ClosestHitFunc(illuminationEstimationLaunchParams.m_rayTracerProperties,
                 illuminationEstimationLaunchParams.m_traversable);
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
  MissFunc(illuminationEstimationLaunchParams.m_rayTracerProperties);
}
extern "C" __global__ void __miss__IE_SS() {
}
#pragma endregion
#pragma region Main ray generation
extern "C" __global__ void __raygen__IE() {
  unsigned ix = optixGetLaunchIndex().x;
  const auto numPointSamples = illuminationEstimationLaunchParams.m_rayTracerProperties.m_rayProperties.m_samples;
  auto &probe = illuminationEstimationLaunchParams.m_lightProbes[ix];
  const auto &a = probe.m_a;
  const auto &b = probe.m_b;
  const auto &c = probe.m_c;
  const auto &pushDistance = illuminationEstimationLaunchParams.m_pushNormalDistance;
  const auto frontFace = probe.m_frontFace;
  const auto backFace = probe.m_backFace;

  auto pointEnergy = glm::vec3(0.0f);
  auto pointDirection = glm::vec3(0.0f);

  PerRayData<glm::vec3> perRayData;
  perRayData.random.Init(ix, illuminationEstimationLaunchParams.m_seed);
  uint32_t u0, u1;
  PackRayDataPointer(&perRayData, u0, u1);
  int sampleSize = 0;
  if (frontFace) {
    for (int sampleID = 0; sampleID < numPointSamples; sampleID++) {
      perRayData.energy = glm::vec3(0.0f);
      perRayData.hit_count = 0;
      glm::vec3 rayDir, rayOrigin;
      float coordA = perRayData.random();
      float coordB = perRayData.random();
      glm::vec3 position = (1.f - coordA - coordB) * a.position + coordA * b.position + coordB * c.position;
      glm::vec3 normal = (1.f - coordA - coordB) * a.normal + coordA * b.normal + coordB * c.normal;
      rayDir = RandomSampleHemisphere(perRayData.random, normal);
      rayOrigin = position + normal * pushDistance;
      float3 rayOriginInternal = make_float3(rayOrigin.x, rayOrigin.y, rayOrigin.z);
      float3 rayDirection = make_float3(rayDir.x, rayDir.y, rayDir.z);
      optixTrace(illuminationEstimationLaunchParams.m_traversable, rayOriginInternal, rayDirection,
                 1e-3f,  // tmin
                 1e20f,  // tmax
                 0.0f,   // rayTime
                 static_cast<OptixVisibilityMask>(255),
                 OPTIX_RAY_FLAG_NONE,                      // OPTIX_RAY_FLAG_NONE,
                 static_cast<int>(RayType::Radiance),      // SBT offset
                 static_cast<int>(RayType::RayTypeCount),  // SBT stride
                 static_cast<int>(RayType::Radiance),      // missSBTIndex
                 u0, u1);
      auto energy = perRayData.energy * glm::abs(glm::dot(normal, rayDir));
      pointEnergy += energy;
      pointDirection += rayDir * glm::length(energy);
    }
    sampleSize += numPointSamples;
  }
  if (backFace) {
    for (int sampleID = 0; sampleID < numPointSamples; sampleID++) {
      perRayData.energy = glm::vec3(0.0f);
      perRayData.hit_count = 0;
      glm::vec3 rayDir, rayOrigin;
      float coordA = perRayData.random();
      float coordB = perRayData.random();
      glm::vec3 position = (1.f - coordA - coordB) * a.position + coordA * b.position + coordB * c.position;
      glm::vec3 normal = -(1.f - coordA - coordB) * a.normal - coordA * b.normal - coordB * c.normal;
      rayDir = RandomSampleHemisphere(perRayData.random, normal);
      rayOrigin = position + normal * pushDistance;
      float3 rayOriginInternal = make_float3(rayOrigin.x, rayOrigin.y, rayOrigin.z);
      float3 rayDirection = make_float3(rayDir.x, rayDir.y, rayDir.z);
      optixTrace(illuminationEstimationLaunchParams.m_traversable, rayOriginInternal, rayDirection,
                 1e-3f,  // tmin
                 1e20f,  // tmax
                 0.0f,   // rayTime
                 static_cast<OptixVisibilityMask>(255),
                 OPTIX_RAY_FLAG_NONE,                      // OPTIX_RAY_FLAG_NONE,
                 static_cast<int>(RayType::Radiance),      // SBT offset
                 static_cast<int>(RayType::RayTypeCount),  // SBT stride
                 static_cast<int>(RayType::Radiance),      // missSBTIndex
                 u0, u1);
      auto energy = perRayData.energy * glm::abs(glm::dot(normal, rayDir));
      pointEnergy += energy;
      pointDirection += rayDir * glm::length(energy);
    }
    sampleSize += numPointSamples;
  }
  if (sampleSize != 0) {
    probe.m_energy = pointEnergy / ((float)sampleSize);
    probe.m_direction = glm::normalize(pointDirection);
  }
}
#pragma endregion
}  // namespace evo_engine
