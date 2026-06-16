#include "RayFunctions.cuh"

namespace evo_engine {
extern "C" __constant__ IlluminationEstimationLaunchParams illuminationEstimationLaunchParams;

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
extern "C" __global__ void __closesthit__IE_R() {
  ClosestHitFunc(illuminationEstimationLaunchParams.ray_tracer_properties,
                 illuminationEstimationLaunchParams.traversable);
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
}
extern "C" __global__ void __miss__IE_SS() {
}
#pragma endregion
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

  auto pointEnergy = glm::vec3(0.0f);
  auto pointDirection = glm::vec3(0.0f);

  PerRayData<glm::vec3> perRayData;
  perRayData.random.Init(ix, illuminationEstimationLaunchParams.seed);
  uint32_t u0, u1;
  PackRayDataPointer(&perRayData, u0, u1);
  int sampleSize = 0;
  if (frontFace) {
    for (int sampleID = 0; sampleID < numPointSamples; sampleID++) {
      perRayData.energy = glm::vec3(0.0f);
      perRayData.hit_count = 0;
      glm::vec3 rayDir, rayOrigin;
      glm::vec3 position;
      glm::vec3 normal;
      SampleProbePoint(a, b, c, perRayData.random, position, normal);
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
      glm::vec3 position;
      glm::vec3 normal;
      SampleProbePoint(a, b, c, perRayData.random, position, normal);
      normal = -normal;
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
      auto energy = perRayData.energy * glm::abs(glm::dot(normal, rayDir));
      pointEnergy += energy;
      pointDirection += rayDir * glm::length(energy);
    }
    sampleSize += numPointSamples;
  }
  if (sampleSize != 0) {
    probe.energy = pointEnergy / ((float)sampleSize);
    probe.direction = glm::normalize(pointDirection);
  }
}
#pragma endregion
}  // namespace evo_engine
