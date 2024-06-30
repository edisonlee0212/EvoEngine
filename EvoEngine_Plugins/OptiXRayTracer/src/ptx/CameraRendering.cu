#include "RayFunctions.cuh"

namespace evo_engine {
extern "C" __constant__ CameraRenderingLaunchParams cameraRenderingLaunchParams;
#pragma region Closest hit functions
extern "C" __global__ void __closesthit__CR_R() {
  ClosestHitFunc(cameraRenderingLaunchParams.m_rayTracerProperties, cameraRenderingLaunchParams.m_traversable);
}

extern "C" __global__ void __closesthit__CR_SS() {
  SSHit();
}
#pragma endregion
#pragma region Any hit functions

extern "C" __global__ void __anyhit__CR_R() {
  AnyHitFunc();
}

extern "C" __global__ void __anyhit__CR_SS() {
  SSAnyHit();
}
#pragma endregion
#pragma region Miss functions
extern "C" __global__ void __miss__CR_R() {
  MissFunc(cameraRenderingLaunchParams.m_rayTracerProperties);
}
extern "C" __global__ void __miss__CR_SS() {
}
#pragma endregion
#pragma region Main ray generation
extern "C" __global__ void __raygen__CR() {
  float ix = optixGetLaunchIndex().x;
  float iy = optixGetLaunchIndex().y;
  const uint32_t fbIndex = ix + iy * cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x;

  // compute a test pattern based on pixel ID

  PerRayData<glm::vec3> cameraRayData;
  cameraRayData.hit_count = 0;
  cameraRayData.random.Init(ix + cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x * iy,
                            cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_frameId);
  cameraRayData.energy = glm::vec3(0);
  cameraRayData.normal = glm::vec3(0);
  cameraRayData.albedo = glm::vec3(0);
  cameraRayData.position = glm::vec3(999999.0f);
  // the values we store the PRD pointer in:
  uint32_t u0, u1;
  PackRayDataPointer(&cameraRayData, u0, u1);

  const auto numPixelSamples = cameraRenderingLaunchParams.m_rayTracerProperties.m_rayProperties.m_samples;
  auto pixelColor = glm::vec3(0.f);
  auto pixelNormal = glm::vec3(0.f);
  auto pixelAlbedo = glm::vec3(0.f);
  auto pixelPosition = glm::vec3(0.0f);

  float halfX = cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.x * .5f;
  float halfY = cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_size.y * .5f;

  for (int sampleID = 0; sampleID < numPixelSamples; sampleID++) {
    glm::vec2 screen =
        glm::vec2((ix + cameraRayData.random() - halfX) / halfX, (iy + cameraRayData.random() - halfY) / halfY);
    glm::vec4 start = cameraRenderingLaunchParams.m_cameraProperties.m_inverseProjectionView *
                      glm::vec4(screen.x, screen.y, -1.0f, 1.0f);
    glm::vec4 end = cameraRenderingLaunchParams.m_cameraProperties.m_inverseProjectionView *
                    glm::vec4(screen.x, screen.y, 1.0f, 1.0f);
    start /= start.w;
    end /= end.w;
    glm::vec3 rayStart = start;
    glm::vec3 rayEnd = end;
    glm::vec3 primaryRayDir = glm::normalize(rayEnd - rayStart);
    glm::vec3 convergence = rayStart + primaryRayDir * cameraRenderingLaunchParams.m_cameraProperties.m_focalLength;
    float angle = cameraRayData.random() * 3.1415927f * 2.0f;
    glm::vec3 aperturePoint =
        rayStart + cameraRenderingLaunchParams.m_cameraProperties.m_aperture *
                       (cameraRenderingLaunchParams.m_cameraProperties.m_horizontal * glm::sin(angle) +
                        cameraRenderingLaunchParams.m_cameraProperties.m_vertical * glm::cos(angle));
    glm::vec3 rayDir = glm::normalize(convergence - aperturePoint);
    float3 rayOrigin = make_float3(aperturePoint.x, aperturePoint.y, aperturePoint.z);
    float3 rayDirection = make_float3(rayDir.x, rayDir.y, rayDir.z);

    optixTrace(cameraRenderingLaunchParams.m_traversable, rayOrigin, rayDirection,
               0.f,    // tmin
               1e20f,  // tmax
               0.0f,   // rayTime
               static_cast<OptixVisibilityMask>(255),
               OPTIX_RAY_FLAG_NONE,                      // OPTIX_RAY_FLAG_NONE,
               static_cast<int>(RayType::Radiance),      // SBT offset
               static_cast<int>(RayType::RayTypeCount),  // SBT stride
               static_cast<int>(RayType::Radiance),      // missSBTIndex
               u0, u1);
    pixelColor += cameraRayData.energy / static_cast<float>(numPixelSamples);
    pixelNormal += cameraRayData.normal / static_cast<float>(numPixelSamples);
    pixelAlbedo += cameraRayData.albedo / static_cast<float>(numPixelSamples);
    pixelPosition += cameraRayData.position / static_cast<float>(numPixelSamples);
    cameraRayData.energy = glm::vec3(0.0f);
    cameraRayData.normal = glm::vec3(0.0f);
    cameraRayData.albedo = glm::vec3(0.0f);
    cameraRayData.position = glm::vec3(0.0f);
    cameraRayData.hit_count = 0;
  }

  // and write/accumulate to frame buffer ...
  if (cameraRenderingLaunchParams.m_cameraProperties.m_accumulate) {
    if (cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_frameId > 1) {
      glm::vec3 currentGammaCorrectedColor =
          cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_colorBuffer[fbIndex];
      glm::vec3 accumulatedColor = glm::vec3(
          glm::pow(currentGammaCorrectedColor, glm::vec3(cameraRenderingLaunchParams.m_cameraProperties.m_gamma)));
      pixelColor +=
          static_cast<float>(cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_frameId) * accumulatedColor;
      pixelColor /= static_cast<float>(cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_frameId + 1);
    }
  }
  auto gammaCorrectedColor =
      glm::pow(pixelColor, glm::vec3(1.0 / cameraRenderingLaunchParams.m_cameraProperties.m_gamma));
  // and write to frame buffer ...
  cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_colorBuffer[fbIndex] = glm::vec4(gammaCorrectedColor, 1.0f);
  if (cameraRenderingLaunchParams.m_cameraProperties.m_outputType == OutputType::Depth) {
    float distance = glm::distance(cameraRenderingLaunchParams.m_cameraProperties.m_from, pixelPosition);
    cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_albedoBuffer[fbIndex] = glm::vec4(
        glm::vec3(glm::clamp(distance / cameraRenderingLaunchParams.m_cameraProperties.m_maxDistance, 0.0f, 1.0f)),
        1.0f);
  } else {
    cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_albedoBuffer[fbIndex] = glm::vec4(pixelAlbedo, 1.0f);
  }
  cameraRenderingLaunchParams.m_cameraProperties.m_frame.m_normalBuffer[fbIndex] = glm::vec4(pixelNormal, 1.0f);
}
#pragma endregion
}  // namespace evo_engine
