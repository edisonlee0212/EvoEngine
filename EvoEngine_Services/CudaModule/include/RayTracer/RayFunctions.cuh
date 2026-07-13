#pragma once

#include "BSDF.cuh"
#include "BSSDF.cuh"
#include "Environment.cuh"
namespace evo_engine {
static __forceinline__ __device__ bool TraceSkydomeSun(const EnvironmentProperties &environment, Random &random,
                                                       const OptixTraversableHandle traversable,
                                                       const glm::vec3 &position, const glm::vec3 &normal,
                                                       glm::vec3 &direction, float &n_dot_l) {
  if (environment.environmental_lighting_type != EnvironmentalLightingType::Skydome ||
      environment.sun_intensity <= 0.0f)
    return false;
  direction = SampleSunDirection(random, environment.sun_direction, environment.sun_angular_diameter_radians);
  n_dot_l = glm::dot(normal, direction);
  return n_dot_l > 0.0f && TraceVisibility(traversable, position + normal * 1e-3f, direction);
}

static __forceinline__ __device__ void AnyHitFunc() {
  const float3 rayDirectionInternal = optixGetWorldRayDirection();
  glm::vec3 rayDirection = glm::vec3(rayDirectionInternal.x, rayDirectionInternal.y, rayDirectionInternal.z);
#pragma region Retrive information
  const auto &sbtData = *(const SBT *)optixGetSbtDataPointer();
  auto hit_info = sbtData.GetHitInfo(rayDirection);
#pragma endregion
  switch (sbtData.material_type) {
    case MaterialType::Default: {
      PerRayData<glm::vec3> &perRayData = *GetRayDataPointer<PerRayData<glm::vec3>>();
      glm::vec4 albedoColor = static_cast<SurfaceMaterial *>(sbtData.material)->GetAlbedo(hit_info.tex_coord);
      if (albedoColor.w <= perRayData.random())
        optixIgnoreIntersection();
    } break;
  }
}

static __forceinline__ __device__ void ClosestHitFunc(const RayTracerProperties &rayTracerProperties,
                                                      OptixTraversableHandle optixTraversableHandle,
                                                      const bool enableSkydomeSun = false) {
  const float3 rayDirectionInternal = optixGetWorldRayDirection();
  glm::vec3 rayDirection = glm::vec3(rayDirectionInternal.x, rayDirectionInternal.y, rayDirectionInternal.z);
#pragma region Retrive information
  const auto &sbtData = *(const SBT *)optixGetSbtDataPointer();
  auto hit_info = sbtData.GetHitInfo(rayDirection);
#pragma endregion
  PerRayData<glm::vec3> &perRayData = *GetRayDataPointer<PerRayData<glm::vec3>>();
  unsigned hitCount = perRayData.hit_count + 1;

  // start with some ambient term
  auto energy = glm::vec3(0.0f);
  uint32_t u0, u1;
  PackRayDataPointer(&perRayData, u0, u1);
  perRayData.hit_count = hitCount;
  perRayData.energy = glm::vec3(0.0f);
  auto &environment = rayTracerProperties.environment;
  if (sbtData.material_type != MaterialType::CompressedBTF) {
    auto *material = static_cast<SurfaceMaterial *>(sbtData.material);
    material->ApplyNormalTexture(hit_info.normal, hit_info.tex_coord, hit_info.tangent);
    float metallic = material->GetMetallic(hit_info.tex_coord);
    float roughness = material->GetRoughness(hit_info.tex_coord);
    glm::vec3 albedoColor;
    if (sbtData.material_type == MaterialType::Default) {
      albedoColor = material->GetAlbedo(hit_info.tex_coord);
    } else {
      albedoColor = hit_info.color;
    }
    energy = glm::vec3(0.0f);
    float f = 1.0f;
    if (metallic >= 0.0f)
      f = (metallic + 2) / (metallic + 1);
    if (environment.environmental_lighting_type == EnvironmentalLightingType::SingleLightSource) {
      glm::vec3 newRayDirection =
          RandomSampleHemisphere(perRayData.random, environment.sun_direction, 1.0f - environment.light_size);
      energy += glm::vec3(environment.color) * environment.ambient_light_intensity * albedoColor;
      const float NdotL = glm::dot(hit_info.normal, newRayDirection);
      if (NdotL > 0.0f) {
        uint32_t u0, u1;
        PackRayDataPointer(&perRayData, u0, u1);
        perRayData.energy = glm::vec3(0.0f);
        optixTrace(
            optixTraversableHandle, make_float3(hit_info.position.x, hit_info.position.y, hit_info.position.z),
            make_float3(newRayDirection.x, newRayDirection.y, newRayDirection.z),
            1e-3f,  // tmin
            1e20f,  // tmax
            0.0f,   // rayTime
            static_cast<OptixVisibilityMask>(255),
            OPTIX_RAY_FLAG_DISABLE_ANYHIT | OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT | OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT,
            static_cast<int>(RayType::Radiance),      // SBT offset
            static_cast<int>(RayType::RayTypeCount),  // SBT stride
            static_cast<int>(RayType::Radiance),      // missSBTIndex
            u0, u1);
        energy += perRayData.energy * NdotL * albedoColor;
      }
    } else {
      if (enableSkydomeSun && environment.environmental_lighting_type == EnvironmentalLightingType::Skydome) {
        glm::vec3 sun_direction;
        float n_dot_l;
        if (TraceSkydomeSun(environment, perRayData.random, optixTraversableHandle, hit_info.position, hit_info.normal,
                            sun_direction, n_dot_l)) {
          energy += environment.sun_color * environment.sun_intensity * n_dot_l * albedoColor;
        }
      }
      if (perRayData.hit_count <= rayTracerProperties.ray_properties.bounces) {
        bool needSample = false;
        if (hitCount <= 1 && material->material_properties.subsurface_factor > 0.0f &&
            material->material_properties.subsurface_radius.x > 0.0f) {
          float3 incidentRayOrigin;
          float3 newRayDirectionInternal;
          glm::vec3 outNormal;
          needSample = BSSRDF(metallic, perRayData.random, material->material_properties.subsurface_radius.x,
                              sbtData.handle, optixTraversableHandle, hit_info.position, rayDirection, hit_info.normal,
                              incidentRayOrigin, newRayDirectionInternal, outNormal);
          if (needSample) {
            optixTrace(optixTraversableHandle, incidentRayOrigin, newRayDirectionInternal,
                       1e-3f,  // tmin
                       1e20f,  // tmax
                       0.0f,   // rayTime
                       static_cast<OptixVisibilityMask>(255), OPTIX_RAY_FLAG_NONE,
                       static_cast<int>(RayType::Radiance),      // SBT offset
                       static_cast<int>(RayType::RayTypeCount),  // SBT stride
                       static_cast<int>(RayType::Radiance),      // missSBTIndex
                       u0, u1);
            energy +=
                material->material_properties.subsurface_factor * material->material_properties.subsurface_color *
                glm::clamp(glm::abs(glm::dot(outNormal, glm::vec3(newRayDirectionInternal.x, newRayDirectionInternal.y,
                                                                  newRayDirectionInternal.z))) *
                                   roughness +
                               (1.0f - roughness) * f,
                           0.0f, 1.0f) *
                perRayData.energy;
          }
        }
        float3 newRayDirectionInternal;
        BRDF(metallic, perRayData.random, rayDirection, hit_info.normal, newRayDirectionInternal);
        optixTrace(optixTraversableHandle, make_float3(hit_info.position.x, hit_info.position.y, hit_info.position.z),
                   newRayDirectionInternal,
                   1e-3f,  // tmin
                   1e20f,  // tmax
                   0.0f,   // rayTime
                   static_cast<OptixVisibilityMask>(255), OPTIX_RAY_FLAG_NONE,
                   static_cast<int>(RayType::Radiance),      // SBT offset
                   static_cast<int>(RayType::RayTypeCount),  // SBT stride
                   static_cast<int>(RayType::Radiance),      // missSBTIndex
                   u0, u1);
        energy += (1.0f - material->material_properties.subsurface_factor) * albedoColor *
                  glm::clamp(
                      glm::abs(glm::dot(hit_info.normal, glm::vec3(newRayDirectionInternal.x, newRayDirectionInternal.y,
                                                                   newRayDirectionInternal.z))) *
                              roughness +
                          (1.0f - roughness) * f,
                      0.0f, 1.0f) *
                  perRayData.energy;
      }
    }
    if (hitCount == 1) {
      perRayData.normal = hit_info.normal;
      perRayData.albedo = albedoColor;
      perRayData.position = hit_info.position;
    }
    perRayData.energy =
        energy + static_cast<SurfaceMaterial *>(sbtData.material)->material_properties.emission * albedoColor;

  } else {
    glm::vec3 btfColor;
    if (perRayData.hit_count <= rayTracerProperties.ray_properties.bounces) {
      energy = glm::vec3(0.0f);
      float f = 1.0f;
      glm::vec3 reflected = Reflect(rayDirection, hit_info.normal);
      if (environment.environmental_lighting_type == EnvironmentalLightingType::SingleLightSource) {
        glm::vec3 newRayDirection =
            RandomSampleHemisphere(perRayData.random, environment.sun_direction, 1.0f - environment.light_size);
        static_cast<SurfaceCompressedBtf *>(sbtData.material)
            ->GetValue(hit_info.tex_coord, rayDirection, newRayDirection, hit_info.normal, hit_info.tangent, btfColor);
        energy += glm::vec3(environment.color) * environment.ambient_light_intensity * btfColor;
        const float NdotL = glm::dot(hit_info.normal, newRayDirection);
        if (NdotL > 0.0f) {
          auto origin = hit_info.position;
          origin += hit_info.normal * 1e-3f;
          float3 incidentRayOrigin = make_float3(origin.x, origin.y, origin.z);
          float3 newRayDirectionInternal = make_float3(newRayDirection.x, newRayDirection.y, newRayDirection.z);
          optixTrace(
              optixTraversableHandle, incidentRayOrigin, newRayDirectionInternal,
              1e-3f,  // tmin
              1e20f,  // tmax
              0.0f,   // rayTime
              static_cast<OptixVisibilityMask>(255),
              OPTIX_RAY_FLAG_DISABLE_ANYHIT | OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT | OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT,
              static_cast<int>(RayType::Radiance),      // SBT offset
              static_cast<int>(RayType::RayTypeCount),  // SBT stride
              static_cast<int>(RayType::Radiance),      // missSBTIndex
              u0, u1);
          energy += perRayData.energy * NdotL * btfColor;
        }
      } else {
        if (enableSkydomeSun && environment.environmental_lighting_type == EnvironmentalLightingType::Skydome) {
          glm::vec3 sun_direction;
          float n_dot_l;
          if (TraceSkydomeSun(environment, perRayData.random, optixTraversableHandle, hit_info.position,
                              hit_info.normal, sun_direction, n_dot_l)) {
            static_cast<SurfaceCompressedBtf *>(sbtData.material)
                ->GetValue(hit_info.tex_coord, rayDirection, sun_direction, hit_info.normal, hit_info.tangent,
                           btfColor);
            energy += environment.sun_color * environment.sun_intensity * n_dot_l * btfColor;
          }
        }
        glm::vec3 newRayDirection = RandomSampleHemisphere(perRayData.random, reflected, 0.0f);
        static_cast<SurfaceCompressedBtf *>(sbtData.material)
            ->GetValue(hit_info.tex_coord, rayDirection, newRayDirection, hit_info.normal, hit_info.tangent, btfColor);
        auto origin = hit_info.position;
        origin += hit_info.normal * 1e-3f;
        float3 incidentRayOrigin = make_float3(origin.x, origin.y, origin.z);
        float3 newRayDirectionInternal = make_float3(newRayDirection.x, newRayDirection.y, newRayDirection.z);
        optixTrace(optixTraversableHandle, incidentRayOrigin, newRayDirectionInternal,
                   1e-3f,  // tmin
                   1e20f,  // tmax
                   0.0f,   // rayTime
                   static_cast<OptixVisibilityMask>(255),
                   OPTIX_RAY_FLAG_DISABLE_ANYHIT,            // OPTIX_RAY_FLAG_NONE,
                   static_cast<int>(RayType::Radiance),      // SBT offset
                   static_cast<int>(RayType::RayTypeCount),  // SBT stride
                   static_cast<int>(RayType::Radiance),      // missSBTIndex
                   u0, u1);
        energy += btfColor * perRayData.energy;
      }
    }
    if (hitCount == 1) {
      perRayData.normal = hit_info.normal;
      perRayData.albedo = btfColor;
      perRayData.position = hit_info.position;
    }
    perRayData.energy = energy;
  }
}

static __forceinline__ __device__ void MissFunc(const RayTracerProperties &rayTracerProperties) {
  PerRayData<glm::vec3> &perRayData = *GetRayDataPointer<PerRayData<glm::vec3>>();
  const float3 rayDir = optixGetWorldRayDirection();
  float3 rayOrigin = optixGetWorldRayOrigin();
  glm::vec3 rayOrig = glm::vec3(rayOrigin.x, rayOrigin.y, rayOrigin.z);
  glm::vec3 rayDirection = glm::vec3(rayDir.x, rayDir.y, rayDir.z);
  auto &environment = rayTracerProperties.environment;
  glm::vec3 environmentalLightColor = CalculateEnvironmentalLight(rayOrig, rayDirection, environment);
  perRayData.albedo = perRayData.energy = environmentalLightColor;
}
}  // namespace evo_engine
