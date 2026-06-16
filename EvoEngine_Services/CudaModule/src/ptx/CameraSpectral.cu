#include "BSDF.cuh"
#include "BSSDF.cuh"
#include "Environment.cuh"

namespace evo_engine {
extern "C" __constant__ CameraSpectralLaunchParams cameraSpectralLaunchParams;

static __forceinline__ __device__ void CameraSpectralAnyHitFunc() {
  const float3 ray_direction_internal = optixGetWorldRayDirection();
  glm::vec3 ray_direction =
      glm::vec3(ray_direction_internal.x, ray_direction_internal.y, ray_direction_internal.z);
  const auto &sbt_data = *(const SBT *)optixGetSbtDataPointer();
  const auto hit_info = sbt_data.GetHitInfo(ray_direction);
  switch (sbt_data.material_type) {
    case MaterialType::Default: {
      auto &per_ray_data = *GetRayDataPointer<PerRayData<glm::vec3>>();
      const glm::vec4 albedo_color = static_cast<SurfaceMaterial *>(sbt_data.material)->GetAlbedo(hit_info.tex_coord);
      if (albedo_color.w <= per_ray_data.random())
        optixIgnoreIntersection();
    } break;
  }
}

static __forceinline__ __device__ void CameraSpectralClosestHitFunc() {
  const float3 ray_direction_internal = optixGetWorldRayDirection();
  glm::vec3 ray_direction =
      glm::vec3(ray_direction_internal.x, ray_direction_internal.y, ray_direction_internal.z);
  const auto &sbt_data = *(const SBT *)optixGetSbtDataPointer();
  auto hit_info = sbt_data.GetHitInfo(ray_direction);
  auto &per_ray_data = *GetRayDataPointer<PerRayData<glm::vec3>>();
  const unsigned hit_count = per_ray_data.hit_count + 1;

  glm::vec3 energy = glm::vec3(0.0f);
  uint32_t u0, u1;
  PackRayDataPointer(&per_ray_data, u0, u1);
  per_ray_data.hit_count = hit_count;
  per_ray_data.energy = glm::vec3(0.0f);
  auto &environment = cameraSpectralLaunchParams.ray_tracer_properties.environment;
  if (sbt_data.material_type != MaterialType::CompressedBTF) {
    auto *material = static_cast<SurfaceMaterial *>(sbt_data.material);
    material->ApplyNormalTexture(hit_info.normal, hit_info.tex_coord, hit_info.tangent);
    const float metallic = material->GetMetallic(hit_info.tex_coord);
    const float roughness = material->GetRoughness(hit_info.tex_coord);
    glm::vec3 albedo_color;
    if (sbt_data.material_type == MaterialType::Default) {
      albedo_color = material->GetAlbedo(hit_info.tex_coord);
    } else {
      albedo_color = hit_info.color;
    }
    float f = 1.0f;
    if (metallic >= 0.0f)
      f = (metallic + 2.0f) / (metallic + 1.0f);
    if (environment.environmental_lighting_type == EnvironmentalLightingType::SingleLightSource) {
      const glm::vec3 new_ray_direction =
          RandomSampleHemisphere(per_ray_data.random, environment.sun_direction, 1.0f - environment.light_size);
      energy += glm::vec3(environment.color) * environment.ambient_light_intensity * albedo_color;
      const float n_dot_l = glm::dot(hit_info.normal, new_ray_direction);
      if (n_dot_l > 0.0f) {
        PackRayDataPointer(&per_ray_data, u0, u1);
        per_ray_data.energy = glm::vec3(0.0f);
        optixTrace(cameraSpectralLaunchParams.traversable,
                   make_float3(hit_info.position.x, hit_info.position.y, hit_info.position.z),
                   make_float3(new_ray_direction.x, new_ray_direction.y, new_ray_direction.z), 1e-3f, 1e20f, 0.0f,
                   static_cast<OptixVisibilityMask>(255),
                   OPTIX_RAY_FLAG_DISABLE_ANYHIT | OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT |
                       OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT,
                   static_cast<int>(RayType::Radiance), static_cast<int>(RayType::RayTypeCount),
                   static_cast<int>(RayType::Radiance), u0, u1);
        energy += per_ray_data.energy * n_dot_l * albedo_color;
      }
    } else if (per_ray_data.hit_count <= cameraSpectralLaunchParams.ray_tracer_properties.ray_properties.bounces) {
      bool need_sample = false;
      if (hit_count <= 1 && material->material_properties.subsurface_factor > 0.0f &&
          material->material_properties.subsurface_radius.x > 0.0f) {
        float3 incident_ray_origin;
        float3 new_ray_direction_internal;
        glm::vec3 out_normal;
        need_sample = BSSRDF(metallic, per_ray_data.random, material->material_properties.subsurface_radius.x,
                             sbt_data.handle, cameraSpectralLaunchParams.traversable, hit_info.position, ray_direction,
                             hit_info.normal, incident_ray_origin, new_ray_direction_internal, out_normal);
        if (need_sample) {
          optixTrace(cameraSpectralLaunchParams.traversable, incident_ray_origin, new_ray_direction_internal, 1e-3f,
                     1e20f, 0.0f, static_cast<OptixVisibilityMask>(255), OPTIX_RAY_FLAG_NONE,
                     static_cast<int>(RayType::Radiance), static_cast<int>(RayType::RayTypeCount),
                     static_cast<int>(RayType::Radiance), u0, u1);
          energy +=
              material->material_properties.subsurface_factor * material->material_properties.subsurface_color *
              glm::clamp(glm::abs(glm::dot(out_normal, glm::vec3(new_ray_direction_internal.x,
                                                                 new_ray_direction_internal.y,
                                                                 new_ray_direction_internal.z))) *
                                 roughness +
                             (1.0f - roughness) * f,
                         0.0f, 1.0f) *
              per_ray_data.energy;
        }
      }
      float3 new_ray_direction_internal;
      BRDF(metallic, per_ray_data.random, ray_direction, hit_info.normal, new_ray_direction_internal);
      optixTrace(cameraSpectralLaunchParams.traversable,
                 make_float3(hit_info.position.x, hit_info.position.y, hit_info.position.z),
                 new_ray_direction_internal, 1e-3f, 1e20f, 0.0f, static_cast<OptixVisibilityMask>(255),
                 OPTIX_RAY_FLAG_NONE, static_cast<int>(RayType::Radiance), static_cast<int>(RayType::RayTypeCount),
                 static_cast<int>(RayType::Radiance), u0, u1);
      energy +=
          (1.0f - material->material_properties.subsurface_factor) * albedo_color *
          glm::clamp(glm::abs(glm::dot(hit_info.normal, glm::vec3(new_ray_direction_internal.x,
                                                                  new_ray_direction_internal.y,
                                                                  new_ray_direction_internal.z))) *
                             roughness +
                         (1.0f - roughness) * f,
                     0.0f, 1.0f) *
          per_ray_data.energy;
    }
    if (hit_count == 1) {
      per_ray_data.normal = hit_info.normal;
      per_ray_data.albedo = albedo_color;
      per_ray_data.position = hit_info.position;
    }
    per_ray_data.energy = energy + material->material_properties.emission * albedo_color;
  } else {
    glm::vec3 btf_color;
    if (per_ray_data.hit_count <= cameraSpectralLaunchParams.ray_tracer_properties.ray_properties.bounces) {
      const glm::vec3 reflected = Reflect(ray_direction, hit_info.normal);
      if (environment.environmental_lighting_type == EnvironmentalLightingType::SingleLightSource) {
        const glm::vec3 new_ray_direction =
            RandomSampleHemisphere(per_ray_data.random, environment.sun_direction, 1.0f - environment.light_size);
        static_cast<SurfaceCompressedBtf *>(sbt_data.material)
            ->GetValue(hit_info.tex_coord, ray_direction, new_ray_direction, hit_info.normal, hit_info.tangent,
                       btf_color);
        energy += glm::vec3(environment.color) * environment.ambient_light_intensity * btf_color;
        const float n_dot_l = glm::dot(hit_info.normal, new_ray_direction);
        if (n_dot_l > 0.0f) {
          auto origin = hit_info.position + hit_info.normal * 1e-3f;
          optixTrace(cameraSpectralLaunchParams.traversable, make_float3(origin.x, origin.y, origin.z),
                     make_float3(new_ray_direction.x, new_ray_direction.y, new_ray_direction.z), 1e-3f, 1e20f, 0.0f,
                     static_cast<OptixVisibilityMask>(255),
                     OPTIX_RAY_FLAG_DISABLE_ANYHIT | OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT |
                         OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT,
                     static_cast<int>(RayType::Radiance), static_cast<int>(RayType::RayTypeCount),
                     static_cast<int>(RayType::Radiance), u0, u1);
          energy += per_ray_data.energy * n_dot_l * btf_color;
        }
      } else {
        const glm::vec3 new_ray_direction = RandomSampleHemisphere(per_ray_data.random, reflected, 0.0f);
        static_cast<SurfaceCompressedBtf *>(sbt_data.material)
            ->GetValue(hit_info.tex_coord, ray_direction, new_ray_direction, hit_info.normal, hit_info.tangent,
                       btf_color);
        const auto origin = hit_info.position + hit_info.normal * 1e-3f;
        optixTrace(cameraSpectralLaunchParams.traversable, make_float3(origin.x, origin.y, origin.z),
                   make_float3(new_ray_direction.x, new_ray_direction.y, new_ray_direction.z), 1e-3f, 1e20f, 0.0f,
                   static_cast<OptixVisibilityMask>(255), OPTIX_RAY_FLAG_DISABLE_ANYHIT,
                   static_cast<int>(RayType::Radiance), static_cast<int>(RayType::RayTypeCount),
                   static_cast<int>(RayType::Radiance), u0, u1);
        energy += btf_color * per_ray_data.energy;
      }
    }
    if (hit_count == 1) {
      per_ray_data.normal = hit_info.normal;
      per_ray_data.albedo = btf_color;
      per_ray_data.position = hit_info.position;
    }
    per_ray_data.energy = energy;
  }
}

static __forceinline__ __device__ void CameraSpectralMissFunc() {
  auto &per_ray_data = *GetRayDataPointer<PerRayData<glm::vec3>>();
  const float3 ray_dir = optixGetWorldRayDirection();
  const float3 ray_origin = optixGetWorldRayOrigin();
  const glm::vec3 ray_orig = glm::vec3(ray_origin.x, ray_origin.y, ray_origin.z);
  glm::vec3 ray_direction = glm::vec3(ray_dir.x, ray_dir.y, ray_dir.z);
  auto &environment = cameraSpectralLaunchParams.ray_tracer_properties.environment;
  const glm::vec3 environmental_light_color = CalculateEnvironmentalLight(ray_orig, ray_direction, environment);
  per_ray_data.albedo = per_ray_data.energy = environmental_light_color;
}

#pragma region Closest hit functions
extern "C" __global__ void __closesthit__CS_R() {
  CameraSpectralClosestHitFunc();
}

extern "C" __global__ void __closesthit__CS_SS() {
  SSHit();
}
#pragma endregion
#pragma region Any hit functions

extern "C" __global__ void __anyhit__CS_R() {
  CameraSpectralAnyHitFunc();
}

extern "C" __global__ void __anyhit__CS_SS() {
  SSAnyHit();
}
#pragma endregion
#pragma region Miss functions
extern "C" __global__ void __miss__CS_R() {
  CameraSpectralMissFunc();
}
extern "C" __global__ void __miss__CS_SS() {
}
#pragma endregion
#pragma region Main ray generation
extern "C" __global__ void __raygen__CS() {
  float ix = optixGetLaunchIndex().x;
  float iy = optixGetLaunchIndex().y;
  const uint32_t fbIndex = ix + iy * cameraSpectralLaunchParams.camera_properties.target_frame.size.x;

  PerRayData<glm::vec3> camera_ray_data;
  camera_ray_data.hit_count = 0;
  camera_ray_data.random.Init(ix + cameraSpectralLaunchParams.camera_properties.target_frame.size.x * iy,
                              cameraSpectralLaunchParams.camera_properties.target_frame.frame_id);
  camera_ray_data.energy = glm::vec3(0);
  camera_ray_data.normal = glm::vec3(0);
  camera_ray_data.albedo = glm::vec3(0);
  camera_ray_data.position = glm::vec3(999999.0f);
  uint32_t u0, u1;
  PackRayDataPointer(&camera_ray_data, u0, u1);

  const auto samples = cameraSpectralLaunchParams.ray_tracer_properties.ray_properties.samples;
  auto pixel_color = glm::vec4(0.f);
  auto pixel_normal = glm::vec4(0.f);
  auto pixel_albedo = glm::vec4(0.f);
  auto pixel_position = glm::vec3(0.0f);
  float halfX = cameraSpectralLaunchParams.camera_properties.target_frame.size.x * .5f;
  float halfY = cameraSpectralLaunchParams.camera_properties.target_frame.size.y * .5f;

  for (int sampleID = 0; sampleID < samples; sampleID++) {
    glm::vec2 screen =
        glm::vec2((ix + camera_ray_data.random() - halfX) / halfX, (iy + camera_ray_data.random() - halfY) / halfY);
    glm::vec4 start =
        cameraSpectralLaunchParams.camera_properties.inverse_projection_view * glm::vec4(screen.x, screen.y, -1.0f, 1.0f);
    glm::vec4 end =
        cameraSpectralLaunchParams.camera_properties.inverse_projection_view * glm::vec4(screen.x, screen.y, 1.0f, 1.0f);
    start /= start.w;
    end /= end.w;
    glm::vec3 rayStart = start;
    glm::vec3 rayEnd = end;
    glm::vec3 primaryRayDir = glm::normalize(rayEnd - rayStart);
    glm::vec3 convergence = rayStart + primaryRayDir * cameraSpectralLaunchParams.camera_properties.focal_length;
    float angle = camera_ray_data.random() * 3.1415927f * 2.0f;
    glm::vec3 aperturePoint =
        rayStart + cameraSpectralLaunchParams.camera_properties.aperture *
                       (cameraSpectralLaunchParams.camera_properties.horizontal_direction * glm::sin(angle) +
                        cameraSpectralLaunchParams.camera_properties.vertical_direction * glm::cos(angle));
    glm::vec3 rayDir = glm::normalize(convergence - aperturePoint);
    float3 rayOrigin = make_float3(aperturePoint.x, aperturePoint.y, aperturePoint.z);
    float3 rayDirection = make_float3(rayDir.x, rayDir.y, rayDir.z);

    optixTrace(cameraSpectralLaunchParams.traversable, rayOrigin, rayDirection,
               0.f,    // tmin
               1e20f,  // tmax
               0.0f,   // rayTime
               static_cast<OptixVisibilityMask>(255),
               OPTIX_RAY_FLAG_NONE,
               static_cast<int>(RayType::Radiance),
               static_cast<int>(RayType::RayTypeCount),
               static_cast<int>(RayType::Radiance),
               u0, u1);
    if (camera_ray_data.hit_count > 0) {
      pixel_color += glm::vec4(camera_ray_data.energy, 1.0f);
    } else {
      switch (cameraSpectralLaunchParams.camera_properties.background_type) {
        case BackgroundType::Environment: {
          pixel_color += glm::vec4(camera_ray_data.energy, 1.0f);
          break;
        }
        case BackgroundType::Color: {
          pixel_color += cameraSpectralLaunchParams.camera_properties.background_color;
        }
        default:
          break;
      }
    }

    pixel_normal += glm::vec4(camera_ray_data.normal, 1.0f);
    pixel_albedo += glm::vec4(camera_ray_data.albedo, 1.0f);
    pixel_position += camera_ray_data.position;
    camera_ray_data.energy = glm::vec3(0.0f);
    camera_ray_data.normal = glm::vec3(0.0f);
    camera_ray_data.albedo = glm::vec3(0.0f);
    camera_ray_data.position = glm::vec3(0.0f);
    camera_ray_data.hit_count = 0;
  }
  pixel_color /= samples;
  pixel_normal /= samples;
  pixel_albedo /= samples;
  pixel_position /= samples;

  if (cameraSpectralLaunchParams.camera_properties.accumulate) {
    if (cameraSpectralLaunchParams.camera_properties.target_frame.frame_id > 1) {
      glm::vec4 prev_gamma_corrected_color =
          cameraSpectralLaunchParams.camera_properties.target_frame.color_buffer[fbIndex];
      glm::vec4 prev_color = glm::vec4(glm::pow(glm::vec3(prev_gamma_corrected_color),
                                                glm::vec3(cameraSpectralLaunchParams.camera_properties.gamma)),
                                       prev_gamma_corrected_color.a);
      float count = static_cast<float>(cameraSpectralLaunchParams.camera_properties.target_frame.frame_id + 1);
      pixel_color += static_cast<float>(cameraSpectralLaunchParams.camera_properties.target_frame.frame_id) * prev_color;
      pixel_color /= count;
    }
  }
  glm::vec4 output_color =
      glm::vec4(glm::pow(glm::vec3(pixel_color), glm::vec3(1.0 / cameraSpectralLaunchParams.camera_properties.gamma)),
                pixel_color.a);
  glm::vec4 output_albedo = pixel_albedo;
  glm::vec4 output_normal = pixel_normal;
  if (cameraSpectralLaunchParams.camera_properties.output_type == OutputType::Depth) {
    float distance = glm::distance(cameraSpectralLaunchParams.camera_properties.camera_position, pixel_position);
    output_albedo = glm::vec4(
        glm::vec3(glm::clamp(distance / cameraSpectralLaunchParams.camera_properties.max_distance, 0.0f, 1.0f)), 1.0f);
  }

  cameraSpectralLaunchParams.camera_properties.target_frame.albedo_buffer[fbIndex] = output_albedo;
  cameraSpectralLaunchParams.camera_properties.target_frame.color_buffer[fbIndex] = output_color;
  cameraSpectralLaunchParams.camera_properties.target_frame.normal_buffer[fbIndex] = output_normal;
}
#pragma endregion
}  // namespace evo_engine
