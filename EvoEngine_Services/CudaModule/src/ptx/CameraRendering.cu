#include "RayFunctions.cuh"

namespace evo_engine {
extern "C" __constant__ CameraRenderingLaunchParams cameraRenderingLaunchParams;
#pragma region Closest hit functions
extern "C" __global__ void __closesthit__CR_R() {
  ClosestHitFunc(cameraRenderingLaunchParams.ray_tracer_properties, cameraRenderingLaunchParams.traversable, true);
}

extern "C" __global__ void __closesthit__CR_SS() {
  SSHit();
}
extern "C" __global__ void __closesthit__CR_S() {
  optixSetPayload_0(0);
}
#pragma endregion
#pragma region Any hit functions

extern "C" __global__ void __anyhit__CR_R() {
  AnyHitFunc();
}

extern "C" __global__ void __anyhit__CR_SS() {
  SSAnyHit();
}
extern "C" __global__ void __anyhit__CR_S() {
  ShadowAnyHitFunc();
}
#pragma endregion
#pragma region Miss functions
extern "C" __global__ void __miss__CR_R() {
  MissFunc(cameraRenderingLaunchParams.ray_tracer_properties);
}
extern "C" __global__ void __miss__CR_SS() {
}
extern "C" __global__ void __miss__CR_S() {
  ShadowMissFunc();
}
#pragma endregion
#pragma region Main ray generation
extern "C" __global__ void __raygen__CR() {
  float ix = optixGetLaunchIndex().x;
  float iy = optixGetLaunchIndex().y;
  const uint32_t fbIndex = ix + iy * cameraRenderingLaunchParams.camera_properties.target_frame.size.x;

  // compute a test pattern based on pixel ID

  PerRayData<glm::vec3> camera_ray_data;
  camera_ray_data.hit_count = 0;
  camera_ray_data.random.Init(ix + cameraRenderingLaunchParams.camera_properties.target_frame.size.x * iy,
                              cameraRenderingLaunchParams.camera_properties.target_frame.frame_id);
  camera_ray_data.energy = glm::vec3(0);
  camera_ray_data.normal = glm::vec3(0);
  camera_ray_data.albedo = glm::vec3(0);
  camera_ray_data.position = glm::vec3(999999.0f);
  // the values we store the PRD pointer in:
  uint32_t u0, u1;
  PackRayDataPointer(&camera_ray_data, u0, u1);

  const auto samples = cameraRenderingLaunchParams.ray_tracer_properties.ray_properties.samples;
  auto pixel_color = glm::vec4(0.f);
  auto pixel_normal = glm::vec4(0.f);
  auto pixel_albedo = glm::vec4(0.f);
  auto pixel_position = glm::vec3(0.0f);
  float halfX = cameraRenderingLaunchParams.camera_properties.target_frame.size.x * .5f;
  float halfY = cameraRenderingLaunchParams.camera_properties.target_frame.size.y * .5f;

  for (int sampleID = 0; sampleID < samples; sampleID++) {
    glm::vec2 screen =
        glm::vec2((ix + camera_ray_data.random() - halfX) / halfX, (iy + camera_ray_data.random() - halfY) / halfY);
    glm::vec4 start = cameraRenderingLaunchParams.camera_properties.inverse_projection_view *
                      glm::vec4(screen.x, screen.y, -1.0f, 1.0f);
    glm::vec4 end = cameraRenderingLaunchParams.camera_properties.inverse_projection_view *
                    glm::vec4(screen.x, screen.y, 1.0f, 1.0f);
    start /= start.w;
    end /= end.w;
    glm::vec3 rayStart = start;
    glm::vec3 rayEnd = end;
    glm::vec3 primaryRayDir = glm::normalize(rayEnd - rayStart);
    glm::vec3 convergence = rayStart + primaryRayDir * cameraRenderingLaunchParams.camera_properties.focal_length;
    float angle = camera_ray_data.random() * 3.1415927f * 2.0f;
    glm::vec3 aperturePoint =
        rayStart + cameraRenderingLaunchParams.camera_properties.aperture *
                       (cameraRenderingLaunchParams.camera_properties.horizontal_direction * glm::sin(angle) +
                        cameraRenderingLaunchParams.camera_properties.vertical_direction * glm::cos(angle));
    glm::vec3 rayDir = glm::normalize(convergence - aperturePoint);
    float3 rayOrigin = make_float3(aperturePoint.x, aperturePoint.y, aperturePoint.z);
    float3 rayDirection = make_float3(rayDir.x, rayDir.y, rayDir.z);

    optixTrace(cameraRenderingLaunchParams.traversable, rayOrigin, rayDirection,
               0.f,    // tmin
               1e20f,  // tmax
               0.0f,   // rayTime
               static_cast<OptixVisibilityMask>(255),
               OPTIX_RAY_FLAG_NONE,                      // OPTIX_RAY_FLAG_NONE,
               static_cast<int>(RayType::Radiance),      // SBT offset
               static_cast<int>(RayType::RayTypeCount),  // SBT stride
               static_cast<int>(RayType::Radiance),      // missSBTIndex
               u0, u1);
    if (camera_ray_data.hit_count > 0) {
      pixel_color += glm::vec4(camera_ray_data.energy, 1.0f);
    } else {
      switch (cameraRenderingLaunchParams.camera_properties.background_type) {
        case BackgroundType::Environment: {
          pixel_color += glm::vec4(camera_ray_data.energy, 1.0f);
          break;
        }
        case BackgroundType::Color: {
          pixel_color += cameraRenderingLaunchParams.camera_properties.background_color;
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

  // and write/accumulate to frame buffer ...
  if (cameraRenderingLaunchParams.camera_properties.accumulate) {
    if (cameraRenderingLaunchParams.camera_properties.target_frame.frame_id > 1) {
      glm::vec4 prev_gamma_corrected_color =
          cameraRenderingLaunchParams.camera_properties.target_frame.color_buffer[fbIndex];
      glm::vec4 prev_color = glm::vec4(glm::pow(glm::vec3(prev_gamma_corrected_color),
                                                glm::vec3(cameraRenderingLaunchParams.camera_properties.gamma)),
                                       prev_gamma_corrected_color.a);
      float count = static_cast<float>(cameraRenderingLaunchParams.camera_properties.target_frame.frame_id + 1);
      pixel_color +=
          static_cast<float>(cameraRenderingLaunchParams.camera_properties.target_frame.frame_id) * prev_color;
      pixel_color /= count;
    }
  }
  // and write to frame buffer ...
  glm::vec4 output_color =
      glm::vec4(glm::pow(glm::vec3(pixel_color), glm::vec3(1.0 / cameraRenderingLaunchParams.camera_properties.gamma)),
                pixel_color.a);
  glm::vec4 output_albedo = pixel_albedo;
  glm::vec4 output_normal = pixel_normal;
  if (cameraRenderingLaunchParams.camera_properties.output_type == OutputType::Depth) {
    float distance = glm::distance(cameraRenderingLaunchParams.camera_properties.camera_position, pixel_position);
    output_albedo = glm::vec4(
        glm::vec3(glm::clamp(distance / cameraRenderingLaunchParams.camera_properties.max_distance, 0.0f, 1.0f)), 1.0f);
  }

  cameraRenderingLaunchParams.camera_properties.target_frame.albedo_buffer[fbIndex] = output_albedo;
  cameraRenderingLaunchParams.camera_properties.target_frame.color_buffer[fbIndex] = output_color;
  cameraRenderingLaunchParams.camera_properties.target_frame.normal_buffer[fbIndex] = output_normal;
}
#pragma endregion
}  // namespace evo_engine
