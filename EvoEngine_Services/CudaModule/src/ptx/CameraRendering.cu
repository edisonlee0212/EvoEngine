#include "OptixCloud.cuh"
#include "RayFunctions.cuh"

namespace evo_engine {
static __forceinline__ __device__ glm::vec3 Unproject(const glm::mat4& inverse_vp, glm::vec2 screen, float z) {
  const glm::vec4 point = inverse_vp * glm::vec4(screen, z, 1.f);
  return glm::vec3(point) / point.w;
}

static __forceinline__ __device__ void RenderCloudTexel(const CameraRenderingLaunchParams& params, glm::ivec2 pixel) {
  const auto& frame = params.cloud_frame;
  const auto& camera = params.camera_properties;
  const auto& s = params.clouds;
  const auto uv = (glm::vec2(pixel) + 0.5f) / glm::vec2(frame.size);
  const glm::vec2 screen = 2.f * uv - 1.f;
  const auto origin = Unproject(camera.inverse_projection_view, screen, -1.f);
  const auto direction = glm::normalize(Unproject(camera.inverse_projection_view, screen, 1.f) - origin);
  const int index = pixel.x + pixel.y * frame.size.x;
  Random random;
  random.Init(index, frame.sequence);
  float distance;
  auto cloud = EvaluateCloud(params, CloudFloat3(origin), CloudFloat3(direction), s.max_march_distance,
                             random(), glm::vec4(0.f), false, false, glm::vec4(0.f), distance, true,
                             s.debug_visualization ? s.debug_mode : 0, 0.f);
  const auto world = origin + direction * distance;
  glm::vec2 previous_uv;
  if (frame.previous && distance > 0.f && ProjectCloudUv(frame.previous_view_projection, world, previous_uv)) {
    const auto history = FilterCloud(frame.previous, frame.size, previous_uv, cloud,
                                     glm::distance(world, frame.previous_origin));
    cloud = glm::mix(cloud, history, s.temporal_blend_factor);
  }
  frame.current[index] = {cloud, distance > 0.f ? glm::distance(world, camera.camera_position) : 0.f};
}
extern "C" __constant__ CameraRenderingLaunchParams cameraRenderingLaunchParams;
#pragma region Closest hit functions
extern "C" __global__ void __closesthit__CR_R() {
  ClosestHitFunc(cameraRenderingLaunchParams.ray_tracer_properties, cameraRenderingLaunchParams.traversable);
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
  MissFunc(cameraRenderingLaunchParams.ray_tracer_properties);
}
extern "C" __global__ void __miss__CR_SS() {
}
#pragma endregion
#pragma region Main ray generation
extern "C" __global__ void __raygen__CR() {
  if (cameraRenderingLaunchParams.cloud_frame.prepass) {
    RenderCloudTexel(cameraRenderingLaunchParams, glm::ivec2(optixGetLaunchIndex().x, optixGetLaunchIndex().y));
    return;
  }
  float ix = optixGetLaunchIndex().x;
  float iy = optixGetLaunchIndex().y;
  const uint32_t fbIndex = ix + iy * cameraRenderingLaunchParams.camera_properties.target_frame.size.x;

  // compute a test pattern based on pixel ID

  PerRayData<glm::vec3> camera_ray_data;
  camera_ray_data.hit_count = 0;
  camera_ray_data.diffuse_indirect_path = false;
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
    const auto& inverse_vp = cameraRenderingLaunchParams.camera_properties.inverse_projection_view;
    const auto rayStart = Unproject(inverse_vp, screen, -1.f);
    const auto rayEnd = Unproject(inverse_vp, screen, 1.f);
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
    const auto &params = cameraRenderingLaunchParams;
    const bool hit = camera_ray_data.hit_count > 0;
    glm::vec4 color(0.0f);
    if (hit || params.camera_properties.background_type == BackgroundType::Environment)
      color = glm::vec4(camera_ray_data.energy, 1.0f);
    else if (params.camera_properties.background_type == BackgroundType::Color)
      color = params.camera_properties.background_color;

    const auto &s = params.clouds;
    if (s.enabled && params.camera_properties.output_type == OutputType::Color) {
      const float distance = hit ? glm::distance(aperturePoint, camera_ray_data.position) : s.max_march_distance;
      const auto& frame = params.cloud_frame;
      glm::vec2 uv = screen * 0.5f + 0.5f;
      bool use_cache = frame.current != nullptr;
      // A lens sample may look into a different texel. Outside the cached view, march its actual ray.
      if (use_cache && params.camera_properties.aperture != 0.f) {
        use_cache = ProjectCloudUv(frame.view_projection, aperturePoint + rayDir * s.max_march_distance, uv);
        if (use_cache) {
          const auto p = glm::clamp(glm::ivec2(uv * glm::vec2(frame.size)), glm::ivec2(0), frame.size - 1);
          const float depth = frame.current[p.x + p.y * frame.size.x].depth;
          if (depth > 0.f)
            use_cache = ProjectCloudUv(frame.view_projection, aperturePoint + rayDir * depth, uv);
        }
      }
      const glm::vec4 cached = use_cache ? FilterCloud(frame.current, frame.size, uv, glm::vec4(0, 0, 0, 1))
                                         : glm::vec4(0.f);
      auto cloud_random = camera_ray_data.random;
      const float cloud_jitter = (sampleID + cloud_random()) / samples;
      float unused_depth;
      color = EvaluateCloud(params, rayOrigin, rayDirection, distance, cloud_jitter, color, hit,
                            use_cache, cached, unused_depth, false, s.debug_visualization ? s.debug_mode : 0,
                            s.enable_cloud_shadows ? s.cloud_shadow_strength : 0.f);
    }
    pixel_color += color;

    pixel_normal += glm::vec4(camera_ray_data.normal, 1.0f);
    pixel_albedo += glm::vec4(camera_ray_data.albedo, 1.0f);
    pixel_position += camera_ray_data.position;
    camera_ray_data.energy = glm::vec3(0.0f);
    camera_ray_data.normal = glm::vec3(0.0f);
    camera_ray_data.albedo = glm::vec3(0.0f);
    camera_ray_data.position = glm::vec3(0.0f);
    camera_ray_data.hit_count = 0;
    camera_ray_data.diffuse_indirect_path = false;
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
