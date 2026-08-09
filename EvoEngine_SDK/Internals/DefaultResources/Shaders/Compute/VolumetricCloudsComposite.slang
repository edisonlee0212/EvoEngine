// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.VolumetricClouds;

[[vk::binding(0, 1)]] Sampler2D inDepth;
[[vk::binding(1, 1)]] [vk::image_format("rgba32f")] RWTexture2D<float4> inOutColor;
[[vk::binding(4, 1)]] Sampler2D inCloudAccumulation;
[[vk::binding(5, 1)]] Sampler2D inCloudTransmittance;
[[vk::binding(10, 1)]] Sampler2D inPreviousCloudAccumulation;
[[vk::binding(11, 1)]] Sampler2D inPreviousCloudTransmittance;
[[vk::binding(12, 1)]] [vk::image_format("rgba16f")] RWTexture2D<float4> outCloudAccumulationHistory;
[[vk::binding(13, 1)]] [vk::image_format("r16f")] RWTexture2D<float> outCloudTransmittanceHistory;

VolumetricCloudSettingsGpu EE_BUILD_VOLUMETRIC_CLOUD_SETTINGS() {
  VolumetricCloudSettingsGpu settings;
  settings.enabled = EE_VOLUMETRIC_CLOUD_CONSTANTS.flags.x;
  settings.coverage = EE_VOLUMETRIC_CLOUD_CONSTANTS.altitudeCoverageDensity.z;
  settings.density = EE_VOLUMETRIC_CLOUD_CONSTANTS.altitudeCoverageDensity.w;
  settings.bottom_altitude = EE_VOLUMETRIC_CLOUD_CONSTANTS.altitudeCoverageDensity.x;
  settings.top_altitude = EE_VOLUMETRIC_CLOUD_CONSTANTS.altitudeCoverageDensity.y;
  settings.wind_direction = EE_VOLUMETRIC_CLOUD_CONSTANTS.windTime.xy;
  settings.wind_speed = EE_VOLUMETRIC_CLOUD_CONSTANTS.windTime.z;
  settings.primary_step_count = EE_VOLUMETRIC_CLOUD_CONSTANTS.cameraFrameSteps.z;
  settings.light_step_count = EE_VOLUMETRIC_CLOUD_CONSTANTS.cameraFrameSteps.w;
  settings.lighting_intensity = EE_VOLUMETRIC_CLOUD_CONSTANTS.lightingPhaseMaxDistance.x;
  settings.ambient_lighting_strength = EE_VOLUMETRIC_CLOUD_CONSTANTS.lightingPhaseMaxDistance.y;
  settings.phase_anisotropy = EE_VOLUMETRIC_CLOUD_CONSTANTS.lightingPhaseMaxDistance.z;
  settings.base_noise_scale = EE_VOLUMETRIC_CLOUD_CONSTANTS.noiseExtinctionMarchDistance.x;
  settings.detail_noise_scale = EE_VOLUMETRIC_CLOUD_CONSTANTS.noiseExtinctionMarchDistance.y;
  settings.extinction_scale = EE_VOLUMETRIC_CLOUD_CONSTANTS.noiseExtinctionMarchDistance.z;
  settings.max_march_distance = EE_VOLUMETRIC_CLOUD_CONSTANTS.noiseExtinctionMarchDistance.w;
  settings.use_spherical_atmosphere = EE_VOLUMETRIC_CLOUD_CONSTANTS.atmosphereCloudTypeCurl.x > 0.5f ? 1 : 0;
  settings.atmosphere_radius = EE_VOLUMETRIC_CLOUD_CONSTANTS.atmosphereCloudTypeCurl.y;
  settings.cloud_type = EE_VOLUMETRIC_CLOUD_CONSTANTS.atmosphereCloudTypeCurl.z;
  settings.curl_strength = EE_VOLUMETRIC_CLOUD_CONSTANTS.atmosphereCloudTypeCurl.w;
  settings.coarse_step_fraction = EE_VOLUMETRIC_CLOUD_CONSTANTS.marchControl.x;
  settings.fine_step_scale = EE_VOLUMETRIC_CLOUD_CONSTANTS.marchControl.y;
  settings.empty_step_fallback_count = int(round(EE_VOLUMETRIC_CLOUD_CONSTANTS.marchControl.z));
  settings.enable_cloud_shadows = (EE_VOLUMETRIC_CLOUD_CONSTANTS.flags.w & 2) != 0 ? 1 : 0;
  settings.cloud_shadow_strength = EE_VOLUMETRIC_CLOUD_CONSTANTS.lightingPhaseMaxDistance.w;
  settings.cloud_shadow_step_count = max(EE_VOLUMETRIC_CLOUD_CONSTANTS.flags.w >> 2, 1);
  settings.debug_mode = EE_VOLUMETRIC_CLOUD_CONSTANTS.flags.y != 0 ? EE_VOLUMETRIC_CLOUD_CONSTANTS.flags.z : 0;
  return settings;
}

float3 EE_VOLUMETRIC_CLOUD_RAY_DIRECTION(in int camera_index, in float2 tex_coord) {
  float3 camera_position = EE_CAMERA_POSITION(camera_index);
  float3 far_world = EE_DEPTH_TO_WORLD_POS(camera_index, tex_coord, 1.0f);
  return normalize(far_world - camera_position);
}

float EE_VOLUMETRIC_CLOUD_SCENE_DISTANCE(in int camera_index, in float2 tex_coord, in float3 ray_origin,
                                         in float3 ray_direction, in float max_distance) {
  if ((EE_VOLUMETRIC_CLOUD_CONSTANTS.flags.w & 1) != 0) {
    float hit_distance = inDepth.SampleLevel(tex_coord, 0.0f).r;
    float camera_far = EE_CAMERA_FAR(camera_index);
    if (hit_distance >= camera_far * 0.999f) {
      return max_distance;
    }
    return clamp(hit_distance, 0.0f, max_distance);
  }
  float ndc_depth = inDepth.SampleLevel(tex_coord, 0.0f).r;
  if (ndc_depth >= 0.999999f) {
    return max_distance;
  }
  float3 world_position = EE_DEPTH_TO_WORLD_POS(camera_index, tex_coord, ndc_depth);
  return clamp(dot(world_position - ray_origin, ray_direction), 0.0f, max_distance);
}

int2 EE_VOLUMETRIC_CLOUD_DEPTH_AWARE_TEXEL(in int camera_index, in float2 tex_coord, in float3 ray_origin,
                                            in float3 ray_direction, in float max_distance) {
  uint cloud_width = 0;
  uint cloud_height = 0;
  inCloudAccumulation.GetDimensions(cloud_width, cloud_height);
  int2 cloud_resolution = int2(cloud_width, cloud_height);
  float2 cloud_pixel = tex_coord * float2(cloud_resolution) - float2(0.5f);
  int2 base_pixel = int2(floor(cloud_pixel));
  float target_distance =
      EE_VOLUMETRIC_CLOUD_SCENE_DISTANCE(camera_index, tex_coord, ray_origin, ray_direction, max_distance);

  int2 best_pixel = clamp(base_pixel, int2(0), cloud_resolution - int2(1));
  float best_error = 3.402823466e+38f;
  for (int y = 0; y <= 1; ++y) {
    for (int x = 0; x <= 1; ++x) {
      int2 candidate = clamp(base_pixel + int2(x, y), int2(0), cloud_resolution - int2(1));
      float2 candidate_tex_coord = (float2(candidate) + float2(0.5f)) / float2(cloud_resolution);
      float3 candidate_ray_direction = EE_VOLUMETRIC_CLOUD_RAY_DIRECTION(camera_index, candidate_tex_coord);
      float candidate_distance = EE_VOLUMETRIC_CLOUD_SCENE_DISTANCE(camera_index, candidate_tex_coord, ray_origin,
                                                                    candidate_ray_direction, max_distance);
      float candidate_error = abs(candidate_distance - target_distance);
      if (candidate_error < best_error) {
        best_error = candidate_error;
        best_pixel = candidate;
      }
    }
  }
  return best_pixel;
}

void EE_VOLUMETRIC_CLOUD_SUN_LIGHT(out float3 sun_direction, out float3 sun_radiance, out float3 ambient_radiance) {
  ambient_radiance = max(EE_ENVIRONMENT.background_color.rgb, float3(0.0f)) *
                     max(EE_ENVIRONMENT.diffuse_sky_intensity, 0.0f) *
                     max(EE_RENDER_INFO.indirect_lighting_intensity, 0.0f);
  sun_direction = normalize(float3(0.35f, 1.0f, 0.2f));
  sun_radiance = float3(1.0f);
  if (EE_RENDER_INFO.directional_light_size > 0) {
    DirectionalLight light = EE_DIRECTIONAL_LIGHTS[0];
    sun_direction = normalize(-light.direction);
    sun_radiance = max(light.diffuse.rgb, float3(0.0f));
  }
}

float2 EE_VOLUMETRIC_CLOUD_PREVIOUS_TEX_COORD(in int camera_index, in float3 world_position, out int valid) {
  float4 previous_clip = mul(float4(world_position, 1.0f), EE_CAMERAS[camera_index].previous_projection_view);
  valid = 0;
  if (previous_clip.w <= 0.0001f) {
    return float2(0.0f);
  }
  float3 previous_ndc = previous_clip.xyz / previous_clip.w;
  float2 previous_tex_coord = previous_ndc.xy * 0.5f + float2(0.5f);
  if (previous_tex_coord.x >= 0.0f && previous_tex_coord.x <= 1.0f && previous_tex_coord.y >= 0.0f &&
      previous_tex_coord.y <= 1.0f && previous_ndc.z >= -0.001f && previous_ndc.z <= 1.001f) {
    valid = 1;
  }
  return previous_tex_coord;
}

void EE_VOLUMETRIC_CLOUD_APPLY_TEMPORAL_HISTORY(in int camera_index, in float3 ray_origin, in float3 ray_direction,
                                                in float max_distance, inout float4 cloud_accumulation,
                                                inout float cloud_transmittance) {
  float history_weight = clamp(EE_VOLUMETRIC_CLOUD_CONSTANTS.marchControl.w, 0.0f, 0.98f);
  if (history_weight <= 0.0f || cloud_accumulation.a <= 0.0f || cloud_transmittance >= 0.999f) {
    return;
  }

  float3 reprojection_position = ray_origin + ray_direction * clamp(cloud_accumulation.a, 0.0f, max_distance);
  int valid;
  float2 previous_tex_coord = EE_VOLUMETRIC_CLOUD_PREVIOUS_TEX_COORD(camera_index, reprojection_position, valid);
  if (valid == 0) {
    return;
  }

  float4 previous_accumulation = inPreviousCloudAccumulation.SampleLevel(previous_tex_coord, 0.0f);
  float previous_transmittance = inPreviousCloudTransmittance.SampleLevel(previous_tex_coord, 0.0f).r;
  float distance_error = abs(previous_accumulation.a - cloud_accumulation.a);
  float max_distance_error = max(25.0f, cloud_accumulation.a * 0.25f);
  if (previous_accumulation.a <= 0.0f || distance_error > max_distance_error ||
      abs(previous_transmittance - cloud_transmittance) > 0.35f) {
    return;
  }

  cloud_accumulation.rgb = lerp(cloud_accumulation.rgb, previous_accumulation.rgb, history_weight);
  cloud_transmittance = lerp(cloud_transmittance, previous_transmittance, history_weight);
  cloud_accumulation.a = lerp(cloud_accumulation.a, previous_accumulation.a, min(history_weight, 0.5f));
}

float4 EE_VOLUMETRIC_CLOUD_COMPOSITE_DEBUG(in float3 scene_color, in float3 cloud_radiance, in float transmittance,
                                         in float march_distance, in float max_distance) {
  int debug_mode = EE_VOLUMETRIC_CLOUD_CONSTANTS.flags.y != 0 ? EE_VOLUMETRIC_CLOUD_CONSTANTS.flags.z : 0;
  if (debug_mode == 1) {
    return float4(cloud_radiance, 1.0f);
  }
  if (debug_mode == 2) {
    return float4(float3(transmittance), 1.0f);
  }
  if (debug_mode == 3) {
    return float4(float3(EE_VOLUMETRIC_CLOUD_Saturate(march_distance / max(max_distance, 1.0f))), 1.0f);
  }
  if (debug_mode == 4 || debug_mode == 5 || debug_mode == 6) {
    return float4(cloud_radiance, 1.0f);
  }
  return float4(cloud_radiance + scene_color * transmittance, 1.0f);
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  int2 pixel = int2(dispatch_thread_id.xy);
  uint width = 0;
  uint height = 0;
  inOutColor.GetDimensions(width, height);
  int2 resolution = int2(width, height);
  if (pixel.x >= resolution.x || pixel.y >= resolution.y) {
    return;
  }

  float2 tex_coord = (float2(pixel) + float2(0.5f)) / float2(resolution);
  int camera_index = EE_VOLUMETRIC_CLOUD_CONSTANTS.cameraFrameSteps.x;
  float3 ray_origin = EE_CAMERA_POSITION(camera_index);
  float3 ray_direction = EE_VOLUMETRIC_CLOUD_RAY_DIRECTION(camera_index, tex_coord);
  float max_distance = max(EE_VOLUMETRIC_CLOUD_CONSTANTS.noiseExtinctionMarchDistance.w, 0.0f);
  int2 cloud_pixel =
      EE_VOLUMETRIC_CLOUD_DEPTH_AWARE_TEXEL(camera_index, tex_coord, ray_origin, ray_direction, max_distance);

  float4 scene_color = inOutColor.Load(pixel);
  float4 cloud_accumulation = inCloudAccumulation.Load(int3(cloud_pixel, 0));
  float cloud_transmittance = inCloudTransmittance.Load(int3(cloud_pixel, 0)).r;
  VolumetricCloudSettingsGpu cloud_settings = EE_BUILD_VOLUMETRIC_CLOUD_SETTINGS();
  float scene_distance =
      EE_VOLUMETRIC_CLOUD_SCENE_DISTANCE(camera_index, tex_coord, ray_origin, ray_direction, max_distance);
  float3 shadowed_scene_color = scene_color.rgb;
  if (scene_distance < max_distance * 0.999f && scene_distance > 0.0f) {
    float3 sun_direction;
    float3 sun_radiance;
    float3 ambient_radiance;
    EE_VOLUMETRIC_CLOUD_SUN_LIGHT(sun_direction, sun_radiance, ambient_radiance);
    float3 surface_position = ray_origin + ray_direction * scene_distance;
    float cloud_shadow = EE_VOLUMETRIC_CLOUD_SurfaceShadowTransmittance(
        cloud_settings, surface_position, ray_origin, sun_direction, EE_VOLUMETRIC_CLOUD_CONSTANTS.windTime.w);
    shadowed_scene_color *= lerp(1.0f, cloud_shadow, clamp(cloud_settings.cloud_shadow_strength, 0.0f, 1.0f));
  }
  EE_VOLUMETRIC_CLOUD_APPLY_TEMPORAL_HISTORY(camera_index, ray_origin, ray_direction, max_distance,
                                             cloud_accumulation, cloud_transmittance);
  outCloudAccumulationHistory[pixel] = cloud_accumulation;
  outCloudTransmittanceHistory[pixel] = cloud_transmittance;
  float4 output_color = EE_VOLUMETRIC_CLOUD_COMPOSITE_DEBUG(shadowed_scene_color, cloud_accumulation.rgb,
                                                          cloud_transmittance, cloud_accumulation.a, max_distance);
  output_color.a = scene_color.a;
  inOutColor[pixel] = output_color;
}
