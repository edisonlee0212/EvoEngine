// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.VolumetricClouds;

[[vk::binding(0, 1)]] Sampler2D inDepth;
[[vk::binding(2, 1)]] [vk::image_format("rgba16f")] RWTexture2D<float4> outCloudAccumulation;
[[vk::binding(3, 1)]] [vk::image_format("r16f")] RWTexture2D<float> outCloudTransmittance;

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

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  int2 pixel = int2(dispatch_thread_id.xy);
  uint width = 0;
  uint height = 0;
  outCloudAccumulation.GetDimensions(width, height);
  int2 resolution = int2(width, height);
  if (pixel.x >= resolution.x || pixel.y >= resolution.y) {
    return;
  }

  float2 tex_coord = (float2(pixel) + float2(0.5f)) / float2(resolution);
  int camera_index = EE_VOLUMETRIC_CLOUD_CONSTANTS.cameraFrameSteps.x;
  float3 ray_origin = EE_CAMERA_POSITION(camera_index);
  float3 ray_direction = EE_VOLUMETRIC_CLOUD_RAY_DIRECTION(camera_index, tex_coord);
  float max_distance = max(EE_VOLUMETRIC_CLOUD_CONSTANTS.noiseExtinctionMarchDistance.w, 0.0f);
  float scene_distance =
      EE_VOLUMETRIC_CLOUD_SCENE_DISTANCE(camera_index, tex_coord, ray_origin, ray_direction, max_distance);

  float3 sun_direction;
  float3 sun_radiance;
  float3 ambient_radiance;
  EE_VOLUMETRIC_CLOUD_SUN_LIGHT(sun_direction, sun_radiance, ambient_radiance);

  float2 jitter = EE_VOLUMETRIC_CLOUD_TemporalJitter(pixel, uint(EE_VOLUMETRIC_CLOUD_CONSTANTS.cameraFrameSteps.y));
  VolumetricCloudSettingsGpu cloud_settings = EE_BUILD_VOLUMETRIC_CLOUD_SETTINGS();
  VolumetricCloudMarchResult cloud = EE_VOLUMETRIC_CLOUD_March(
      cloud_settings, ray_origin, ray_direction, max_distance, scene_distance, sun_direction, sun_radiance,
      ambient_radiance, EE_VOLUMETRIC_CLOUD_CONSTANTS.windTime.w, jitter.x);

  outCloudAccumulation[pixel] =
      float4(EE_VOLUMETRIC_CLOUD_DebugAccumulation(cloud, cloud_settings.debug_mode), cloud.march_distance);
  outCloudTransmittance[pixel] = cloud.transmittance;
}
