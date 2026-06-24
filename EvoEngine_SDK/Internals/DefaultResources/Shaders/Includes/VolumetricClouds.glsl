#ifndef _VOLUMETRIC_CLOUDS_GLSL_
#define _VOLUMETRIC_CLOUDS_GLSL_

const float EE_VOLUMETRIC_CLOUD_PI = 3.14159265359f;
const float EE_VOLUMETRIC_CLOUD_MIN_TRANSMITTANCE = 0.003f;

struct VolumetricCloudSettingsGpu {
  int enabled;
  float coverage;
  float density;
  float bottom_altitude;
  float top_altitude;
  vec2 wind_direction;
  float wind_speed;
  int primary_step_count;
  int light_step_count;
  float lighting_intensity;
  float ambient_lighting_strength;
  float phase_anisotropy;
  float base_noise_scale;
  float detail_noise_scale;
  float extinction_scale;
  int debug_mode;
};

struct VolumetricCloudRayInterval {
  float t_min;
  float t_max;
  int valid;
};

struct VolumetricCloudMarchResult {
  vec3 radiance;
  float transmittance;
  float mean_density;
  float march_distance;
  float mean_base_shape;
  float mean_detail_erosion;
  float mean_weather_coverage;
};

struct VolumetricCloudDensitySample {
  float final_density;
  float base_shape;
  float detail_erosion;
  float weather_coverage;
};

layout(set = 1, binding = 6) uniform sampler3D EE_VOLUMETRIC_CLOUD_BASE_SHAPE_NOISE;
layout(set = 1, binding = 7) uniform sampler3D EE_VOLUMETRIC_CLOUD_DETAIL_EROSION_NOISE;
layout(set = 1, binding = 8) uniform sampler2D EE_VOLUMETRIC_CLOUD_WEATHER_COVERAGE;

float EE_VOLUMETRIC_CLOUD_Saturate(in float value) {
  return clamp(value, 0.0f, 1.0f);
}

float EE_VOLUMETRIC_CLOUD_Hash31(in vec3 value) {
  return fract(sin(dot(value, vec3(127.1f, 311.7f, 74.7f))) * 43758.5453123f);
}

float EE_VOLUMETRIC_CLOUD_ValueNoise(in vec3 position) {
  vec3 cell = floor(position);
  vec3 local_position = fract(position);
  vec3 fade = local_position * local_position * (3.0f - 2.0f * local_position);

  float n000 = EE_VOLUMETRIC_CLOUD_Hash31(cell + vec3(0.0f, 0.0f, 0.0f));
  float n100 = EE_VOLUMETRIC_CLOUD_Hash31(cell + vec3(1.0f, 0.0f, 0.0f));
  float n010 = EE_VOLUMETRIC_CLOUD_Hash31(cell + vec3(0.0f, 1.0f, 0.0f));
  float n110 = EE_VOLUMETRIC_CLOUD_Hash31(cell + vec3(1.0f, 1.0f, 0.0f));
  float n001 = EE_VOLUMETRIC_CLOUD_Hash31(cell + vec3(0.0f, 0.0f, 1.0f));
  float n101 = EE_VOLUMETRIC_CLOUD_Hash31(cell + vec3(1.0f, 0.0f, 1.0f));
  float n011 = EE_VOLUMETRIC_CLOUD_Hash31(cell + vec3(0.0f, 1.0f, 1.0f));
  float n111 = EE_VOLUMETRIC_CLOUD_Hash31(cell + vec3(1.0f, 1.0f, 1.0f));

  float nx00 = mix(n000, n100, fade.x);
  float nx10 = mix(n010, n110, fade.x);
  float nx01 = mix(n001, n101, fade.x);
  float nx11 = mix(n011, n111, fade.x);
  float nxy0 = mix(nx00, nx10, fade.y);
  float nxy1 = mix(nx01, nx11, fade.y);
  return mix(nxy0, nxy1, fade.z);
}

float EE_VOLUMETRIC_CLOUD_Fbm(in vec3 position) {
  float value = 0.0f;
  float amplitude = 0.5f;
  float frequency = 1.0f;
  for (int octave_index = 0; octave_index < 5; ++octave_index) {
    value += amplitude * EE_VOLUMETRIC_CLOUD_ValueNoise(position * frequency);
    frequency *= 2.03f;
    amplitude *= 0.5f;
  }
  return value;
}

vec2 EE_VOLUMETRIC_CLOUD_TemporalJitter(in ivec2 pixel, in uint frame_index) {
  vec3 seed = vec3(float(pixel.x), float(pixel.y), float(frame_index & 1023u));
  return vec2(EE_VOLUMETRIC_CLOUD_Hash31(seed), EE_VOLUMETRIC_CLOUD_Hash31(seed + vec3(19.19f, 7.31f, 3.17f)));
}

float EE_VOLUMETRIC_CLOUD_HenyeyGreensteinPhase(in float cos_theta, in float anisotropy) {
  float g = clamp(anisotropy, -0.99f, 0.99f);
  float g2 = g * g;
  float denominator = max(pow(1.0f + g2 - 2.0f * g * cos_theta, 1.5f), 0.0001f);
  return (1.0f - g2) / (4.0f * EE_VOLUMETRIC_CLOUD_PI * denominator);
}

float EE_VOLUMETRIC_CLOUD_DirectionalPhase(in float cos_theta, in float anisotropy) {
  float forward_phase = EE_VOLUMETRIC_CLOUD_HenyeyGreensteinPhase(cos_theta, anisotropy);
  float broad_phase = EE_VOLUMETRIC_CLOUD_HenyeyGreensteinPhase(cos_theta, 0.15f);
  return max(forward_phase + broad_phase * 0.35f, 0.055f);
}

float EE_VOLUMETRIC_CLOUD_PowderEffect(in float density) {
  return mix(0.65f, 1.35f, 1.0f - exp(-EE_VOLUMETRIC_CLOUD_Saturate(density) * 3.0f));
}

float EE_VOLUMETRIC_CLOUD_EdgeLighting(in float density, in float light_transmittance, in float cos_theta) {
  float thin_edge = pow(1.0f - EE_VOLUMETRIC_CLOUD_Saturate(density), 2.0f);
  float forward_lit = EE_VOLUMETRIC_CLOUD_Saturate(cos_theta * 0.5f + 0.5f);
  float unshadowed = smoothstep(0.05f, 0.65f, light_transmittance);
  return thin_edge * forward_lit * unshadowed * 0.35f;
}

float EE_VOLUMETRIC_CLOUD_HeightFraction(in VolumetricCloudSettingsGpu settings, in float world_height) {
  float layer_thickness = max(settings.top_altitude - settings.bottom_altitude, 1.0f);
  return EE_VOLUMETRIC_CLOUD_Saturate((world_height - settings.bottom_altitude) / layer_thickness);
}

float EE_VOLUMETRIC_CLOUD_Remap(in float value, in float old_min, in float old_max, in float new_min,
                                in float new_max) {
  float denominator = max(old_max - old_min, 0.00001f);
  return new_min + EE_VOLUMETRIC_CLOUD_Saturate((value - old_min) / denominator) * (new_max - new_min);
}

float EE_VOLUMETRIC_CLOUD_HeightProfile(in float height_fraction) {
  float bottom_fade = EE_VOLUMETRIC_CLOUD_Saturate(height_fraction * 5.0f);
  float upper_fade = EE_VOLUMETRIC_CLOUD_Saturate((1.0f - height_fraction) * 3.5f);
  float body = mix(0.35f, 1.0f, EE_VOLUMETRIC_CLOUD_Saturate(height_fraction * 2.2f));
  return bottom_fade * upper_fade * body;
}

float EE_VOLUMETRIC_CLOUD_DetailErosionAmount(in float base_shape_density, in vec4 detail_noise) {
  float edge_weight = 1.0f - smoothstep(0.35f, 0.85f, base_shape_density);
  float detail_erosion = dot(detail_noise, vec4(0.45f, 0.30f, 0.18f, 0.07f));
  return detail_erosion * edge_weight * 0.35f;
}

float EE_VOLUMETRIC_CLOUD_EdgeErosion(in float base_shape_density, in vec4 detail_noise) {
  return EE_VOLUMETRIC_CLOUD_Saturate(base_shape_density -
                                      EE_VOLUMETRIC_CLOUD_DetailErosionAmount(base_shape_density, detail_noise));
}

vec3 EE_VOLUMETRIC_CLOUD_WindOffset(in VolumetricCloudSettingsGpu settings, in float time_seconds) {
  vec2 wind_direction = dot(settings.wind_direction, settings.wind_direction) > 0.0001f
                            ? normalize(settings.wind_direction)
                            : vec2(1.0f, 0.0f);
  return vec3(wind_direction.x, 0.0f, wind_direction.y) * max(settings.wind_speed, 0.0f) * time_seconds;
}

vec4 EE_VOLUMETRIC_CLOUD_SampleBaseShapeNoise(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                              in float time_seconds) {
  vec3 wind_position = world_position + EE_VOLUMETRIC_CLOUD_WindOffset(settings, time_seconds);
  return texture(EE_VOLUMETRIC_CLOUD_BASE_SHAPE_NOISE, wind_position * max(settings.base_noise_scale, 0.00001f));
}

vec4 EE_VOLUMETRIC_CLOUD_SampleDetailErosionNoise(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                                  in float time_seconds) {
  vec3 wind_position = world_position + EE_VOLUMETRIC_CLOUD_WindOffset(settings, time_seconds);
  return texture(EE_VOLUMETRIC_CLOUD_DETAIL_EROSION_NOISE,
                 wind_position * max(settings.detail_noise_scale, 0.00001f) + vec3(0.17f, 0.41f, 0.73f));
}

vec4 EE_VOLUMETRIC_CLOUD_SampleWeatherCoverage(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                               in float time_seconds) {
  vec3 wind_position = world_position + EE_VOLUMETRIC_CLOUD_WindOffset(settings, time_seconds) * 0.25f;
  return texture(EE_VOLUMETRIC_CLOUD_WEATHER_COVERAGE,
                 wind_position.xz * max(settings.base_noise_scale * 0.08f, 0.00001f));
}

VolumetricCloudDensitySample EE_VOLUMETRIC_CLOUD_SampleDensityComponents(in VolumetricCloudSettingsGpu settings,
                                                                         in vec3 world_position,
                                                                         in float time_seconds) {
  VolumetricCloudDensitySample sample;
  sample.final_density = 0.0f;
  sample.base_shape = 0.0f;
  sample.detail_erosion = 0.0f;
  sample.weather_coverage = 0.0f;

  float height_fraction = EE_VOLUMETRIC_CLOUD_HeightFraction(settings, world_position.y);
  if (height_fraction <= 0.0f || height_fraction >= 1.0f) {
    return sample;
  }

  vec4 base_noise = EE_VOLUMETRIC_CLOUD_SampleBaseShapeNoise(settings, world_position, time_seconds);
  vec4 detail_noise = EE_VOLUMETRIC_CLOUD_SampleDetailErosionNoise(settings, world_position, time_seconds);
  vec4 weather = EE_VOLUMETRIC_CLOUD_SampleWeatherCoverage(settings, world_position, time_seconds);
  float height_profile = EE_VOLUMETRIC_CLOUD_HeightProfile(height_fraction);
  float local_coverage = EE_VOLUMETRIC_CLOUD_Saturate(settings.coverage * mix(0.35f, 1.35f, weather.r));
  float coverage_threshold = 1.0f - local_coverage;
  float base_shape = dot(base_noise.rgb, vec3(0.60f, 0.25f, 0.15f));
  float base_shape_density = EE_VOLUMETRIC_CLOUD_Remap(base_shape, coverage_threshold, 1.0f, 0.0f, 1.0f);
  float detail_erosion = EE_VOLUMETRIC_CLOUD_DetailErosionAmount(base_shape_density, detail_noise);
  float eroded_shape_density = EE_VOLUMETRIC_CLOUD_EdgeErosion(base_shape_density, detail_noise);
  float weather_density = mix(0.75f, 1.20f, weather.b);
  sample.final_density = EE_VOLUMETRIC_CLOUD_Saturate(eroded_shape_density * height_profile * weather_density *
                                                      max(settings.density, 0.0f));
  sample.base_shape = base_shape_density;
  sample.detail_erosion = detail_erosion;
  sample.weather_coverage = local_coverage;
  return sample;
}

float EE_VOLUMETRIC_CLOUD_SampleDensity(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                        in float time_seconds) {
  return EE_VOLUMETRIC_CLOUD_SampleDensityComponents(settings, world_position, time_seconds).final_density;
}

VolumetricCloudRayInterval EE_VOLUMETRIC_CLOUD_IntersectLayer(in vec3 ray_origin, in vec3 ray_direction,
                                                              in VolumetricCloudSettingsGpu settings,
                                                              in float max_distance) {
  VolumetricCloudRayInterval interval;
  interval.t_min = 0.0f;
  interval.t_max = 0.0f;
  interval.valid = 0;

  float capped_distance = max(max_distance, 0.0f);
  if (capped_distance <= 0.0f) {
    return interval;
  }

  if (abs(ray_direction.y) < 0.00001f) {
    if (ray_origin.y >= settings.bottom_altitude && ray_origin.y <= settings.top_altitude) {
      interval.t_max = capped_distance;
      interval.valid = 1;
    }
    return interval;
  }

  float t0 = (settings.bottom_altitude - ray_origin.y) / ray_direction.y;
  float t1 = (settings.top_altitude - ray_origin.y) / ray_direction.y;
  if (t0 > t1) {
    float temp = t0;
    t0 = t1;
    t1 = temp;
  }

  interval.t_min = max(t0, 0.0f);
  interval.t_max = min(t1, capped_distance);
  interval.valid = interval.t_max > interval.t_min ? 1 : 0;
  return interval;
}

float EE_VOLUMETRIC_CLOUD_LightTransmittance(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                             in vec3 light_direction, in float time_seconds) {
  VolumetricCloudRayInterval interval =
      EE_VOLUMETRIC_CLOUD_IntersectLayer(world_position, light_direction, settings, 1000000.0f);
  if (interval.valid == 0) {
    return 1.0f;
  }

  int step_count = clamp(settings.light_step_count, 1, 128);
  float step_length = (interval.t_max - interval.t_min) / float(step_count);
  float optical_depth = 0.0f;
  for (int step_index = 0; step_index < step_count; ++step_index) {
    float t = interval.t_min + (float(step_index) + 0.5f) * step_length;
    vec3 sample_position = world_position + light_direction * t;
    optical_depth += EE_VOLUMETRIC_CLOUD_SampleDensity(settings, sample_position, time_seconds) * step_length *
                     max(settings.extinction_scale, 0.00001f);
  }
  return exp(-optical_depth);
}

VolumetricCloudMarchResult EE_VOLUMETRIC_CLOUD_March(in VolumetricCloudSettingsGpu settings, in vec3 ray_origin,
                                                     in vec3 ray_direction, in float max_distance,
                                                     in float scene_distance, in vec3 sun_direction,
                                                     in vec3 sun_radiance, in vec3 ambient_radiance,
                                                     in float time_seconds, in float jitter) {
  VolumetricCloudMarchResult result;
  result.radiance = vec3(0.0f);
  result.transmittance = 1.0f;
  result.mean_density = 0.0f;
  result.march_distance = 0.0f;
  result.mean_base_shape = 0.0f;
  result.mean_detail_erosion = 0.0f;
  result.mean_weather_coverage = 0.0f;

  if (settings.enabled == 0) {
    return result;
  }

  float capped_distance = scene_distance > 0.0f ? min(max_distance, scene_distance) : max_distance;
  VolumetricCloudRayInterval interval =
      EE_VOLUMETRIC_CLOUD_IntersectLayer(ray_origin, ray_direction, settings, capped_distance);
  if (interval.valid == 0) {
    return result;
  }

  int step_count = clamp(settings.primary_step_count, 1, 512);
  float step_length = (interval.t_max - interval.t_min) / float(step_count);
  float cos_theta = dot(ray_direction, sun_direction);
  float phase = EE_VOLUMETRIC_CLOUD_DirectionalPhase(cos_theta, settings.phase_anisotropy);
  float density_sum = 0.0f;
  float base_shape_sum = 0.0f;
  float detail_erosion_sum = 0.0f;
  float weather_coverage_sum = 0.0f;
  float used_steps = 0.0f;

  for (int step_index = 0; step_index < step_count; ++step_index) {
    float t = interval.t_min + (float(step_index) + EE_VOLUMETRIC_CLOUD_Saturate(jitter)) * step_length;
    if (t > interval.t_max || result.transmittance <= EE_VOLUMETRIC_CLOUD_MIN_TRANSMITTANCE) {
      break;
    }

    vec3 sample_position = ray_origin + ray_direction * t;
    VolumetricCloudDensitySample density_sample =
        EE_VOLUMETRIC_CLOUD_SampleDensityComponents(settings, sample_position, time_seconds);
    float density = density_sample.final_density;
    density_sum += density;
    base_shape_sum += density_sample.base_shape;
    detail_erosion_sum += density_sample.detail_erosion;
    weather_coverage_sum += density_sample.weather_coverage;
    used_steps += 1.0f;
    if (density <= 0.0f) {
      continue;
    }

    float sample_extinction = density * step_length * max(settings.extinction_scale, 0.00001f);
    float sample_transmittance = exp(-sample_extinction);
    float light_transmittance =
        EE_VOLUMETRIC_CLOUD_LightTransmittance(settings, sample_position, sun_direction, time_seconds);
    float powder = EE_VOLUMETRIC_CLOUD_PowderEffect(density);
    float edge_lighting = EE_VOLUMETRIC_CLOUD_EdgeLighting(density, light_transmittance, cos_theta);
    vec3 ambient_lighting = ambient_radiance * settings.ambient_lighting_strength * mix(1.0f, 0.55f, density);
    vec3 sun_lighting =
        sun_radiance * (light_transmittance * phase * powder + edge_lighting) * settings.lighting_intensity;
    vec3 lighting = ambient_lighting + sun_lighting;
    result.radiance += result.transmittance * (1.0f - sample_transmittance) * lighting;
    result.transmittance *= sample_transmittance;
    result.march_distance = t;
  }

  result.mean_density = used_steps > 0.0f ? density_sum / used_steps : 0.0f;
  result.mean_base_shape = used_steps > 0.0f ? base_shape_sum / used_steps : 0.0f;
  result.mean_detail_erosion = used_steps > 0.0f ? detail_erosion_sum / used_steps : 0.0f;
  result.mean_weather_coverage = used_steps > 0.0f ? weather_coverage_sum / used_steps : 0.0f;
  result.transmittance = EE_VOLUMETRIC_CLOUD_Saturate(result.transmittance);
  return result;
}

vec3 EE_VOLUMETRIC_CLOUD_Composite(in vec3 scene_color, in VolumetricCloudMarchResult cloud) {
  return cloud.radiance + scene_color * cloud.transmittance;
}

vec3 EE_VOLUMETRIC_CLOUD_DebugAccumulation(in VolumetricCloudMarchResult cloud, in int debug_mode) {
  if (debug_mode == 1) {
    return vec3(cloud.mean_density);
  }
  if (debug_mode == 4) {
    return vec3(cloud.mean_base_shape);
  }
  if (debug_mode == 5) {
    return vec3(EE_VOLUMETRIC_CLOUD_Saturate(cloud.mean_detail_erosion * 4.0f));
  }
  if (debug_mode == 6) {
    return vec3(cloud.mean_weather_coverage);
  }
  return cloud.radiance;
}

vec4 EE_VOLUMETRIC_CLOUD_Debug(in vec3 scene_color, in VolumetricCloudMarchResult cloud, in float max_distance,
                               in int debug_mode) {
  if (debug_mode == 1) {
    return vec4(vec3(cloud.mean_density), 1.0f);
  }
  if (debug_mode == 2) {
    return vec4(vec3(cloud.transmittance), 1.0f);
  }
  if (debug_mode == 3) {
    return vec4(vec3(EE_VOLUMETRIC_CLOUD_Saturate(cloud.march_distance / max(max_distance, 1.0f))), 1.0f);
  }
  if (debug_mode == 4 || debug_mode == 5 || debug_mode == 6) {
    return vec4(EE_VOLUMETRIC_CLOUD_DebugAccumulation(cloud, debug_mode), 1.0f);
  }
  return vec4(EE_VOLUMETRIC_CLOUD_Composite(scene_color, cloud), 1.0f);
}

#endif
