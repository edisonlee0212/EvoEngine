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
};

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

float EE_VOLUMETRIC_CLOUD_HeightFraction(in VolumetricCloudSettingsGpu settings, in float world_height) {
  float layer_thickness = max(settings.top_altitude - settings.bottom_altitude, 1.0f);
  return EE_VOLUMETRIC_CLOUD_Saturate((world_height - settings.bottom_altitude) / layer_thickness);
}

vec3 EE_VOLUMETRIC_CLOUD_WindOffset(in VolumetricCloudSettingsGpu settings, in float time_seconds) {
  vec2 wind_direction = dot(settings.wind_direction, settings.wind_direction) > 0.0001f
                            ? normalize(settings.wind_direction)
                            : vec2(1.0f, 0.0f);
  return vec3(wind_direction.x, 0.0f, wind_direction.y) * max(settings.wind_speed, 0.0f) * time_seconds;
}

float EE_VOLUMETRIC_CLOUD_SampleDensity(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                        in float time_seconds) {
  float height_fraction = EE_VOLUMETRIC_CLOUD_HeightFraction(settings, world_position.y);
  if (height_fraction <= 0.0f || height_fraction >= 1.0f) {
    return 0.0f;
  }

  vec3 wind_position = world_position + EE_VOLUMETRIC_CLOUD_WindOffset(settings, time_seconds);
  float base_noise = EE_VOLUMETRIC_CLOUD_Fbm(wind_position * max(settings.base_noise_scale, 0.00001f));
  float detail_noise =
      EE_VOLUMETRIC_CLOUD_Fbm(wind_position * max(settings.detail_noise_scale, 0.00001f) + vec3(13.5f, 2.3f, 8.1f));
  float height_shape = EE_VOLUMETRIC_CLOUD_Saturate(height_fraction * 4.0f) *
                       EE_VOLUMETRIC_CLOUD_Saturate((1.0f - height_fraction) * 3.0f);
  float coverage_threshold = 1.0f - EE_VOLUMETRIC_CLOUD_Saturate(settings.coverage);
  float cloud_shape = base_noise * 0.78f + detail_noise * 0.22f;
  return EE_VOLUMETRIC_CLOUD_Saturate((cloud_shape - coverage_threshold) * max(settings.density, 0.0f) * height_shape);
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
  float phase = EE_VOLUMETRIC_CLOUD_HenyeyGreensteinPhase(dot(ray_direction, sun_direction), settings.phase_anisotropy);
  float density_sum = 0.0f;
  float used_steps = 0.0f;

  for (int step_index = 0; step_index < step_count; ++step_index) {
    float t = interval.t_min + (float(step_index) + EE_VOLUMETRIC_CLOUD_Saturate(jitter)) * step_length;
    if (t > interval.t_max || result.transmittance <= EE_VOLUMETRIC_CLOUD_MIN_TRANSMITTANCE) {
      break;
    }

    vec3 sample_position = ray_origin + ray_direction * t;
    float density = EE_VOLUMETRIC_CLOUD_SampleDensity(settings, sample_position, time_seconds);
    density_sum += density;
    used_steps += 1.0f;
    if (density <= 0.0f) {
      continue;
    }

    float sample_extinction = density * step_length * max(settings.extinction_scale, 0.00001f);
    float sample_transmittance = exp(-sample_extinction);
    float light_transmittance =
        EE_VOLUMETRIC_CLOUD_LightTransmittance(settings, sample_position, sun_direction, time_seconds);
    vec3 lighting = ambient_radiance * settings.ambient_lighting_strength +
                    sun_radiance * light_transmittance * phase * settings.lighting_intensity;
    result.radiance += result.transmittance * (1.0f - sample_transmittance) * lighting;
    result.transmittance *= sample_transmittance;
    result.march_distance = t;
  }

  result.mean_density = used_steps > 0.0f ? density_sum / used_steps : 0.0f;
  result.transmittance = EE_VOLUMETRIC_CLOUD_Saturate(result.transmittance);
  return result;
}

vec3 EE_VOLUMETRIC_CLOUD_Composite(in vec3 scene_color, in VolumetricCloudMarchResult cloud) {
  return cloud.radiance + scene_color * cloud.transmittance;
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
  return vec4(EE_VOLUMETRIC_CLOUD_Composite(scene_color, cloud), 1.0f);
}

#endif
