#ifndef _VOLUMETRIC_CLOUDS_GLSL_
#define _VOLUMETRIC_CLOUDS_GLSL_

const float EE_VOLUMETRIC_CLOUD_PI = 3.14159265359f;
const float EE_VOLUMETRIC_CLOUD_MIN_TRANSMITTANCE = 0.003f;
const float EE_VOLUMETRIC_CLOUD_EPSILON = 0.00001f;

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
  float max_march_distance;
  int use_spherical_atmosphere;
  float atmosphere_radius;
  float cloud_type;
  float curl_strength;
  float coarse_step_fraction;
  float fine_step_scale;
  int empty_step_fallback_count;
  int enable_cloud_shadows;
  float cloud_shadow_strength;
  int cloud_shadow_step_count;
  int debug_mode;
};

struct VolumetricCloudRayInterval {
  float t_min;
  float t_max;
  int valid;
  vec3 atmosphere_center;
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
  float low_resolution_density;
  float base_shape;
  float detail_erosion;
  float weather_coverage;
  float relative_height;
  float cloud_type;
};

layout(set = 1, binding = 6) uniform sampler3D EE_VOLUMETRIC_CLOUD_BASE_SHAPE_NOISE;
layout(set = 1, binding = 7) uniform sampler3D EE_VOLUMETRIC_CLOUD_DETAIL_EROSION_NOISE;
layout(set = 1, binding = 8) uniform sampler2D EE_VOLUMETRIC_CLOUD_WEATHER_COVERAGE;
layout(set = 1, binding = 9) uniform sampler2D EE_VOLUMETRIC_CLOUD_CURL_NOISE;

float EE_VOLUMETRIC_CLOUD_Saturate(in float value) {
  return clamp(value, 0.0f, 1.0f);
}

float EE_VOLUMETRIC_CLOUD_Hash31(in vec3 value) {
  return fract(sin(dot(value, vec3(127.1f, 311.7f, 74.7f))) * 43758.5453123f);
}

vec2 EE_VOLUMETRIC_CLOUD_TemporalJitter(in ivec2 pixel, in uint frame_index) {
  vec3 seed = vec3(float(pixel.x), float(pixel.y), float(frame_index & 1023u));
  return vec2(EE_VOLUMETRIC_CLOUD_Hash31(seed), EE_VOLUMETRIC_CLOUD_Hash31(seed + vec3(19.19f, 7.31f, 3.17f)));
}

float EE_VOLUMETRIC_CLOUD_Remap(in float value, in float old_min, in float old_max, in float new_min,
                                in float new_max) {
  float denominator = max(old_max - old_min, EE_VOLUMETRIC_CLOUD_EPSILON);
  return clamp(new_min + ((value - old_min) / denominator) * (new_max - new_min), min(new_min, new_max),
               max(new_min, new_max));
}

float EE_VOLUMETRIC_CLOUD_HenyeyGreensteinPhase(in float cos_theta, in float anisotropy) {
  float g = clamp(anisotropy, -0.99f, 0.99f);
  float g2 = g * g;
  float denominator = max(pow(1.0f + g2 - 2.0f * g * cos_theta, 1.5f), EE_VOLUMETRIC_CLOUD_EPSILON);
  return (1.0f - g2) / (4.0f * EE_VOLUMETRIC_CLOUD_PI * denominator);
}

float EE_VOLUMETRIC_CLOUD_DirectionalPhase(in float cos_theta, in float anisotropy) {
  float forward_phase = EE_VOLUMETRIC_CLOUD_HenyeyGreensteinPhase(cos_theta, anisotropy);
  float silver_lining_phase = 0.7f * EE_VOLUMETRIC_CLOUD_HenyeyGreensteinPhase(cos_theta, 0.89f);
  return max(max(forward_phase, silver_lining_phase), 0.055f);
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

float EE_VOLUMETRIC_CLOUD_LayerThickness(in VolumetricCloudSettingsGpu settings) {
  return max(settings.top_altitude - settings.bottom_altitude, 1.0f);
}

float EE_VOLUMETRIC_CLOUD_InnerAtmosphereRadius(in VolumetricCloudSettingsGpu settings) {
  float layer_thickness = EE_VOLUMETRIC_CLOUD_LayerThickness(settings);
  return max(settings.atmosphere_radius * 0.5f, layer_thickness * 2.0f + 1.0f);
}

vec3 EE_VOLUMETRIC_CLOUD_AtmosphereCenter(in VolumetricCloudSettingsGpu settings, in vec3 reference_origin) {
  vec3 center = reference_origin;
  center.y = settings.bottom_altitude - EE_VOLUMETRIC_CLOUD_InnerAtmosphereRadius(settings);
  return center;
}

void EE_VOLUMETRIC_CLOUD_RaySphereIntersection(in vec3 ray_origin, in vec3 ray_direction, in vec3 center,
                                               in float radius, out int hit, out float t_near, out float t_far) {
  vec3 oc = ray_origin - center;
  float b = dot(oc, ray_direction);
  float c = dot(oc, oc) - radius * radius;
  float h = b * b - c;
  hit = h >= 0.0f ? 1 : 0;
  if (hit == 0) {
    t_near = 0.0f;
    t_far = 0.0f;
    return;
  }
  h = sqrt(h);
  t_near = -b - h;
  t_far = -b + h;
}

float EE_VOLUMETRIC_CLOUD_HeightFraction(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                         in vec3 atmosphere_center) {
  if (settings.use_spherical_atmosphere != 0) {
    float inner_radius = EE_VOLUMETRIC_CLOUD_InnerAtmosphereRadius(settings);
    float layer_thickness = EE_VOLUMETRIC_CLOUD_LayerThickness(settings);
    return EE_VOLUMETRIC_CLOUD_Saturate((length(world_position - atmosphere_center) - inner_radius) /
                                        layer_thickness);
  }
  float layer_thickness = EE_VOLUMETRIC_CLOUD_LayerThickness(settings);
  return EE_VOLUMETRIC_CLOUD_Saturate((world_position.y - settings.bottom_altitude) / layer_thickness);
}

vec3 EE_VOLUMETRIC_CLOUD_ProjectedShellPoint(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                             in vec3 atmosphere_center) {
  if (settings.use_spherical_atmosphere != 0) {
    return atmosphere_center + EE_VOLUMETRIC_CLOUD_InnerAtmosphereRadius(settings) *
                                   normalize(world_position - atmosphere_center);
  }
  return vec3(world_position.x, settings.bottom_altitude, world_position.z);
}

float EE_VOLUMETRIC_CLOUD_CloudLayerDensity(in float relative_height, in float cloud_type) {
  relative_height = EE_VOLUMETRIC_CLOUD_Saturate(relative_height);
  cloud_type = EE_VOLUMETRIC_CLOUD_Saturate(cloud_type);

  float cumulus = max(0.0f, EE_VOLUMETRIC_CLOUD_Remap(relative_height, 0.0f, 0.2f, 0.0f, 1.0f) *
                                EE_VOLUMETRIC_CLOUD_Remap(relative_height, 0.7f, 0.9f, 1.0f, 0.0f));
  float stratocumulus = max(0.0f, EE_VOLUMETRIC_CLOUD_Remap(relative_height, 0.0f, 0.2f, 0.0f, 1.0f) *
                                      EE_VOLUMETRIC_CLOUD_Remap(relative_height, 0.2f, 0.7f, 1.0f, 0.0f));
  float stratus = max(0.0f, EE_VOLUMETRIC_CLOUD_Remap(relative_height, 0.0f, 0.1f, 0.0f, 1.0f) *
                                EE_VOLUMETRIC_CLOUD_Remap(relative_height, 0.2f, 0.3f, 1.0f, 0.0f));

  float low_mix = mix(stratus, stratocumulus, EE_VOLUMETRIC_CLOUD_Saturate(cloud_type * 2.0f));
  float high_mix = mix(stratocumulus, cumulus, EE_VOLUMETRIC_CLOUD_Saturate((cloud_type - 0.5f) * 2.0f));
  return mix(low_mix, high_mix, cloud_type);
}

float EE_VOLUMETRIC_CLOUD_HeightBiasCoverage(in float coverage, in float relative_height) {
  return pow(EE_VOLUMETRIC_CLOUD_Saturate(coverage),
             clamp(EE_VOLUMETRIC_CLOUD_Remap(relative_height, 0.7f, 0.8f, 1.0f, 0.8f), 0.8f, 1.0f));
}

vec3 EE_VOLUMETRIC_CLOUD_WindOffset(in VolumetricCloudSettingsGpu settings, in float time_seconds,
                                    in float relative_height) {
  vec2 wind_direction = dot(settings.wind_direction, settings.wind_direction) > 0.0001f
                            ? normalize(settings.wind_direction)
                            : vec2(1.0f, 0.0f);
  vec3 base_wind = vec3(wind_direction.x, 0.0f, wind_direction.y);
  vec3 height_wind = relative_height * vec3(0.1f, 0.05f, 0.0f);
  return (base_wind + height_wind) * max(settings.wind_speed, 0.0f) * (time_seconds + relative_height * 20.0f);
}

vec4 EE_VOLUMETRIC_CLOUD_SampleBaseShapeNoise(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                              in float time_seconds, in float relative_height) {
  vec3 wind_position = world_position + EE_VOLUMETRIC_CLOUD_WindOffset(settings, time_seconds, relative_height);
  return texture(EE_VOLUMETRIC_CLOUD_BASE_SHAPE_NOISE, wind_position * max(settings.base_noise_scale, 0.00001f));
}

vec4 EE_VOLUMETRIC_CLOUD_SampleWeatherCoverage(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                               in vec3 atmosphere_center, in float time_seconds,
                                               in float relative_height) {
  vec3 projected_position = EE_VOLUMETRIC_CLOUD_ProjectedShellPoint(settings, world_position, atmosphere_center);
  vec3 wind_position = projected_position + EE_VOLUMETRIC_CLOUD_WindOffset(settings, time_seconds, relative_height);
  return texture(EE_VOLUMETRIC_CLOUD_WEATHER_COVERAGE,
                 (wind_position.xz - atmosphere_center.xz) * max(settings.base_noise_scale * 0.08f, 0.00001f));
}

vec4 EE_VOLUMETRIC_CLOUD_SampleDetailErosionNoise(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                                  in vec3 atmosphere_center, in float time_seconds,
                                                  in float relative_height) {
  vec3 projected_position = EE_VOLUMETRIC_CLOUD_ProjectedShellPoint(settings, world_position, atmosphere_center);
  vec3 curl = texture(EE_VOLUMETRIC_CLOUD_CURL_NOISE,
                      (projected_position.xz - atmosphere_center.xz) *
                          max(settings.base_noise_scale * 0.20f, 0.00001f))
                  .rgb *
              2.0f - vec3(1.0f);
  vec3 wind_position = world_position + EE_VOLUMETRIC_CLOUD_WindOffset(settings, time_seconds, relative_height);
  wind_position += vec3(curl.x, curl.y * 0.2f, curl.z) * max(settings.curl_strength, 0.0f);
  return texture(EE_VOLUMETRIC_CLOUD_DETAIL_EROSION_NOISE,
                 wind_position * max(settings.detail_noise_scale, 0.00001f) + vec3(0.17f, 0.41f, 0.73f));
}

VolumetricCloudDensitySample EE_VOLUMETRIC_CLOUD_SampleDensityComponents(in VolumetricCloudSettingsGpu settings,
                                                                         in vec3 world_position,
                                                                         in vec3 atmosphere_center,
                                                                         in float time_seconds,
                                                                         in int use_high_detail) {
  VolumetricCloudDensitySample density_result;
  density_result.final_density = 0.0f;
  density_result.low_resolution_density = 0.0f;
  density_result.base_shape = 0.0f;
  density_result.detail_erosion = 0.0f;
  density_result.weather_coverage = 0.0f;
  density_result.relative_height = 0.0f;
  density_result.cloud_type = 0.0f;

  float relative_height = EE_VOLUMETRIC_CLOUD_HeightFraction(settings, world_position, atmosphere_center);
  density_result.relative_height = relative_height;
  if (relative_height <= 0.0f || relative_height >= 1.0f) {
    return density_result;
  }

  vec4 weather = EE_VOLUMETRIC_CLOUD_SampleWeatherCoverage(settings, world_position, atmosphere_center, time_seconds,
                                                           relative_height);
  float local_coverage = EE_VOLUMETRIC_CLOUD_Saturate(settings.coverage * mix(0.35f, 1.35f, weather.r));
  float cloud_type = EE_VOLUMETRIC_CLOUD_Saturate(mix(settings.cloud_type, weather.g, 0.65f));
  float layer_density = EE_VOLUMETRIC_CLOUD_CloudLayerDensity(relative_height, cloud_type);
  vec4 low_res_noise = EE_VOLUMETRIC_CLOUD_SampleBaseShapeNoise(settings, world_position, time_seconds,
                                                                relative_height);

  float low_shape_density = layer_density * EE_VOLUMETRIC_CLOUD_Remap(low_res_noise.r, 0.3f, 1.0f, 0.0f, 1.0f);
  density_result.base_shape = low_shape_density;
  density_result.weather_coverage = local_coverage;
  density_result.cloud_type = cloud_type;
  if (low_shape_density < 0.0001f) {
    return density_result;
  }

  float height_coverage = EE_VOLUMETRIC_CLOUD_HeightBiasCoverage(min(0.85f, local_coverage), relative_height);
  float low_res_erosion = dot(low_res_noise.gba, vec3(0.625f, 0.25f, 0.125f));
  low_res_erosion = EE_VOLUMETRIC_CLOUD_Remap(low_res_erosion, height_coverage, 1.0f, 0.0f, 1.0f);
  float low_resolution_density = EE_VOLUMETRIC_CLOUD_Remap(low_shape_density, low_res_erosion, 1.0f, 0.0f, 1.0f);
  density_result.low_resolution_density = low_resolution_density;
  density_result.detail_erosion = low_res_erosion;

  if (low_resolution_density < 0.0001f) {
    return density_result;
  }

  float final_density = low_resolution_density;
  if (use_high_detail != 0) {
    vec4 high_res_noise = EE_VOLUMETRIC_CLOUD_SampleDetailErosionNoise(settings, world_position, atmosphere_center,
                                                                       time_seconds, relative_height);
    float high_res_erosion = dot(high_res_noise.rgb, vec3(0.625f, 0.25f, 0.125f));
    high_res_erosion = mix(high_res_erosion, 1.0f - high_res_erosion, EE_VOLUMETRIC_CLOUD_Saturate(relative_height * 10.0f));
    final_density = EE_VOLUMETRIC_CLOUD_Remap(low_resolution_density, high_res_erosion, 1.0f, 0.0f, 1.0f);
    density_result.detail_erosion = high_res_erosion;
  }

  float weather_density = mix(0.75f, 1.20f, weather.b);
  density_result.final_density =
      EE_VOLUMETRIC_CLOUD_Saturate(final_density * weather_density * max(settings.density, 0.0f));
  return density_result;
}

float EE_VOLUMETRIC_CLOUD_SampleDensity(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                        in vec3 atmosphere_center, in float time_seconds) {
  return EE_VOLUMETRIC_CLOUD_SampleDensityComponents(settings, world_position, atmosphere_center, time_seconds, 1)
      .final_density;
}

VolumetricCloudRayInterval EE_VOLUMETRIC_CLOUD_IntersectLayer(in vec3 ray_origin, in vec3 ray_direction,
                                                              in VolumetricCloudSettingsGpu settings,
                                                              in float max_distance) {
  VolumetricCloudRayInterval interval;
  interval.t_min = 0.0f;
  interval.t_max = 0.0f;
  interval.valid = 0;
  interval.atmosphere_center = EE_VOLUMETRIC_CLOUD_AtmosphereCenter(settings, ray_origin);

  float capped_distance = max(max_distance, 0.0f);
  if (capped_distance <= 0.0f) {
    return interval;
  }

  if (settings.use_spherical_atmosphere != 0) {
    if (ray_direction.y <= 0.0f) {
      return interval;
    }
    float inner_radius = EE_VOLUMETRIC_CLOUD_InnerAtmosphereRadius(settings);
    float outer_radius = inner_radius + EE_VOLUMETRIC_CLOUD_LayerThickness(settings);
    int outer_hit;
    float outer_near;
    float outer_far;
    EE_VOLUMETRIC_CLOUD_RaySphereIntersection(ray_origin, ray_direction, interval.atmosphere_center, outer_radius,
                                              outer_hit, outer_near, outer_far);
    if (outer_hit == 0 || outer_far <= 0.0f) {
      return interval;
    }

    float origin_radius = length(ray_origin - interval.atmosphere_center);
    float t_min = max(outer_near, 0.0f);
    if (origin_radius < inner_radius) {
      int inner_hit;
      float inner_near;
      float inner_far;
      EE_VOLUMETRIC_CLOUD_RaySphereIntersection(ray_origin, ray_direction, interval.atmosphere_center, inner_radius,
                                                inner_hit, inner_near, inner_far);
      if (inner_hit == 0) {
        return interval;
      }
      t_min = max(inner_far, 0.0f);
    } else if (origin_radius < outer_radius) {
      t_min = 0.0f;
    }

    interval.t_min = max(t_min, 0.0f);
    interval.t_max = min(outer_far, capped_distance);
    interval.valid = interval.t_max > interval.t_min ? 1 : 0;
    return interval;
  }

  if (abs(ray_direction.y) < EE_VOLUMETRIC_CLOUD_EPSILON) {
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

VolumetricCloudRayInterval EE_VOLUMETRIC_CLOUD_IntersectLayerWithCenter(
    in vec3 ray_origin, in vec3 ray_direction, in VolumetricCloudSettingsGpu settings, in float max_distance,
    in vec3 atmosphere_center) {
  VolumetricCloudRayInterval interval;
  interval.t_min = 0.0f;
  interval.t_max = 0.0f;
  interval.valid = 0;
  interval.atmosphere_center = atmosphere_center;

  float capped_distance = max(max_distance, 0.0f);
  if (capped_distance <= 0.0f) {
    return interval;
  }

  if (settings.use_spherical_atmosphere != 0) {
    float inner_radius = EE_VOLUMETRIC_CLOUD_InnerAtmosphereRadius(settings);
    float outer_radius = inner_radius + EE_VOLUMETRIC_CLOUD_LayerThickness(settings);
    int outer_hit;
    float outer_near;
    float outer_far;
    EE_VOLUMETRIC_CLOUD_RaySphereIntersection(ray_origin, ray_direction, atmosphere_center, outer_radius, outer_hit,
                                              outer_near, outer_far);
    if (outer_hit == 0 || outer_far <= 0.0f) {
      return interval;
    }

    float t_min = max(outer_near, 0.0f);
    int inner_hit;
    float inner_near;
    float inner_far;
    EE_VOLUMETRIC_CLOUD_RaySphereIntersection(ray_origin, ray_direction, atmosphere_center, inner_radius, inner_hit,
                                              inner_near, inner_far);
    if (inner_hit != 0 && inner_far > 0.0f && length(ray_origin - atmosphere_center) < inner_radius) {
      t_min = max(inner_far, 0.0f);
    }

    interval.t_min = t_min;
    interval.t_max = min(outer_far, capped_distance);
    interval.valid = interval.t_max > interval.t_min ? 1 : 0;
    return interval;
  }

  if (abs(ray_direction.y) < EE_VOLUMETRIC_CLOUD_EPSILON) {
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

float EE_VOLUMETRIC_CLOUD_SurfaceShadowTransmittance(in VolumetricCloudSettingsGpu settings, in vec3 surface_position,
                                                     in vec3 reference_origin, in vec3 sun_direction,
                                                     in float time_seconds) {
  if (settings.enabled == 0 || settings.enable_cloud_shadows == 0 || settings.cloud_shadow_strength <= 0.0f) {
    return 1.0f;
  }
  vec3 ray_direction = normalize(sun_direction);
  vec3 ray_origin = surface_position + ray_direction * 0.25f;
  vec3 atmosphere_center = EE_VOLUMETRIC_CLOUD_AtmosphereCenter(settings, reference_origin);
  float shadow_distance = max(settings.max_march_distance, EE_VOLUMETRIC_CLOUD_LayerThickness(settings) * 2.0f);
  VolumetricCloudRayInterval interval =
      EE_VOLUMETRIC_CLOUD_IntersectLayerWithCenter(ray_origin, ray_direction, settings, shadow_distance,
                                                   atmosphere_center);
  if (interval.valid == 0) {
    return 1.0f;
  }

  int step_count = clamp(settings.cloud_shadow_step_count, 1, 64);
  float step_length = max((interval.t_max - interval.t_min) / float(step_count), EE_VOLUMETRIC_CLOUD_EPSILON);
  float t = interval.t_min + step_length * 0.5f;
  float optical_depth = 0.0f;
  for (int step_index = 0; step_index < 64; ++step_index) {
    if (step_index >= step_count || t > interval.t_max) {
      break;
    }
    vec3 sample_position = ray_origin + ray_direction * t;
    VolumetricCloudDensitySample density_sample =
        EE_VOLUMETRIC_CLOUD_SampleDensityComponents(settings, sample_position, atmosphere_center, time_seconds, 0);
    optical_depth += density_sample.low_resolution_density * step_length;
    t += step_length;
  }
  return EE_VOLUMETRIC_CLOUD_Saturate(exp(-optical_depth * max(settings.extinction_scale, 0.00001f) * 1.5f));
}

vec3 EE_VOLUMETRIC_CLOUD_ConeSample(in int sample_index) {
  if (sample_index == 0) {
    return vec3(0.0f, 0.6f, 0.0f);
  }
  if (sample_index == 1) {
    return vec3(0.0f, 0.5f, 0.05f);
  }
  if (sample_index == 2) {
    return vec3(0.1f, 0.75f, 0.0f);
  }
  if (sample_index == 3) {
    return vec3(0.2f, 2.5f, 0.3f);
  }
  if (sample_index == 4) {
    return vec3(0.0f, 6.0f, 0.0f);
  }
  return vec3(-0.1f, 1.0f, -0.2f);
}

float EE_VOLUMETRIC_CLOUD_LightDensity(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                       in vec3 atmosphere_center, in vec3 light_direction, in float time_seconds,
                                       in float primary_step_length) {
  vec3 basis_up = abs(light_direction.y) < 0.95f ? vec3(0.0f, 1.0f, 0.0f) : vec3(1.0f, 0.0f, 0.0f);
  vec3 basis_right = normalize(cross(basis_up, light_direction));
  basis_up = normalize(cross(light_direction, basis_right));
  int sample_count = clamp(settings.light_step_count, 1, 6);
  float density_along_light = 0.0f;
  for (int sample_index = 0; sample_index < 6; ++sample_index) {
    if (sample_index >= sample_count) {
      break;
    }
    vec3 cone_sample = EE_VOLUMETRIC_CLOUD_ConeSample(sample_index);
    vec3 sample_offset = basis_right * cone_sample.x + light_direction * cone_sample.y + basis_up * cone_sample.z;
    vec3 sample_position = world_position + 3.0f * primary_step_length * sample_offset;
    density_along_light += EE_VOLUMETRIC_CLOUD_SampleDensity(settings, sample_position, atmosphere_center,
                                                             time_seconds);
  }
  return density_along_light;
}

float EE_VOLUMETRIC_CLOUD_LightTransmittance(in VolumetricCloudSettingsGpu settings, in vec3 world_position,
                                             in vec3 atmosphere_center, in vec3 light_direction,
                                             in float time_seconds, in float primary_step_length,
                                             in float cos_theta) {
  float density_along_light = EE_VOLUMETRIC_CLOUD_LightDensity(settings, world_position, atmosphere_center,
                                                              light_direction, time_seconds, primary_step_length);
  float beers_law = exp(-density_along_light);
  float beers_modulated = max(beers_law, 0.7f * exp(-0.25f * density_along_light));
  return mix(beers_law, beers_modulated, -cos_theta * 0.5f + 0.5f);
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

  int max_steps = clamp(settings.primary_step_count, 1, 512);
  int fallback_misses = clamp(settings.empty_step_fallback_count, 1, 64);
  float interval_length = max(interval.t_max - interval.t_min, EE_VOLUMETRIC_CLOUD_EPSILON);
  float coarse_step_length = max(interval_length * clamp(settings.coarse_step_fraction, 0.0001f, 1.0f),
                                 interval_length / float(max_steps));
  float fine_step_length = max(coarse_step_length * clamp(settings.fine_step_scale, 0.01f, 1.0f),
                               interval_length / float(max_steps));
  float step_length = coarse_step_length;
  float t = interval.t_min + EE_VOLUMETRIC_CLOUD_Saturate(jitter) * step_length;
  float cos_theta = dot(ray_direction, sun_direction);
  float phase = EE_VOLUMETRIC_CLOUD_DirectionalPhase(cos_theta, settings.phase_anisotropy);
  float density_sum = 0.0f;
  float base_shape_sum = 0.0f;
  float detail_erosion_sum = 0.0f;
  float weather_coverage_sum = 0.0f;
  float used_steps = 0.0f;
  int high_resolution_march = 0;
  int misses = 0;

  for (int step_index = 0; step_index < 512; ++step_index) {
    if (step_index >= max_steps || t > interval.t_max ||
        result.transmittance <= EE_VOLUMETRIC_CLOUD_MIN_TRANSMITTANCE) {
      break;
    }

    vec3 sample_position = ray_origin + ray_direction * t;
    VolumetricCloudDensitySample density_sample = EE_VOLUMETRIC_CLOUD_SampleDensityComponents(
        settings, sample_position, interval.atmosphere_center, time_seconds, high_resolution_march);
    density_sum += density_sample.final_density;
    base_shape_sum += density_sample.base_shape;
    detail_erosion_sum += density_sample.detail_erosion;
    weather_coverage_sum += density_sample.weather_coverage;
    used_steps += 1.0f;

    if (density_sample.low_resolution_density > 0.0001f) {
      misses = 0;
      if (high_resolution_march == 0) {
        t = max(interval.t_min, t - step_length);
        step_length = fine_step_length;
        high_resolution_march = 1;
        continue;
      }

      float density = density_sample.final_density;
      if (density > 0.0001f) {
        float sample_extinction = density * step_length * max(settings.extinction_scale, 0.00001f);
        float sample_transmittance = exp(-sample_extinction);
        float light_transmittance = EE_VOLUMETRIC_CLOUD_LightTransmittance(
            settings, sample_position, interval.atmosphere_center, sun_direction, time_seconds, step_length, cos_theta);
        float in_scattering =
            0.09f + pow(max(density_sample.low_resolution_density, 0.0f),
                        EE_VOLUMETRIC_CLOUD_Remap(density_sample.relative_height, 0.3f, 0.85f, 0.5f, 2.0f));
        in_scattering *= pow(EE_VOLUMETRIC_CLOUD_Remap(density_sample.relative_height, 0.07f, 0.34f, 0.1f, 1.0f),
                             0.8f);
        float powder = EE_VOLUMETRIC_CLOUD_PowderEffect(density);
        float edge_lighting = EE_VOLUMETRIC_CLOUD_EdgeLighting(density, light_transmittance, cos_theta);
        vec3 ambient_lighting = ambient_radiance * settings.ambient_lighting_strength * mix(1.0f, 0.55f, density);
        vec3 sun_lighting =
            sun_radiance * settings.lighting_intensity *
            (light_transmittance * phase * in_scattering * powder + edge_lighting);
        result.radiance += result.transmittance * (1.0f - sample_transmittance) *
                           (ambient_lighting + sun_lighting);
        result.transmittance *= sample_transmittance;
        result.march_distance = t;
      }
    } else if (high_resolution_march != 0) {
      ++misses;
      if (misses >= fallback_misses) {
        high_resolution_march = 0;
        step_length = coarse_step_length;
      }
    }

    t += step_length;
  }

  float horizon_fade = settings.use_spherical_atmosphere != 0
                           ? smoothstep(0.0f, 0.1f, max(ray_direction.y, 0.0f))
                           : 1.0f;
  result.radiance *= horizon_fade;
  result.transmittance = mix(1.0f, EE_VOLUMETRIC_CLOUD_Saturate(result.transmittance), horizon_fade);
  result.mean_density = used_steps > 0.0f ? density_sum / used_steps : 0.0f;
  result.mean_base_shape = used_steps > 0.0f ? base_shape_sum / used_steps : 0.0f;
  result.mean_detail_erosion = used_steps > 0.0f ? detail_erosion_sum / used_steps : 0.0f;
  result.mean_weather_coverage = used_steps > 0.0f ? weather_coverage_sum / used_steps : 0.0f;
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
