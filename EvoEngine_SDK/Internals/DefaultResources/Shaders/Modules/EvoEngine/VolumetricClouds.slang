
public static const float EE_VOLUMETRIC_CLOUD_PI = 3.14159265359f;
public static const float EE_VOLUMETRIC_CLOUD_MIN_TRANSMITTANCE = 0.003f;
public static const float EE_VOLUMETRIC_CLOUD_EPSILON = 0.00001f;

public struct VolumetricCloudSettingsGpu {
  public int enabled;
  public float coverage;
  public float density;
  public float bottom_altitude;
  public float top_altitude;
  public float2 wind_direction;
  public float wind_speed;
  public int primary_step_count;
  public int light_step_count;
  public float lighting_intensity;
  public float ambient_lighting_strength;
  public float phase_anisotropy;
  public float base_noise_scale;
  public float detail_noise_scale;
  public float extinction_scale;
  public float max_march_distance;
  public int use_spherical_atmosphere;
  public float atmosphere_radius;
  public float cloud_type;
  public float curl_strength;
  public float coarse_step_fraction;
  public float fine_step_scale;
  public int empty_step_fallback_count;
  public int enable_cloud_shadows;
  public float cloud_shadow_strength;
  public int cloud_shadow_step_count;
  public int debug_mode;
};

public struct VolumetricCloudRayInterval {
  public float t_min;
  public float t_max;
  public int valid;
  public float3 atmosphere_center;
};

public struct VolumetricCloudMarchResult {
  public float3 radiance;
  public float transmittance;
  public float mean_density;
  public float march_distance;
  public float mean_base_shape;
  public float mean_detail_erosion;
  public float mean_weather_coverage;
};

public struct VolumetricCloudDensitySample {
  public float final_density;
  public float low_resolution_density;
  public float base_shape;
  public float detail_erosion;
  public float weather_coverage;
  public float relative_height;
  public float cloud_type;
};

public struct EeVolumetricCloudConstants {
  public float4 altitudeCoverageDensity;
  public float4 windTime;
  public float4 lightingPhaseMaxDistance;
  public float4 noiseExtinctionMarchDistance;
  public float4 atmosphereCloudTypeCurl;
  public float4 marchControl;
  public int4 cameraFrameSteps;
  public int4 flags;
};

[[vk::push_constant]]
public ConstantBuffer<EeVolumetricCloudConstants> EE_VOLUMETRIC_CLOUD_CONSTANTS;

[[vk::binding(6, 1)]] Sampler3D EE_VOLUMETRIC_CLOUD_BASE_SHAPE_NOISE;
[[vk::binding(7, 1)]] Sampler3D EE_VOLUMETRIC_CLOUD_DETAIL_EROSION_NOISE;
[[vk::binding(8, 1)]] Sampler2D EE_VOLUMETRIC_CLOUD_WEATHER_COVERAGE;
[[vk::binding(9, 1)]] Sampler2D EE_VOLUMETRIC_CLOUD_CURL_NOISE;

public float EE_VOLUMETRIC_CLOUD_Saturate(in float value) {
  return clamp(value, 0.0f, 1.0f);
}

float EE_VOLUMETRIC_CLOUD_Hash31(in float3 value) {
  return frac(sin(dot(value, float3(127.1f, 311.7f, 74.7f))) * 43758.5453123f);
}

public float2 EE_VOLUMETRIC_CLOUD_TemporalJitter(in int2 pixel, in uint frame_index) {
  float3 seed = float3(float(pixel.x), float(pixel.y), float(frame_index & 1023u));
  return float2(EE_VOLUMETRIC_CLOUD_Hash31(seed), EE_VOLUMETRIC_CLOUD_Hash31(seed + float3(19.19f, 7.31f, 3.17f)));
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
  return lerp(0.65f, 1.35f, 1.0f - exp(-EE_VOLUMETRIC_CLOUD_Saturate(density) * 3.0f));
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

float3 EE_VOLUMETRIC_CLOUD_AtmosphereCenter(in VolumetricCloudSettingsGpu settings, in float3 reference_origin) {
  float3 center = reference_origin;
  center.y = settings.bottom_altitude - EE_VOLUMETRIC_CLOUD_InnerAtmosphereRadius(settings);
  return center;
}

void EE_VOLUMETRIC_CLOUD_RaySphereIntersection(in float3 ray_origin, in float3 ray_direction, in float3 center,
                                               in float radius, out int hit, out float t_near, out float t_far) {
  float3 oc = ray_origin - center;
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

float EE_VOLUMETRIC_CLOUD_HeightFraction(in VolumetricCloudSettingsGpu settings, in float3 world_position,
                                         in float3 atmosphere_center) {
  if (settings.use_spherical_atmosphere != 0) {
    float inner_radius = EE_VOLUMETRIC_CLOUD_InnerAtmosphereRadius(settings);
    float layer_thickness = EE_VOLUMETRIC_CLOUD_LayerThickness(settings);
    return EE_VOLUMETRIC_CLOUD_Saturate((length(world_position - atmosphere_center) - inner_radius) /
                                        layer_thickness);
  }
  float layer_thickness = EE_VOLUMETRIC_CLOUD_LayerThickness(settings);
  return EE_VOLUMETRIC_CLOUD_Saturate((world_position.y - settings.bottom_altitude) / layer_thickness);
}

float3 EE_VOLUMETRIC_CLOUD_ProjectedShellPoint(in VolumetricCloudSettingsGpu settings, in float3 world_position,
                                             in float3 atmosphere_center) {
  if (settings.use_spherical_atmosphere != 0) {
    return atmosphere_center + EE_VOLUMETRIC_CLOUD_InnerAtmosphereRadius(settings) *
                                   normalize(world_position - atmosphere_center);
  }
  return float3(world_position.x, settings.bottom_altitude, world_position.z);
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

  float low_mix = lerp(stratus, stratocumulus, EE_VOLUMETRIC_CLOUD_Saturate(cloud_type * 2.0f));
  float high_mix = lerp(stratocumulus, cumulus, EE_VOLUMETRIC_CLOUD_Saturate((cloud_type - 0.5f) * 2.0f));
  return lerp(low_mix, high_mix, cloud_type);
}

float EE_VOLUMETRIC_CLOUD_HeightBiasCoverage(in float coverage, in float relative_height) {
  return pow(EE_VOLUMETRIC_CLOUD_Saturate(coverage),
             clamp(EE_VOLUMETRIC_CLOUD_Remap(relative_height, 0.7f, 0.8f, 1.0f, 0.8f), 0.8f, 1.0f));
}

float3 EE_VOLUMETRIC_CLOUD_WindOffset(in VolumetricCloudSettingsGpu settings, in float time_seconds,
                                    in float relative_height) {
  float2 wind_direction = dot(settings.wind_direction, settings.wind_direction) > 0.0001f
                            ? normalize(settings.wind_direction)
                            : float2(1.0f, 0.0f);
  float3 base_wind = float3(wind_direction.x, 0.0f, wind_direction.y);
  float3 height_wind = relative_height * float3(0.1f, 0.05f, 0.0f);
  return (base_wind + height_wind) * max(settings.wind_speed, 0.0f) * (time_seconds + relative_height * 20.0f);
}

float4 EE_VOLUMETRIC_CLOUD_SampleBaseShapeNoise(in VolumetricCloudSettingsGpu settings, in float3 world_position,
                                              in float time_seconds, in float relative_height) {
  float3 wind_position = world_position + EE_VOLUMETRIC_CLOUD_WindOffset(settings, time_seconds, relative_height);
  return EE_VOLUMETRIC_CLOUD_BASE_SHAPE_NOISE.SampleLevel(
      wind_position * max(settings.base_noise_scale, 0.00001f), 0.0f);
}

float4 EE_VOLUMETRIC_CLOUD_SampleWeatherCoverage(in VolumetricCloudSettingsGpu settings, in float3 world_position,
                                               in float3 atmosphere_center, in float time_seconds,
                                               in float relative_height) {
  float3 projected_position = EE_VOLUMETRIC_CLOUD_ProjectedShellPoint(settings, world_position, atmosphere_center);
  float3 wind_position = projected_position + EE_VOLUMETRIC_CLOUD_WindOffset(settings, time_seconds, relative_height);
  return EE_VOLUMETRIC_CLOUD_WEATHER_COVERAGE.SampleLevel(
      (wind_position.xz - atmosphere_center.xz) * max(settings.base_noise_scale * 0.08f, 0.00001f), 0.0f);
}

float4 EE_VOLUMETRIC_CLOUD_SampleDetailErosionNoise(in VolumetricCloudSettingsGpu settings, in float3 world_position,
                                                  in float3 atmosphere_center, in float time_seconds,
                                                  in float relative_height) {
  float3 projected_position = EE_VOLUMETRIC_CLOUD_ProjectedShellPoint(settings, world_position, atmosphere_center);
  float3 curl = EE_VOLUMETRIC_CLOUD_CURL_NOISE.SampleLevel(
                    (projected_position.xz - atmosphere_center.xz) *
                        max(settings.base_noise_scale * 0.20f, 0.00001f),
                    0.0f)
                  .rgb *
              2.0f - float3(1.0f);
  float3 wind_position = world_position + EE_VOLUMETRIC_CLOUD_WindOffset(settings, time_seconds, relative_height);
  wind_position += float3(curl.x, curl.y * 0.2f, curl.z) * max(settings.curl_strength, 0.0f);
  return EE_VOLUMETRIC_CLOUD_DETAIL_EROSION_NOISE.SampleLevel(
      wind_position * max(settings.detail_noise_scale, 0.00001f) + float3(0.17f, 0.41f, 0.73f), 0.0f);
}

VolumetricCloudDensitySample EE_VOLUMETRIC_CLOUD_SampleDensityComponents(in VolumetricCloudSettingsGpu settings,
                                                                         in float3 world_position,
                                                                         in float3 atmosphere_center,
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

  float4 weather = EE_VOLUMETRIC_CLOUD_SampleWeatherCoverage(settings, world_position, atmosphere_center, time_seconds,
                                                           relative_height);
  float local_coverage = EE_VOLUMETRIC_CLOUD_Saturate(settings.coverage * lerp(0.35f, 1.35f, weather.r));
  float cloud_type = EE_VOLUMETRIC_CLOUD_Saturate(lerp(settings.cloud_type, weather.g, 0.65f));
  float layer_density = EE_VOLUMETRIC_CLOUD_CloudLayerDensity(relative_height, cloud_type);
  float4 low_res_noise = EE_VOLUMETRIC_CLOUD_SampleBaseShapeNoise(settings, world_position, time_seconds,
                                                                relative_height);

  float low_shape_density = layer_density * EE_VOLUMETRIC_CLOUD_Remap(low_res_noise.r, 0.3f, 1.0f, 0.0f, 1.0f);
  density_result.base_shape = low_shape_density;
  density_result.weather_coverage = local_coverage;
  density_result.cloud_type = cloud_type;
  if (low_shape_density < 0.0001f) {
    return density_result;
  }

  float height_coverage = EE_VOLUMETRIC_CLOUD_HeightBiasCoverage(min(0.85f, local_coverage), relative_height);
  float low_res_erosion = dot(low_res_noise.gba, float3(0.625f, 0.25f, 0.125f));
  low_res_erosion = EE_VOLUMETRIC_CLOUD_Remap(low_res_erosion, height_coverage, 1.0f, 0.0f, 1.0f);
  float low_resolution_density = EE_VOLUMETRIC_CLOUD_Remap(low_shape_density, low_res_erosion, 1.0f, 0.0f, 1.0f);
  density_result.low_resolution_density = low_resolution_density;
  density_result.detail_erosion = low_res_erosion;

  if (low_resolution_density < 0.0001f) {
    return density_result;
  }

  float final_density = low_resolution_density;
  if (use_high_detail != 0) {
    float4 high_res_noise = EE_VOLUMETRIC_CLOUD_SampleDetailErosionNoise(settings, world_position, atmosphere_center,
                                                                       time_seconds, relative_height);
    float high_res_erosion = dot(high_res_noise.rgb, float3(0.625f, 0.25f, 0.125f));
    high_res_erosion = lerp(high_res_erosion, 1.0f - high_res_erosion, EE_VOLUMETRIC_CLOUD_Saturate(relative_height * 10.0f));
    final_density = EE_VOLUMETRIC_CLOUD_Remap(low_resolution_density, high_res_erosion, 1.0f, 0.0f, 1.0f);
    density_result.detail_erosion = high_res_erosion;
  }

  float weather_density = lerp(0.75f, 1.20f, weather.b);
  density_result.final_density =
      EE_VOLUMETRIC_CLOUD_Saturate(final_density * weather_density * max(settings.density, 0.0f));
  return density_result;
}

float EE_VOLUMETRIC_CLOUD_SampleDensity(in VolumetricCloudSettingsGpu settings, in float3 world_position,
                                        in float3 atmosphere_center, in float time_seconds) {
  return EE_VOLUMETRIC_CLOUD_SampleDensityComponents(settings, world_position, atmosphere_center, time_seconds, 1)
      .final_density;
}

VolumetricCloudRayInterval EE_VOLUMETRIC_CLOUD_IntersectLayer(in float3 ray_origin, in float3 ray_direction,
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
    in float3 ray_origin, in float3 ray_direction, in VolumetricCloudSettingsGpu settings, in float max_distance,
    in float3 atmosphere_center) {
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

public float EE_VOLUMETRIC_CLOUD_SurfaceShadowTransmittance(in VolumetricCloudSettingsGpu settings, in float3 surface_position,
                                                     in float3 reference_origin, in float3 sun_direction,
                                                     in float time_seconds) {
  if (settings.enabled == 0 || settings.enable_cloud_shadows == 0 || settings.cloud_shadow_strength <= 0.0f) {
    return 1.0f;
  }
  float3 ray_direction = normalize(sun_direction);
  float3 ray_origin = surface_position + ray_direction * 0.25f;
  float3 atmosphere_center = EE_VOLUMETRIC_CLOUD_AtmosphereCenter(settings, reference_origin);
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
    float3 sample_position = ray_origin + ray_direction * t;
    VolumetricCloudDensitySample density_sample =
        EE_VOLUMETRIC_CLOUD_SampleDensityComponents(settings, sample_position, atmosphere_center, time_seconds, 0);
    optical_depth += density_sample.low_resolution_density * step_length;
    t += step_length;
  }
  return EE_VOLUMETRIC_CLOUD_Saturate(exp(-optical_depth * max(settings.extinction_scale, 0.00001f) * 1.5f));
}

float3 EE_VOLUMETRIC_CLOUD_ConeSample(in int sample_index) {
  if (sample_index == 0) {
    return float3(0.0f, 0.6f, 0.0f);
  }
  if (sample_index == 1) {
    return float3(0.0f, 0.5f, 0.05f);
  }
  if (sample_index == 2) {
    return float3(0.1f, 0.75f, 0.0f);
  }
  if (sample_index == 3) {
    return float3(0.2f, 2.5f, 0.3f);
  }
  if (sample_index == 4) {
    return float3(0.0f, 6.0f, 0.0f);
  }
  return float3(-0.1f, 1.0f, -0.2f);
}

float EE_VOLUMETRIC_CLOUD_LightDensity(in VolumetricCloudSettingsGpu settings, in float3 world_position,
                                       in float3 atmosphere_center, in float3 light_direction, in float time_seconds,
                                       in float primary_step_length) {
  float3 basis_up = abs(light_direction.y) < 0.95f ? float3(0.0f, 1.0f, 0.0f) : float3(1.0f, 0.0f, 0.0f);
  float3 basis_right = normalize(cross(basis_up, light_direction));
  basis_up = normalize(cross(light_direction, basis_right));
  int sample_count = clamp(settings.light_step_count, 1, 6);
  float density_along_light = 0.0f;
  for (int sample_index = 0; sample_index < 6; ++sample_index) {
    if (sample_index >= sample_count) {
      break;
    }
    float3 cone_sample = EE_VOLUMETRIC_CLOUD_ConeSample(sample_index);
    float3 sample_offset = basis_right * cone_sample.x + light_direction * cone_sample.y + basis_up * cone_sample.z;
    float3 sample_position = world_position + 3.0f * primary_step_length * sample_offset;
    density_along_light += EE_VOLUMETRIC_CLOUD_SampleDensity(settings, sample_position, atmosphere_center,
                                                             time_seconds);
  }
  return density_along_light;
}

float EE_VOLUMETRIC_CLOUD_LightTransmittance(in VolumetricCloudSettingsGpu settings, in float3 world_position,
                                             in float3 atmosphere_center, in float3 light_direction,
                                             in float time_seconds, in float primary_step_length,
                                             in float cos_theta) {
  float density_along_light = EE_VOLUMETRIC_CLOUD_LightDensity(settings, world_position, atmosphere_center,
                                                              light_direction, time_seconds, primary_step_length);
  float beers_law = exp(-density_along_light);
  float beers_modulated = max(beers_law, 0.7f * exp(-0.25f * density_along_light));
  return lerp(beers_law, beers_modulated, -cos_theta * 0.5f + 0.5f);
}

public VolumetricCloudMarchResult EE_VOLUMETRIC_CLOUD_March(in VolumetricCloudSettingsGpu settings, in float3 ray_origin,
                                                     in float3 ray_direction, in float max_distance,
                                                     in float scene_distance, in float3 sun_direction,
                                                     in float3 sun_radiance, in float3 ambient_radiance,
                                                     in float time_seconds, in float jitter) {
  VolumetricCloudMarchResult result;
  result.radiance = float3(0.0f);
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

    float3 sample_position = ray_origin + ray_direction * t;
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
        float3 ambient_lighting = ambient_radiance * settings.ambient_lighting_strength * lerp(1.0f, 0.55f, density);
        float3 sun_lighting =
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
  result.transmittance = lerp(1.0f, EE_VOLUMETRIC_CLOUD_Saturate(result.transmittance), horizon_fade);
  result.mean_density = used_steps > 0.0f ? density_sum / used_steps : 0.0f;
  result.mean_base_shape = used_steps > 0.0f ? base_shape_sum / used_steps : 0.0f;
  result.mean_detail_erosion = used_steps > 0.0f ? detail_erosion_sum / used_steps : 0.0f;
  result.mean_weather_coverage = used_steps > 0.0f ? weather_coverage_sum / used_steps : 0.0f;
  return result;
}

public float3 EE_VOLUMETRIC_CLOUD_DebugAccumulation(in VolumetricCloudMarchResult cloud, in int debug_mode) {
  if (debug_mode == 1) {
    return float3(cloud.mean_density);
  }
  if (debug_mode == 4) {
    return float3(cloud.mean_base_shape);
  }
  if (debug_mode == 5) {
    return float3(EE_VOLUMETRIC_CLOUD_Saturate(cloud.mean_detail_erosion * 4.0f));
  }
  if (debug_mode == 6) {
    return float3(cloud.mean_weather_coverage);
  }
  return cloud.radiance;
}
