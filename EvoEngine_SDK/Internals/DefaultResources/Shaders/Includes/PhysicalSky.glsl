/*
 * Adapted from nvpro_core2 nvshaders sky_io.h.slang and sky_functions.h.slang.
 *
 * Copyright (c) 2022-2025, NVIDIA CORPORATION. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EE_PHYSICAL_SKY_GLSL
#define EE_PHYSICAL_SKY_GLSL

const float EE_PHYSICAL_SKY_PI = 3.14159265359f;

struct EEPhysicalSkyParameters {
  vec3 rgb_unit_conversion;
  float multiplier;
  float haze;
  float red_blue_shift;
  float saturation;
  float horizon_height;
  vec3 ground_color;
  float horizon_blur;
  vec3 night_color;
  float sun_disk_intensity;
  vec3 sun_direction;
  float sun_disk_scale;
  float sun_glow_intensity;
  int y_is_up;
};

struct EEPhysicalSkySamplingResult {
  vec3 direction;
  float pdf;
  vec3 radiance;
};

EEPhysicalSkyParameters EE_PHYSICAL_SKY_DEFAULT_PARAMETERS() {
  EEPhysicalSkyParameters params;
  params.rgb_unit_conversion = vec3(1.0f / 80000.0f);
  params.multiplier = 0.1f;
  params.haze = 0.1f;
  params.red_blue_shift = 0.1f;
  params.saturation = 1.0f;
  params.horizon_height = 0.0f;
  params.ground_color = vec3(0.4f);
  params.horizon_blur = 0.3f;
  params.night_color = vec3(0.0f, 0.0f, 0.01f);
  params.sun_disk_intensity = 1.0f;
  params.sun_direction = vec3(-1.23413404e-08f, 0.707106829f, 0.707106709f);
  params.sun_disk_scale = 1.0f;
  params.sun_glow_intensity = 1.0f;
  params.y_is_up = 1;
  return params;
}

float EE_PHYSICAL_SKY_LUMINANCE(const vec3 rgb) {
  return 0.2126f * rgb.x + 0.7152f * rgb.y + 0.0722f * rgb.z;
}

vec3 EE_PHYSICAL_SKY_LOCAL_COORDS_TO_DIR(const vec3 main_vec, const float x, const float y, const float z) {
  const vec3 u = normalize(abs(main_vec.x) < abs(main_vec.y) ? vec3(0.0f, -main_vec.z, main_vec.y)
                                                             : vec3(main_vec.z, 0.0f, -main_vec.x));
  const vec3 v = cross(main_vec, u);
  return x * u + y * v + z * main_vec;
}

vec2 EE_PHYSICAL_SKY_SQUARE_TO_DISK(const float in_x, const float in_y) {
  const float local_x = 2.0f * in_x - 1.0f;
  const float local_y = 2.0f * in_y - 1.0f;
  if (local_x == 0.0f && local_y == 0.0f) {
    return vec2(0.0f);
  }

  float r;
  float phi;
  if (local_x > -local_y) {
    if (local_x > local_y) {
      r = local_x;
      phi = (EE_PHYSICAL_SKY_PI / 4.0f) * (1.0f + local_y / local_x);
    } else {
      r = local_y;
      phi = (EE_PHYSICAL_SKY_PI / 4.0f) * (3.0f - local_x / local_y);
    }
  } else {
    if (local_x < local_y) {
      r = -local_x;
      phi = (EE_PHYSICAL_SKY_PI / 4.0f) * (5.0f + local_y / local_x);
    } else {
      r = -local_y;
      phi = (EE_PHYSICAL_SKY_PI / 4.0f) * (7.0f - local_x / local_y);
    }
  }
  return vec2(r, phi);
}

vec3 EE_PHYSICAL_SKY_REFLECTION_DIR_DIFFUSE(const vec3 normal, const vec2 sample_value) {
  const vec2 r_phi = EE_PHYSICAL_SKY_SQUARE_TO_DISK(sample_value.x, sample_value.y);
  const float x = r_phi.x * cos(r_phi.y);
  const float y = r_phi.x * sin(r_phi.y);
  const float z = sqrt(max(0.0f, 1.0f - x * x - y * y));
  return EE_PHYSICAL_SKY_LOCAL_COORDS_TO_DIR(normal, x, y, z);
}

vec3 EE_PHYSICAL_SKY_CALC_SUN_COLOR(const vec3 sun_dir, const float turbidity) {
  if (sun_dir.z <= 0.0f) {
    return vec3(0.0f);
  }

  const vec3 ko = vec3(12.0f, 8.5f, 0.9f);
  const vec3 wavelength = vec3(0.610f, 0.550f, 0.470f);
  const vec3 sol_rad = vec3(1.0f, 0.992f, 0.911f) * (127500.0f / 0.9878f);
  const float m = 1.0f / (sun_dir.z + 0.15f * pow(93.885f - degrees(acos(sun_dir.z)), -1.253f));
  const float beta = 0.04608f * turbidity - 0.04586f;
  const vec3 ta = exp(-m * beta * pow(wavelength, vec3(-1.3f)));
  const vec3 to = exp(-m * ko * 0.0035f);
  const vec3 tr = exp(-m * 0.008735f * pow(wavelength, vec3(-4.08f)));
  return tr * ta * to * sol_rad;
}

vec3 EE_PHYSICAL_SKY_COLOR_XYZ(const vec3 in_dir, const vec3 in_sun_pos, const float turbidity,
                               const float luminance) {
  vec3 xyz;
  float a;
  float b;
  float c;
  float d;
  float e;
  float cos_gamma = dot(in_sun_pos, in_dir);
  if (cos_gamma > 1.0f) {
    cos_gamma = 2.0f - cos_gamma;
  }
  const float gamma = acos(cos_gamma);
  const float cos_theta = in_dir.z;
  const float cos_theta_sun = in_sun_pos.z;
  const float theta_sun = acos(cos_theta_sun);
  const float t2 = turbidity * turbidity;
  const float ts2 = theta_sun * theta_sun;
  const float ts3 = ts2 * theta_sun;
  const float zenith_x = ((+0.001650f * ts3 - 0.003742f * ts2 + 0.002088f * theta_sun) * t2 +
                          (-0.029028f * ts3 + 0.063773f * ts2 - 0.032020f * theta_sun + 0.003948f) * turbidity +
                          (+0.116936f * ts3 - 0.211960f * ts2 + 0.060523f * theta_sun + 0.258852f));
  const float zenith_y = ((+0.002759f * ts3 - 0.006105f * ts2 + 0.003162f * theta_sun) * t2 +
                          (-0.042149f * ts3 + 0.089701f * ts2 - 0.041536f * theta_sun + 0.005158f) * turbidity +
                          (+0.153467f * ts3 - 0.267568f * ts2 + 0.066698f * theta_sun + 0.266881f));
  xyz.y = luminance;

  const float inv_cos_theta = 1.0f / max(cos_theta, 0.001f);
  a = -0.019257f * turbidity - (0.29f - sqrt(cos_theta_sun) * 0.09f);
  b = -0.066513f * turbidity + 0.000818f;
  c = -0.000417f * turbidity + 0.212479f;
  d = -0.064097f * turbidity - 0.898875f;
  e = -0.003251f * turbidity + 0.045178f;

  float exp_b_inv_cos_theta = exp(b * inv_cos_theta);
  float exp_b = exp(b);
  float exp_d_gamma = exp(d * gamma);
  float exp_d_theta_sun = exp(d * theta_sun);
  float x = (((1.0f + a * exp_b_inv_cos_theta) * (1.0f + c * exp_d_gamma + e * cos_gamma * cos_gamma)) /
             ((1.0f + a * exp_b) * (1.0f + c * exp_d_theta_sun + e * cos_theta_sun * cos_theta_sun)));

  a = -0.016698f * turbidity - 0.260787f;
  b = -0.094958f * turbidity + 0.009213f;
  c = -0.007928f * turbidity + 0.210230f;
  d = -0.044050f * turbidity - 1.653694f;
  e = -0.010922f * turbidity + 0.052919f;

  exp_b_inv_cos_theta = exp(b * inv_cos_theta);
  exp_b = exp(b);
  exp_d_gamma = exp(d * gamma);
  exp_d_theta_sun = exp(d * theta_sun);
  float y = (((1.0f + a * exp_b_inv_cos_theta) * (1.0f + c * exp_d_gamma + e * cos_gamma * cos_gamma)) /
             ((1.0f + a * exp_b) * (1.0f + c * exp_d_theta_sun + e * cos_theta_sun * cos_theta_sun)));

  x = zenith_x * x;
  y = zenith_y * y;
  xyz.x = (x / y) * xyz.y;
  xyz.z = ((1.0f - x - y) / y) * xyz.y;
  return xyz;
}

float EE_PHYSICAL_SKY_LUMINANCE_DISTRIBUTION(const vec3 in_dir, const vec3 in_sun_pos, const float turbidity) {
  const float cos_gamma = clamp(dot(in_sun_pos, in_dir), 0.0f, 1.0f);
  const float gamma = acos(cos_gamma);
  const float cos_theta = in_dir.z;
  const float cos_theta_sun = in_sun_pos.z;
  const float theta_sun = acos(cos_theta_sun);

  const float a = 0.178721f * turbidity - 1.463037f;
  const float b = -0.355402f * turbidity + 0.427494f;
  const float c = -0.022669f * turbidity + 5.325056f;
  const float d = 0.120647f * turbidity - 2.577052f;
  const float e = -0.066967f * turbidity + 0.370275f;

  const float inv_cos_theta = 1.0f / max(cos_theta, 0.001f);
  const float exp_b_inv_cos_theta = exp(b * inv_cos_theta);
  const float exp_b = exp(b);
  const float exp_d_gamma = exp(d * gamma);
  const float exp_d_theta_sun = exp(d * theta_sun);

  return (((1.0f + a * exp_b_inv_cos_theta) * (1.0f + c * exp_d_gamma + e * cos_gamma * cos_gamma)) /
          ((1.0f + a * exp_b) * (1.0f + c * exp_d_theta_sun + e * cos_theta_sun * cos_theta_sun)));
}

vec3 EE_PHYSICAL_SKY_CALC_SKY_COLOR(const vec3 sun_dir, const vec3 in_dir, const float turbidity) {
  const float theta_sun = acos(sun_dir.z);
  const float chi = (4.0f / 9.0f - turbidity / 120.0f) * (EE_PHYSICAL_SKY_PI - 2.0f * theta_sun);
  float luminance = 1000.0f * ((4.0453f * turbidity - 4.9710f) * tan(chi) - 0.2155f * turbidity + 2.4192f);
  luminance *= EE_PHYSICAL_SKY_LUMINANCE_DISTRIBUTION(in_dir, sun_dir, turbidity);

  const vec3 xyz = EE_PHYSICAL_SKY_COLOR_XYZ(in_dir, sun_dir, turbidity, luminance);
  const vec3 env_color = vec3(3.241f * xyz.x - 1.537f * xyz.y - 0.499f * xyz.z,
                             -0.969f * xyz.x + 1.876f * xyz.y + 0.042f * xyz.z,
                              0.056f * xyz.x - 0.204f * xyz.y + 1.057f * xyz.z);
  return env_color * EE_PHYSICAL_SKY_PI;
}

vec3 EE_PHYSICAL_SKY_CALC_SKY_IRRADIANCE(const vec3 sun_dir, const float haze) {
  vec3 color_sum = vec3(0.0f);
  const vec3 normal = vec3(0.0f, 0.0f, 1.0f);

  for (float u = 0.125f; u < 1.0f; u += 0.25f) {
    for (float v = 0.125f; v < 1.0f; v += 0.25f) {
      const vec3 diff = EE_PHYSICAL_SKY_REFLECTION_DIR_DIFFUSE(normal, vec2(u, v));
      color_sum += EE_PHYSICAL_SKY_CALC_SKY_COLOR(sun_dir, diff, haze);
    }
  }
  return color_sum / 16.0f;
}

float EE_PHYSICAL_SKY_TWEAK_SATURATION(const float saturation, const float haze) {
  if (saturation > 1.0f) {
    return 1.0f;
  }

  const float low_saturation = saturation * saturation * saturation;
  float local_haze = clamp((haze - 2.0f) / 15.0f, 0.0f, 1.0f);
  local_haze *= local_haze * local_haze;
  return mix(saturation, low_saturation, local_haze);
}

vec3 EE_PHYSICAL_SKY_TWEAK_VECTOR(const vec3 dir, const int y_is_up, const float horizon_height) {
  vec3 out_dir = dir;
  if (y_is_up == 1) {
    out_dir = vec3(dir.x, dir.z, dir.y);
  }
  if (horizon_height != 0.0f) {
    out_dir.z -= horizon_height;
    out_dir = normalize(out_dir);
  }
  return out_dir;
}

vec3 EE_PHYSICAL_SKY_INTERNAL_TO_WORLD(const EEPhysicalSkyParameters params, const vec3 dir) {
  return params.y_is_up == 1 ? vec3(dir.x, dir.z, dir.y) : dir;
}

float EE_PHYSICAL_SKY_WORLD_ELEVATION(const EEPhysicalSkyParameters params, const vec3 dir) {
  return params.y_is_up == 1 ? dir.y : dir.z;
}

vec3 EE_PHYSICAL_SKY_TWEAK_COLOR(const vec3 tint, const float saturation, const float redness) {
  const float intensity = EE_PHYSICAL_SKY_LUMINANCE(tint);
  vec3 out_tint = saturation <= 0.0f ? vec3(intensity) : mix(vec3(intensity), tint, saturation);
  out_tint *= vec3(1.0f + redness, 1.0f, 1.0f - redness);
  return max(out_tint, vec3(0.0f));
}

vec2 EE_PHYSICAL_SKY_CALC_PHYSICAL_SCALE(const float sun_disk_scale, const float sun_glow_intensity,
                                         const float sun_disk_intensity) {
  const float sun_angular_radius = 0.00465f;
  const float sun_disk_radius = sun_angular_radius * sun_disk_scale;
  const float sun_glow_radius = sun_disk_radius * 10.0f;
  const float sun_glow_radius_squared = sun_glow_radius * sun_glow_radius;
  const float sun_glow_radius_cubed = sun_glow_radius_squared * sun_glow_radius;

  const float glow_func_integral =
      sun_glow_intensity * ((4.0f * EE_PHYSICAL_SKY_PI) -
                            (24.0f * EE_PHYSICAL_SKY_PI) / sun_glow_radius_squared +
                            (24.0f * EE_PHYSICAL_SKY_PI) * sin(sun_glow_radius) / sun_glow_radius_cubed);
  float target_sun_disk_integral = sun_disk_intensity * EE_PHYSICAL_SKY_PI;
  float sky_sun_glow_scale = 1.0f;
  const float max_glow_integral = 0.5f * target_sun_disk_integral;
  if (glow_func_integral > max_glow_integral) {
    sky_sun_glow_scale *= max_glow_integral / glow_func_integral;
    target_sun_disk_integral -= max_glow_integral;
  } else {
    target_sun_disk_integral -= glow_func_integral;
  }

  const float cos_sun_disk_radius = cos(sun_disk_radius);
  const float sun_disk_area = 2.0f * EE_PHYSICAL_SKY_PI * (1.0f - cos_sun_disk_radius);
  const float target_sun_disk_intensity = target_sun_disk_integral / sun_disk_area;
  const float actual_sun_disk_integral = sun_disk_area;
  const float actual_sun_disk_intensity = sun_disk_intensity * 100.0f * actual_sun_disk_integral / sun_disk_area;
  return vec2(target_sun_disk_intensity == 0.0f ? 0.0f : target_sun_disk_intensity / actual_sun_disk_intensity,
              sky_sun_glow_scale);
}

float EE_PHYSICAL_SKY_NIGHT_BRIGHTNESS_ADJUSTMENT(const vec3 sun_dir) {
  const float limit = 0.3090169943749474f;
  if (sun_dir.z <= -limit) {
    return 0.0f;
  }
  float factor = (sun_dir.z + limit) / limit;
  factor *= factor;
  factor *= factor;
  return factor;
}

vec3 EE_PHYSICAL_SKY_EVALUATE(const EEPhysicalSkyParameters params, const vec3 in_direction) {
  if (params.multiplier <= 0.0f) {
    return vec3(0.0f);
  }

  float factor = 1.0f;
  float night_factor = 1.0f;
  vec3 out_color = vec3(0.0f);
  const vec3 rgb_scale = params.rgb_unit_conversion * params.multiplier;
  const float height_adjusted = (params.horizon_height + params.horizon_blur) / 10.0f;
  vec3 dir = EE_PHYSICAL_SKY_TWEAK_VECTOR(in_direction, params.y_is_up, height_adjusted);
  const float local_haze = max(2.0f, 2.0f + params.haze);
  const float local_saturation = EE_PHYSICAL_SKY_TWEAK_SATURATION(params.saturation, local_haze);

  const float downness = dir.z;
  vec3 real_dir = dir;
  if (dir.z < 0.001f) {
    dir.z = 0.001f;
    dir = normalize(dir);
  }

  vec3 sun_dir = params.sun_direction;
  sun_dir = EE_PHYSICAL_SKY_TWEAK_VECTOR(sun_dir, params.y_is_up, height_adjusted);
  vec3 real_sun_dir = sun_dir;
  if (sun_dir.z < 0.001f) {
    factor = EE_PHYSICAL_SKY_NIGHT_BRIGHTNESS_ADJUSTMENT(sun_dir);
    sun_dir.z = 0.001f;
    sun_dir = normalize(sun_dir);
  }

  vec3 tint = factor > 0.0f ? EE_PHYSICAL_SKY_CALC_SKY_COLOR(sun_dir, dir, local_haze) * factor : vec3(0.0f);
  const vec3 data_sun_color = EE_PHYSICAL_SKY_CALC_SUN_COLOR(sun_dir, downness > 0.0f ? local_haze : 2.0f);

  if (params.sun_disk_intensity > 0.0f && params.sun_disk_scale > 0.0f) {
    const float sun_angle = acos(clamp(dot(real_dir, real_sun_dir), -1.0f, 1.0f));
    const float glow_radius = 0.00465f * params.sun_disk_scale * 10.0f;
    if (sun_angle < glow_radius) {
      const vec2 scales =
          EE_PHYSICAL_SKY_CALC_PHYSICAL_SCALE(params.sun_disk_scale, params.sun_glow_intensity,
                                             params.sun_disk_intensity);
      const float center_proximity = 1.0f - sun_angle / glow_radius;
      const float center_proximity_squared = center_proximity * center_proximity;
      const float center_proximity_cubed = center_proximity_squared * center_proximity;
      const float glow_factor = center_proximity_cubed * 2.0f * params.sun_glow_intensity * scales.y;
      const float disk_factor =
          smoothstep(0.85f, 0.95f + (local_haze / 500.0f), center_proximity) * 100.0f *
          params.sun_disk_intensity * scales.x;
      tint += data_sun_color * (glow_factor + disk_factor);
    }
  }
  out_color = tint * rgb_scale;

  if (downness <= 0.0f) {
    vec3 irradiance = EE_PHYSICAL_SKY_CALC_SKY_IRRADIANCE(sun_dir, 2.0f);
    vec3 down_color = params.ground_color * (irradiance + data_sun_color * sun_dir.z) * rgb_scale;
    down_color *= factor;
    const float horizon_blur = params.horizon_blur / 10.0f;
    if (horizon_blur > 0.0f) {
      const float dness = smoothstep(0.0f, 1.0f, -downness / horizon_blur);
      out_color = mix(out_color, down_color, dness);
      night_factor = 1.0f - dness;
    } else {
      out_color = down_color;
      night_factor = 0.0f;
    }
  }

  vec3 result = EE_PHYSICAL_SKY_TWEAK_COLOR(out_color, local_saturation, params.red_blue_shift) * EE_PHYSICAL_SKY_PI;
  if (night_factor > 0.0f) {
    const vec3 night = params.night_color * night_factor;
    result = max(result, night);
  }
  return result;
}

vec3 EE_PHYSICAL_SKY_SAMPLE_SPHERICAL_CAP(const float z_min, const vec2 sample_value) {
  const float z = mix(1.0f, z_min, sample_value.y);
  const float r = sqrt(max(0.0f, 1.0f - z * z));
  const float phi = 2.0f * EE_PHYSICAL_SKY_PI * sample_value.x;
  return vec3(r * cos(phi), r * sin(phi), z);
}

float EE_PHYSICAL_SKY_SUN_PROBABILITY(const EEPhysicalSkyParameters params) {
  const float sun_elevation = EE_PHYSICAL_SKY_WORLD_ELEVATION(params, params.sun_direction);
  return params.sun_disk_scale > 1e-5f ? clamp(params.sun_disk_intensity * sun_elevation * 0.5f + 0.5f, 0.1f, 0.9f)
                                       : 0.0f;
}

float EE_PHYSICAL_SKY_PDF(const EEPhysicalSkyParameters params, const vec3 in_direction) {
  const float sun_angular_radius = 0.00465f * params.sun_disk_scale;
  const float sky_pdf = EE_PHYSICAL_SKY_WORLD_ELEVATION(params, in_direction) >= 0.0f
                            ? 1.0f / (2.0f * EE_PHYSICAL_SKY_PI)
                            : 0.0f;
  const float sun_sample_angular_radius = 1.5f * sun_angular_radius;
  const float cos_sun_sample_angular_radius = cos(sun_sample_angular_radius);
  const float sun_sample_solid_angle =
      sun_sample_angular_radius < 0.001f ? EE_PHYSICAL_SKY_PI * sun_sample_angular_radius * sun_sample_angular_radius
                                         : 2.0f * EE_PHYSICAL_SKY_PI *
                                               (1.0f - cos_sun_sample_angular_radius);
  const float sun_pdf = dot(in_direction, params.sun_direction) >= cos_sun_sample_angular_radius
                            ? 1.0f / sun_sample_solid_angle
                            : 0.0f;
  return mix(sky_pdf, sun_pdf, EE_PHYSICAL_SKY_SUN_PROBABILITY(params));
}

EEPhysicalSkySamplingResult EE_PHYSICAL_SKY_SAMPLE(const EEPhysicalSkyParameters params, vec2 random_sample) {
  EEPhysicalSkySamplingResult result;
  const float sun_probability = EE_PHYSICAL_SKY_SUN_PROBABILITY(params);
  float z_min = 0.0f;
  const bool sample_sun = random_sample.x < sun_probability;
  if (sample_sun) {
    random_sample.x = random_sample.x / sun_probability;
    const float sun_sample_angular_radius = 1.5f * 0.00465f * params.sun_disk_scale;
    z_min = cos(sun_sample_angular_radius);
  } else {
    random_sample.x = (random_sample.x - sun_probability) / (1.0f - sun_probability);
  }

  result.direction = EE_PHYSICAL_SKY_SAMPLE_SPHERICAL_CAP(z_min, random_sample);
  if (sample_sun) {
    vec3 up = vec3(0.0f, 0.0f, 1.0f);
    const vec3 right = normalize(cross(up, params.sun_direction));
    up = cross(params.sun_direction, right);
    result.direction = result.direction.x * right + result.direction.y * up + result.direction.z * params.sun_direction;
  } else {
    result.direction = EE_PHYSICAL_SKY_INTERNAL_TO_WORLD(params, result.direction);
  }

  result.radiance = EE_PHYSICAL_SKY_EVALUATE(params, result.direction);
  result.pdf = EE_PHYSICAL_SKY_PDF(params, result.direction);
  return result;
}

#endif
