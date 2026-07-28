// Tonemapping methods adapted from NVIDIA nvpro_core2 nvshaders tonemap_functions.h.slang.
// SPDX-FileCopyrightText: Copyright (c) 2022-2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: Apache-2.0

#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_nonuniform_qualifier : enable
#extension GL_EXT_control_flow_attributes : require

#include "PerFrame.slangh"

layout(push_constant) uniform EE_TONE_MAPPING_CONSTANTS {
  int EE_CAMERA_INDEX;
  int method;
  int is_active;
  int auto_exposure;
  int enable_center_metering;
  int average_mode;
  int dither;
  float exposure;
  float brightness;
  float contrast;
  float saturation;
  float vignette;
  float auto_exposure_speed;
  float ev_min_value;
  float ev_max_value;
  float center_metering_size;
};

const int EE_TONEMAP_FILMIC = 0;
const int EE_TONEMAP_UNCHARTED2 = 1;
const int EE_TONEMAP_CLIP = 2;
const int EE_TONEMAP_ACES = 3;
const int EE_TONEMAP_AGX = 4;
const int EE_TONEMAP_KHRONOS_PBR = 5;
const int EE_TONEMAP_EVOENGINE_EXPONENTIAL = 6;

const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

layout(set = 1, binding = 0, rgba32f) uniform image2D inColor;
layout(set = 2, binding = 1, std430) buffer EE_TONE_MAPPING_LUMINANCE {
  float adapted_luminance;
};

vec3 EE_TONE_MAPPING_TO_SRGB(vec3 rgb) {
  vec3 low = rgb * 12.92f;
  vec3 high = pow(max(rgb, vec3(0.0f)), vec3(1.0f / 2.4f)) * 1.055f - 0.055f;
  return mix(low, high, greaterThan(rgb, vec3(0.0031308f)));
}

float EE_TONE_MAPPING_BT709_LUMINANCE(vec3 color) {
  return dot(color, vec3(0.2126f, 0.7152f, 0.0722f));
}

vec3 EE_TONE_MAPPING_FILMIC(vec3 color) {
  vec3 temp = max(vec3(0.0f), color - vec3(0.004f));
  return (temp * (vec3(6.2f) * temp + vec3(0.5f))) /
         (temp * (vec3(6.2f) * temp + vec3(1.7f)) + vec3(0.06f));
}

vec3 EE_TONE_MAPPING_UNCHARTED2_IMPL(vec3 color) {
  const float a = 0.15f;
  const float b = 0.50f;
  const float c = 0.10f;
  const float d = 0.20f;
  const float e = 0.02f;
  const float f = 0.30f;
  return ((color * (a * color + c * b) + d * e) / (color * (a * color + b) + d * f)) - e / f;
}

vec3 EE_TONE_MAPPING_UNCHARTED2(vec3 color) {
  const float white_point = 11.2f;
  const float exposure_bias = 2.0f;
  color = EE_TONE_MAPPING_UNCHARTED2_IMPL(color * exposure_bias);
  vec3 white_scale = vec3(1.0f) / EE_TONE_MAPPING_UNCHARTED2_IMPL(vec3(white_point));
  return pow(max(color * white_scale, vec3(0.0f)), vec3(1.0f / 2.2f));
}

vec3 EE_TONE_MAPPING_ACES(vec3 color) {
  const mat3 input_matrix =
      mat3(0.59719f, 0.35458f, 0.04823f, 0.07600f, 0.90834f, 0.01566f, 0.02840f, 0.13383f, 0.83777f);
  color = input_matrix * color;

  vec3 a = color * (color + vec3(0.0245786f)) - vec3(0.000090537f);
  vec3 b = color * (vec3(0.983729f) * color + vec3(0.4329510f)) + vec3(0.238081f);
  color = a / b;

  const mat3 output_matrix =
      mat3(1.60475f, -0.53108f, -0.07367f, -0.10208f, 1.10813f, -0.00605f, -0.00327f, -0.07276f, 1.07602f);
  return EE_TONE_MAPPING_TO_SRGB(output_matrix * color);
}

vec3 EE_TONE_MAPPING_AGX(vec3 color) {
  const mat3 agx_matrix =
      mat3(0.842479062253094f, 0.0784335999999992f, 0.0792237451477643f, 0.0423282422610123f,
           0.878468636469772f, 0.0791661274605434f, 0.0423756549057051f, 0.0784336f, 0.879142973793104f);
  color = agx_matrix * color;

  const float min_ev = -12.47393f;
  const float max_ev = 4.026069f;
  color = clamp(log2(max(color, vec3(1e-6f))), vec3(min_ev), vec3(max_ev));
  color = (color - vec3(min_ev)) / (max_ev - min_ev);

  vec3 value = 15.5f * color - 40.14f;
  value = color * value + 31.96f;
  value = color * value - 6.868f;
  value = color * value + 0.4298f;
  value = color * value + 0.1191f;
  value = color * value - 0.0023f;

  const mat3 agx_matrix_inverse =
      mat3(1.19687900512017f, -0.0980208811401368f, -0.0990297440797205f, -0.0528968517574562f,
           1.15190312990417f, -0.0989611768448433f, -0.0529716355144438f, -0.0980434501171241f,
           1.15107367264116f);
  return agx_matrix_inverse * value;
}

vec3 EE_TONE_MAPPING_KHRONOS_PBR(vec3 color) {
  const float start_compression = 0.8f - 0.04f;
  const float desaturation = 0.15f;
  float x = min(color.x, min(color.y, color.z));
  float peak = max(color.x, max(color.y, color.z));
  float offset = x < 0.08f ? x * (-6.25f * x + 1.0f) : 0.04f;
  color -= offset;

  if (peak >= start_compression) {
    const float d = 1.0f - start_compression;
    float new_peak = 1.0f - d * d / (peak + d - start_compression);
    color *= new_peak / peak;

    float g = 1.0f - 1.0f / (desaturation * (peak - new_peak) + 1.0f);
    color = mix(color, vec3(new_peak), vec3(g));
  }
  return EE_TONE_MAPPING_TO_SRGB(color);
}

vec3 EE_TONE_MAPPING_ADJUST_SATURATION(vec3 color, float target_saturation) {
  float lum = EE_TONE_MAPPING_BT709_LUMINANCE(color);
  return mix(vec3(lum), color, vec3(target_saturation));
}

vec3 EE_TONE_MAPPING_APPLY(vec3 color, vec2 pixel, vec2 image_size) {
  color *= exposure;
  if (auto_exposure == 1) {
    color *= 0.18f / max(0.001f, adapted_luminance);
  }

  vec3 mapped = color;
  if (method == EE_TONEMAP_FILMIC) {
    mapped = EE_TONE_MAPPING_FILMIC(color);
  } else if (method == EE_TONEMAP_UNCHARTED2) {
    mapped = EE_TONE_MAPPING_UNCHARTED2(color);
  } else if (method == EE_TONEMAP_CLIP) {
    mapped = EE_TONE_MAPPING_TO_SRGB(color);
  } else if (method == EE_TONEMAP_ACES) {
    mapped = EE_TONE_MAPPING_ACES(color);
  } else if (method == EE_TONEMAP_AGX) {
    mapped = EE_TONE_MAPPING_AGX(color);
  } else if (method == EE_TONEMAP_KHRONOS_PBR) {
    mapped = EE_TONE_MAPPING_KHRONOS_PBR(color);
  } else {
    mapped = vec3(1.0f) - exp(-color);
  }

  mapped = clamp(mix(vec3(0.5f), mapped, vec3(contrast)), vec3(0.0f), vec3(1.0f));
  mapped = pow(max(mapped, vec3(0.0f)), vec3(1.0f / max(brightness, 0.001f)));
  mapped = EE_TONE_MAPPING_ADJUST_SATURATION(mapped, saturation);

  vec2 center_uv = pixel / image_size * 2.0f - 1.0f;
  mapped *= 1.0f - dot(center_uv, center_uv) * vignette;

  if (dither == 1) {
    float noise = fract(dot(pixel, vec2(0.245122331f, 0.430159704f)));
    noise = 0.5f - 2.0f * abs(noise - 0.5f);
    float triangular_noise = sign(noise) * (1.0f - sqrt(1.0f - 2.0f * abs(noise)));
    bvec3 low_end = greaterThan(vec3(0.5f / 255.0f), mapped);
    bvec3 high_end = greaterThan(mapped, vec3(1.0f - 0.5f / 255.0f));
    bvec3 use_uniform = bvec3(low_end.x || high_end.x, low_end.y || high_end.y, low_end.z || high_end.z);
    mapped += mix(vec3(triangular_noise), vec3(noise), use_uniform) / 255.0f;
  }

  return mapped;
}

void Execute(uint pixel_index, uint resolution_x, uint resolution_y);

[shader("compute")][numthreads(WORKGROUP_SIZE, 1, 1)] void main(uint3 groupId : SV_GroupID,
                                                               uint3 groupThreadId : SV_GroupThreadID) {
  uint resolution_x = uint(EE_CAMERA_RESOLUTION_X(EE_CAMERA_INDEX));
  uint resolution_y = uint(EE_CAMERA_RESOLUTION_Y(EE_CAMERA_INDEX));
  uint base_id = groupId.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  uint lane_id = groupThreadId.x;

  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = lane_id + i * WORKGROUP_SIZE;
    uint pixel_index = base_id + local_index;

    if (pixel_index >= resolution_x * resolution_y)
      break;
    Execute(pixel_index, resolution_x, resolution_y);
  }
}

void Execute(uint pixel_index, uint resolution_x, uint resolution_y) {
  vec2 pixel = vec2(pixel_index % resolution_x, pixel_index / resolution_x);
  ivec2 tex_coord = ivec2(pixel);
  vec4 output_color = imageLoad(inColor, tex_coord);

  if (is_active == 1) {
    output_color.xyz = EE_TONE_MAPPING_APPLY(max(output_color.xyz, vec3(0.0f)), pixel, vec2(resolution_x, resolution_y));
  }

  imageStore(inColor, tex_coord, output_color);
}
