// Tonemapping methods adapted from NVIDIA nvpro_core2 nvshaders tonemap_functions.h.slang.
// SPDX-FileCopyrightText: Copyright (c) 2022-2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: Apache-2.0

// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.ToneMapping;

static const int EE_TONEMAP_FILMIC = 0;
static const int EE_TONEMAP_UNCHARTED2 = 1;
static const int EE_TONEMAP_CLIP = 2;
static const int EE_TONEMAP_ACES = 3;
static const int EE_TONEMAP_AGX = 4;
static const int EE_TONEMAP_KHRONOS_PBR = 5;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

[[vk::binding(0, 1)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> inColor;

[[vk::binding(1, 2)]]
RWStructuredBuffer<float> adapted_luminance;

float3 EE_TONE_MAPPING_TO_SRGB(float3 rgb) {
  float3 low = rgb * 12.92f;
  float3 high = pow(max(rgb, float3(0.0f)), float3(1.0f / 2.4f)) * 1.055f - 0.055f;
  return select(rgb > float3(0.0031308f), high, low);
}

float EE_TONE_MAPPING_BT709_LUMINANCE(float3 color) {
  return dot(color, float3(0.2126f, 0.7152f, 0.0722f));
}

float3 EE_TONE_MAPPING_FILMIC(float3 color) {
  float3 temp = max(float3(0.0f), color - float3(0.004f));
  return (temp * (float3(6.2f) * temp + float3(0.5f))) /
         (temp * (float3(6.2f) * temp + float3(1.7f)) + float3(0.06f));
}

float3 EE_TONE_MAPPING_UNCHARTED2_IMPL(float3 color) {
  const float a = 0.15f;
  const float b = 0.50f;
  const float c = 0.10f;
  const float d = 0.20f;
  const float e = 0.02f;
  const float f = 0.30f;
  return ((color * (a * color + c * b) + d * e) / (color * (a * color + b) + d * f)) - e / f;
}

float3 EE_TONE_MAPPING_UNCHARTED2(float3 color) {
  const float white_point = 11.2f;
  const float exposure_bias = 2.0f;
  color = EE_TONE_MAPPING_UNCHARTED2_IMPL(color * exposure_bias);
  float3 white_scale = float3(1.0f) / EE_TONE_MAPPING_UNCHARTED2_IMPL(float3(white_point));
  return pow(max(color * white_scale, float3(0.0f)), float3(1.0f / 2.2f));
}

float3 EE_TONE_MAPPING_ACES(float3 color) {
  const float3x3 input_matrix =
      float3x3(0.59719f, 0.35458f, 0.04823f, 0.07600f, 0.90834f, 0.01566f, 0.02840f, 0.13383f, 0.83777f);
  color = mul(color, input_matrix);

  float3 a = color * (color + float3(0.0245786f)) - float3(0.000090537f);
  float3 b = color * (float3(0.983729f) * color + float3(0.4329510f)) + float3(0.238081f);
  color = a / b;

  const float3x3 output_matrix =
      float3x3(1.60475f, -0.53108f, -0.07367f, -0.10208f, 1.10813f, -0.00605f, -0.00327f, -0.07276f, 1.07602f);
  return EE_TONE_MAPPING_TO_SRGB(mul(color, output_matrix));
}

float3 EE_TONE_MAPPING_AGX(float3 color) {
  const float3x3 agx_matrix =
      float3x3(0.842479062253094f, 0.0784335999999992f, 0.0792237451477643f, 0.0423282422610123f,
           0.878468636469772f, 0.0791661274605434f, 0.0423756549057051f, 0.0784336f, 0.879142973793104f);
  color = mul(color, agx_matrix);

  const float min_ev = -12.47393f;
  const float max_ev = 4.026069f;
  color = clamp(log2(max(color, float3(1e-6f))), float3(min_ev), float3(max_ev));
  color = (color - float3(min_ev)) / (max_ev - min_ev);

  float3 value = 15.5f * color - 40.14f;
  value = color * value + 31.96f;
  value = color * value - 6.868f;
  value = color * value + 0.4298f;
  value = color * value + 0.1191f;
  value = color * value - 0.0023f;

  const float3x3 agx_matrix_inverse =
      float3x3(1.19687900512017f, -0.0980208811401368f, -0.0990297440797205f, -0.0528968517574562f,
           1.15190312990417f, -0.0989611768448433f, -0.0529716355144438f, -0.0980434501171241f,
           1.15107367264116f);
  return mul(value, agx_matrix_inverse);
}

float3 EE_TONE_MAPPING_KHRONOS_PBR(float3 color) {
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
    color = lerp(color, float3(new_peak), float3(g));
  }
  return EE_TONE_MAPPING_TO_SRGB(color);
}

float3 EE_TONE_MAPPING_ADJUST_SATURATION(float3 color, float target_saturation) {
  float lum = EE_TONE_MAPPING_BT709_LUMINANCE(color);
  return lerp(float3(lum), color, float3(target_saturation));
}

float3 EE_TONE_MAPPING_APPLY(float3 color, float2 pixel, float2 image_size) {
  color *= EE_TONE_MAPPING_CONSTANTS.exposure;
  if (EE_TONE_MAPPING_CONSTANTS.auto_exposure == 1) {
    color *= 0.18f / max(0.001f, adapted_luminance[0]);
  }

  float3 mapped = color;
  if (EE_TONE_MAPPING_CONSTANTS.method == EE_TONEMAP_FILMIC) {
    mapped = EE_TONE_MAPPING_FILMIC(color);
  } else if (EE_TONE_MAPPING_CONSTANTS.method == EE_TONEMAP_UNCHARTED2) {
    mapped = EE_TONE_MAPPING_UNCHARTED2(color);
  } else if (EE_TONE_MAPPING_CONSTANTS.method == EE_TONEMAP_CLIP) {
    mapped = EE_TONE_MAPPING_TO_SRGB(color);
  } else if (EE_TONE_MAPPING_CONSTANTS.method == EE_TONEMAP_ACES) {
    mapped = EE_TONE_MAPPING_ACES(color);
  } else if (EE_TONE_MAPPING_CONSTANTS.method == EE_TONEMAP_AGX) {
    mapped = EE_TONE_MAPPING_AGX(color);
  } else if (EE_TONE_MAPPING_CONSTANTS.method == EE_TONEMAP_KHRONOS_PBR) {
    mapped = EE_TONE_MAPPING_KHRONOS_PBR(color);
  } else {
    mapped = float3(1.0f) - exp(-color);
  }

  mapped = clamp(lerp(float3(0.5f), mapped, float3(EE_TONE_MAPPING_CONSTANTS.contrast)), float3(0.0f), float3(1.0f));
  mapped = pow(max(mapped, float3(0.0f)), float3(1.0f / max(EE_TONE_MAPPING_CONSTANTS.brightness, 0.001f)));
  mapped = EE_TONE_MAPPING_ADJUST_SATURATION(mapped, EE_TONE_MAPPING_CONSTANTS.saturation);

  float2 center_uv = pixel / image_size * 2.0f - 1.0f;
  mapped *= 1.0f - dot(center_uv, center_uv) * EE_TONE_MAPPING_CONSTANTS.vignette;

  if (EE_TONE_MAPPING_CONSTANTS.dither == 1) {
    float noise = frac(dot(pixel, float2(0.245122331f, 0.430159704f)));
    noise = 0.5f - 2.0f * abs(noise - 0.5f);
    float triangular_noise = sign(noise) * (1.0f - sqrt(1.0f - 2.0f * abs(noise)));
    bool3 low_end = float3(0.5f / 255.0f) > mapped;
    bool3 high_end = mapped > float3(1.0f - 0.5f / 255.0f);
    bool3 use_uniform = bool3(low_end.x || high_end.x, low_end.y || high_end.y, low_end.z || high_end.z);
    mapped += select(use_uniform, float3(noise), float3(triangular_noise)) / 255.0f;
  }

  return mapped;
}

void Execute(uint pixel_index, uint resolution_x, uint resolution_y);

[shader("compute")][numthreads(WORKGROUP_SIZE, 1, 1)] void main(uint3 groupId : SV_GroupID,
                                                               uint3 groupThreadId : SV_GroupThreadID) {
  uint resolution_x = uint(EE_CAMERA_RESOLUTION_X(EE_TONE_MAPPING_CONSTANTS.camera_index));
  uint resolution_y = uint(EE_CAMERA_RESOLUTION_Y(EE_TONE_MAPPING_CONSTANTS.camera_index));
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
  float2 pixel = float2(pixel_index % resolution_x, pixel_index / resolution_x);
  int2 tex_coord = int2(pixel);
  float4 output_color = inColor.Load(tex_coord);

  if (EE_TONE_MAPPING_CONSTANTS.is_active == 1) {
    output_color.xyz = EE_TONE_MAPPING_APPLY(max(output_color.xyz, float3(0.0f)), pixel, float2(resolution_x, resolution_y));
  }

  inColor[tex_coord] = output_color;
}
