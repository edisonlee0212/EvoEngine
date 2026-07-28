// Auto-exposure histogram adapted from NVIDIA nvpro_core2 nvshaders tonemapper.slang.
// SPDX-FileCopyrightText: Copyright (c) 2022-2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: Apache-2.0

#extension GL_ARB_shading_language_include : enable
#extension GL_EXT_nonuniform_qualifier : enable

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

const uint EE_EXPOSURE_HISTOGRAM_SIZE = 256u;
const uint EE_TONEMAP_WORKGROUP_SIZE = 16u;

layout(set = 1, binding = 0, rgba32f) uniform image2D inColor;
layout(set = 2, binding = 0, std430) buffer EE_TONE_MAPPING_HISTOGRAM {
  uint bins[EE_EXPOSURE_HISTOGRAM_SIZE];
};

shared uint local_bins[EE_EXPOSURE_HISTOGRAM_SIZE];

float EE_TONE_MAPPING_LUMINANCE(vec3 color) {
  return dot(color, vec3(0.2126f, 0.7152f, 0.0722f));
}

float EE_TONE_MAPPING_LUMINANCE_EV100(float luminance) {
  return log2(luminance * (100.0f / 12.5f));
}

uint EE_TONE_MAPPING_HISTOGRAM_BUCKET(vec3 color) {
  const float input_luminance = EE_TONE_MAPPING_LUMINANCE(color);
  if (input_luminance < 0.00001f) {
    return 0u;
  }

  const float ev_range = ev_max_value - ev_min_value;
  const float ev100 = EE_TONE_MAPPING_LUMINANCE_EV100(input_luminance);
  const float normalized_ev = clamp((ev100 - ev_min_value) / ev_range, 0.0f, 1.0f);
  return uint(normalized_ev * float(EE_EXPOSURE_HISTOGRAM_SIZE - 2u) + 1.0f);
}

float EE_TONE_MAPPING_CENTER_METERING_WEIGHT(vec2 uv, float aspect) {
  if (enable_center_metering == 0) {
    return 1.0f;
  }
  const float radius = max(center_metering_size, 0.001f);
  const float weight = clamp(length((uv - vec2(0.5f)) / vec2(1.0f, aspect)) / radius, 0.0f, 1.0f);
  return mix(1.0f, 0.0f, weight);
}

[shader("compute")][numthreads(16, 16, 1)] void main(uint3 dispatchThreadId : SV_DispatchThreadID,
                                                     uint3 groupThreadId : SV_GroupThreadID) {
  uint local_index = groupThreadId.y * EE_TONEMAP_WORKGROUP_SIZE + groupThreadId.x;
  local_bins[local_index] = 0u;
  barrier();

  ivec2 pixel = ivec2(dispatchThreadId.xy);
  ivec2 dimensions = imageSize(inColor);
  if (pixel.x < dimensions.x && pixel.y < dimensions.y) {
    vec3 color = max(imageLoad(inColor, pixel).xyz, vec3(0.0f));
    float aspect = float(dimensions.x) / max(float(dimensions.y), 1.0f);
    float weight = EE_TONE_MAPPING_CENTER_METERING_WEIGHT((vec2(pixel) + vec2(0.5f)) / vec2(dimensions), aspect);
    atomicAdd(local_bins[EE_TONE_MAPPING_HISTOGRAM_BUCKET(color)], uint(weight * 255.0f));
  }

  barrier();
  if (local_bins[local_index] != 0u) {
    atomicAdd(bins[local_index], local_bins[local_index]);
  }
}
