// Auto-exposure histogram adapted from NVIDIA nvpro_core2 nvshaders tonemapper.slang.
// SPDX-FileCopyrightText: Copyright (c) 2022-2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: Apache-2.0

// @evoengine-dialect native
import EvoEngine.ToneMapping;

static const uint EE_TONEMAP_WORKGROUP_SIZE = 16u;

[[vk::binding(0, 1)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> inColor;

[[vk::binding(0, 2)]]
RWStructuredBuffer<uint> bins;

groupshared uint local_bins[EE_EXPOSURE_HISTOGRAM_SIZE];

float EE_TONE_MAPPING_LUMINANCE(float3 color) {
  return dot(color, float3(0.2126f, 0.7152f, 0.0722f));
}

float EE_TONE_MAPPING_LUMINANCE_EV100(float luminance) {
  return log2(luminance * (100.0f / 12.5f));
}

uint EE_TONE_MAPPING_HISTOGRAM_BUCKET(float3 color) {
  const float input_luminance = EE_TONE_MAPPING_LUMINANCE(color);
  if (input_luminance < 0.00001f) return 0u;
  const float ev_range = EE_TONE_MAPPING_CONSTANTS.ev_max_value - EE_TONE_MAPPING_CONSTANTS.ev_min_value;
  const float ev100 = EE_TONE_MAPPING_LUMINANCE_EV100(input_luminance);
  const float normalized_ev =
      clamp((ev100 - EE_TONE_MAPPING_CONSTANTS.ev_min_value) / ev_range, 0.0f, 1.0f);
  return uint(normalized_ev * float(EE_EXPOSURE_HISTOGRAM_SIZE - 2u) + 1.0f);
}

float EE_TONE_MAPPING_CENTER_METERING_WEIGHT(float2 uv, float aspect) {
  if (EE_TONE_MAPPING_CONSTANTS.enable_center_metering == 0) return 1.0f;
  const float radius = max(EE_TONE_MAPPING_CONSTANTS.center_metering_size, 0.001f);
  const float weight = clamp(length((uv - float2(0.5f)) / float2(1.0f, aspect)) / radius, 0.0f, 1.0f);
  return 1.0f - weight;
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID, uint3 group_thread_id : SV_GroupThreadID) {
  uint local_index = group_thread_id.y * EE_TONEMAP_WORKGROUP_SIZE + group_thread_id.x;
  local_bins[local_index] = 0u;
  GroupMemoryBarrierWithGroupSync();

  int2 pixel = int2(dispatch_thread_id.xy);
  uint width = 0;
  uint height = 0;
  inColor.GetDimensions(width, height);
  int2 dimensions = int2(width, height);
  if (all(pixel < dimensions)) {
    float3 color = max(inColor.Load(pixel).xyz, float3(0.0f));
    float aspect = float(dimensions.x) / max(float(dimensions.y), 1.0f);
    float weight = EE_TONE_MAPPING_CENTER_METERING_WEIGHT((float2(pixel) + float2(0.5f)) / float2(dimensions), aspect);
    InterlockedAdd(local_bins[EE_TONE_MAPPING_HISTOGRAM_BUCKET(color)], uint(weight * 255.0f));
  }

  GroupMemoryBarrierWithGroupSync();
  if (local_bins[local_index] != 0u) {
    InterlockedAdd(bins[local_index], local_bins[local_index]);
  }
}
