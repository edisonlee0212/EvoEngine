// Auto-exposure reduction adapted from NVIDIA nvpro_core2 nvshaders tonemapper.slang.
// SPDX-FileCopyrightText: Copyright (c) 2022-2025, NVIDIA CORPORATION.
// SPDX-License-Identifier: Apache-2.0

#extension GL_ARB_shading_language_include : enable

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

layout(set = 2, binding = 0, std430) buffer EE_TONE_MAPPING_HISTOGRAM {
  uint bins[EE_EXPOSURE_HISTOGRAM_SIZE];
};

layout(set = 2, binding = 1, std430) buffer EE_TONE_MAPPING_LUMINANCE {
  float adapted_luminance;
};

float EE_TONE_MAPPING_EV100_LUMINANCE(float ev100) {
  return exp2(ev100) / (100.0f / 12.5f);
}

float EE_TONE_MAPPING_HISTOGRAM_OFFSET_TO_NORMALIZED_EV(float offset) {
  return clamp((offset - 1.0f) / float(EE_EXPOSURE_HISTOGRAM_SIZE - 2u), 0.0f, 1.0f);
}

[shader("compute")][numthreads(1, 1, 1)] void main() {
  float total_weight = 0.0f;
  float weighted_index_sum = 0.0f;
  float prefix_counts[EE_EXPOSURE_HISTOGRAM_SIZE];

  for (uint i = 0u; i < EE_EXPOSURE_HISTOGRAM_SIZE; ++i) {
    float count = float(bins[i]);
    total_weight += count;
    weighted_index_sum += count * float(i);
    prefix_counts[i] = total_weight;
  }

  float discard_sample_count = float(bins[0]);
  float non_black_weight = max(total_weight - discard_sample_count, 1.0f);
  float weighted_average = 0.5f;
  if (average_mode == 1) {
    float median = non_black_weight * 0.5f;
    for (uint i = 1u; i < EE_EXPOSURE_HISTOGRAM_SIZE; ++i) {
      if (prefix_counts[i] - discard_sample_count > median) {
        break;
      }
      weighted_average = float(i) + 0.5f;
    }
  } else {
    weighted_average = weighted_index_sum / max(total_weight, 1.0f);
  }

  float normalized_ev = EE_TONE_MAPPING_HISTOGRAM_OFFSET_TO_NORMALIZED_EV(weighted_average);
  float ev_range = ev_max_value - ev_min_value;
  float current_ev100 = normalized_ev * ev_range + ev_min_value;
  float target_luminance = EE_TONE_MAPPING_EV100_LUMINANCE(current_ev100);
  adapted_luminance += (target_luminance - adapted_luminance) * (1.0f - exp2(-auto_exposure_speed));

  for (uint i = 0u; i < EE_EXPOSURE_HISTOGRAM_SIZE; ++i) {
    bins[i] = 0u;
  }
}
