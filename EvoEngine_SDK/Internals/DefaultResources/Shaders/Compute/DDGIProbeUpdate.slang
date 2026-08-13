// @evoengine-dialect native
import EvoEngine.DDGI;
import EvoEngine.DDGIProbeRayData;

#ifndef EE_DDGI_PROBE_UPDATE_MODE
#  define EE_DDGI_PROBE_UPDATE_MODE 0
#endif

#ifndef EE_DDGI_PROBE_USE_SHARED_RAYS
#  define EE_DDGI_PROBE_USE_SHARED_RAYS 0
#endif

#define EE_DDGI_PROBE_UPDATE_SERIAL 0
#define EE_DDGI_PROBE_UPDATE_IRRADIANCE 1
#define EE_DDGI_PROBE_UPDATE_VISIBILITY 2
#define EE_DDGI_PROBE_UPDATE_GROUP_SIZE 64u
#define EE_DDGI_PROBE_UPDATE_SHARED_RAY_COUNT 256u

struct EeDdgiProbeUpdateConstants {
  uint4 probe_count_ray_count_and_tile_sizes;
  uint4 atlas_columns_fixed_ray_count_and_update_mode;
  uint4 probe_counts_and_rotation;
  float4 update_parameters;
  float4 blend_parameters;
  int4 probe_scroll_offset;
  int4 probe_scroll_delta;
  float4 probe_step_x;
  float4 probe_step_y;
  float4 probe_step_z;
};

[[vk::push_constant]]
ConstantBuffer<EeDdgiProbeUpdateConstants> constants;

[[vk::binding(0, 1)]]
StructuredBuffer<DdgiProbeRayData> EE_DDGI_PROBE_RAY_DATA;

[[vk::binding(1, 1)]]
[vk::image_format("rgba16f")] RWTexture2D<float4> EE_DDGI_IRRADIANCE_ATLAS;

[[vk::binding(2, 1)]]
[vk::image_format("rg16f")] RWTexture2D<float2> EE_DDGI_VISIBILITY_ATLAS;

[[vk::binding(3, 1)]]
RWStructuredBuffer<float4> EE_DDGI_PROBE_METADATA;

[[vk::binding(4, 1)]]
StructuredBuffer<float4> EE_DDGI_PROBE_STATE;

[[vk::binding(5, 1)]]
[vk::image_format("r16f")] RWTexture2D<float> EE_DDGI_PROBE_VARIABILITY_ATLAS;

[[vk::binding(6, 1)]]
StructuredBuffer<DdgiProbeRaySampleInfo> EE_DDGI_PROBE_RAY_SAMPLE_INFO;

#if EE_DDGI_PROBE_USE_SHARED_RAYS
groupshared float4 EE_DDGI_SHARED_PROBE_RAY_DATA[EE_DDGI_PROBE_UPDATE_SHARED_RAY_COUNT];
groupshared float4 EE_DDGI_SHARED_PROBE_RAY_DIRECTIONS[EE_DDGI_PROBE_UPDATE_SHARED_RAY_COUNT];
#endif

float4 EE_DDGI_PROBE_RAY_ROTATION() {
  return float4(constants.probe_step_x.w, constants.probe_step_y.w, constants.probe_step_z.w,
                asfloat(constants.probe_counts_and_rotation.w));
}

uint EE_DDGI_UPDATE_UNIFORM_RAY_COUNT() {
  return (constants.atlas_columns_fixed_ray_count_and_update_mode.z >> 12u) & 0x1fffu;
}

uint EE_DDGI_UPDATE_FIXED_RAY_COUNT() {
  return constants.atlas_columns_fixed_ray_count_and_update_mode.z & 0xfffu;
}

uint EE_DDGI_UPDATE_DISPATCH_GROUP_COUNT_X() {
  return max(uint(constants.probe_scroll_offset.w), 1u);
}

float EE_DDGI_DIRECTIONAL_WEIGHT(float3 ray_direction, float3 texel_direction) {
  return max(dot(normalize(ray_direction), texel_direction), 0.0f);
}

uint2 EE_DDGI_BORDER_COPY_TEXEL(uint2 tile_origin, uint tile_size, uint2 border_texel) {
  uint tile_stride = EE_DDGI_ATLAS_TILE_STRIDE(tile_size);
  uint2 copy_texel = tile_origin;
  bool is_corner = (border_texel.x == 0u || border_texel.x == tile_stride - 1u) &&
                   (border_texel.y == 0u || border_texel.y == tile_stride - 1u);
  bool is_row = border_texel.x > 0u && border_texel.x < tile_stride - 1u;
  if (is_corner) {
    copy_texel += uint2(border_texel.x > 0u ? 1u : tile_size, border_texel.y > 0u ? 1u : tile_size);
  } else if (is_row) {
    copy_texel +=
        uint2((tile_stride - 1u) - border_texel.x, border_texel.y > 0u ? border_texel.y - 1u : border_texel.y + 1u);
  } else {
    copy_texel +=
        uint2(border_texel.x > 0u ? border_texel.x - 1u : border_texel.x + 1u, (tile_stride - 1u) - border_texel.y);
  }
  return copy_texel;
}

float EE_DDGI_MAX_COMPONENT(float3 value) {
  return max(value.x, max(value.y, value.z));
}

float EE_DDGI_LUMINANCE(float3 value) {
  return dot(value, float3(0.2126f, 0.7152f, 0.0722f));
}

struct EeDdgiIrradianceAccumulator {
  float3 irradiance;
  float weight_sum;
  uint backface_count;
  bool valid;
};

void EE_DDGI_INITIALIZE_IRRADIANCE_ACCUMULATOR(out EeDdgiIrradianceAccumulator accumulator) {
  accumulator.irradiance = float3(0.0f);
  accumulator.weight_sum = 0.0f;
  accumulator.backface_count = 0u;
  accumulator.valid = true;
}

void EE_DDGI_ACCUMULATE_IRRADIANCE(inout EeDdgiIrradianceAccumulator accumulator, DdgiProbeRayData ray_data,
                                   float3 ray_direction, float inverse_mixture_pdf, float3 texel_direction,
                                   uint max_backface_count, bool count_backface) {
  if (!accumulator.valid) {
    return;
  }
  if (EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data)) {
    if (count_backface) {
      accumulator.backface_count += 1u;
      if (accumulator.backface_count >= max_backface_count) {
        accumulator.valid = false;
      }
    }
    return;
  }
  float weight = EE_DDGI_DIRECTIONAL_WEIGHT(ray_direction, texel_direction);
  if (weight <= 0.0f) {
    return;
  }
  accumulator.weight_sum += weight;
  accumulator.irradiance += EE_DDGI_PROBE_RAY_RADIANCE(ray_data) * weight * inverse_mixture_pdf;
}

float4 EE_DDGI_FINALIZE_IRRADIANCE(EeDdgiIrradianceAccumulator accumulator, uint blend_ray_count,
                                   bool mixture_weighted) {
  if (!accumulator.valid) {
    return float4(0.0f);
  }
  float3 irradiance;
  if (mixture_weighted) {
    irradiance = accumulator.irradiance * (1.0f / (2.0f * EE_DDGI_PI * max(float(blend_ray_count), 1.0f)));
  } else {
    float epsilon = max(float(blend_ray_count), 1.0f) * 1e-9f;
    irradiance = accumulator.irradiance * (1.0f / (2.0f * max(accumulator.weight_sum, epsilon)));
  }
  float irradiance_gamma = max(constants.update_parameters.z, 1.0f);
  irradiance = pow(min(max(irradiance, float3(0.0f)), float3(64.0f)), float3(1.0f / irradiance_gamma));
  return float4(irradiance, 1.0f);
}

float4 EE_DDGI_DIRECTIONAL_IRRADIANCE(uint ray_offset, uint fixed_ray_count, uint uniform_ray_count,
                                      uint total_ray_count, float3 texel_direction) {
  bool mixture_weighted = total_ray_count > uniform_ray_count;
  uint blend_ray_count = total_ray_count > fixed_ray_count ? total_ray_count - fixed_ray_count : 0u;
  uint uniform_blend_ray_count = uniform_ray_count > fixed_ray_count ? uniform_ray_count - fixed_ray_count : 0u;
  uint max_backface_count =
      max(1u, uint(float(uniform_blend_ray_count) * clamp(constants.blend_parameters.x, 0.0f, 1.0f)));
  EeDdgiIrradianceAccumulator accumulator;
  EE_DDGI_INITIALIZE_IRRADIANCE_ACCUMULATOR(accumulator);
  for (uint ray_index = fixed_ray_count; ray_index < total_ray_count; ++ray_index) {
    DdgiProbeRayData ray_data = EE_DDGI_PROBE_RAY_DATA[ray_offset + ray_index];
    float3 ray_direction;
    float inverse_mixture_pdf = 1.0f;
    if (mixture_weighted) {
      DdgiProbeRaySampleInfo sample_info = EE_DDGI_PROBE_RAY_SAMPLE_INFO[ray_offset + ray_index];
      ray_direction = EE_DDGI_PROBE_RAY_DIRECTION(sample_info);
      inverse_mixture_pdf = EE_DDGI_PROBE_RAY_INVERSE_PDF(sample_info);
    } else {
      ray_direction = EE_DDGI_ROTATED_PROBE_RAY_DIRECTION(ray_index, uniform_ray_count, fixed_ray_count,
                                                          EE_DDGI_PROBE_RAY_ROTATION());
    }
    EE_DDGI_ACCUMULATE_IRRADIANCE(accumulator, ray_data, ray_direction, inverse_mixture_pdf, texel_direction,
                                  max_backface_count, ray_index < uniform_ray_count);
    if (!accumulator.valid) {
      break;
    }
  }
  return EE_DDGI_FINALIZE_IRRADIANCE(accumulator, blend_ray_count, mixture_weighted);
}

float4 EE_DDGI_BLEND_IRRADIANCE_HISTORY(float4 current_irradiance, float4 history_irradiance, float history_weight) {
  if (current_irradiance.a <= 0.0f) {
    return history_irradiance;
  }

  float hysteresis = clamp(history_weight, 0.0f, 1.0f);
  float previous_bright_response_confidence = clamp(history_irradiance.a - 1.0f, 0.0f, 1.0f);
  if (history_irradiance.a <= 0.0f) {
    hysteresis = 0.0f;
  }

  float3 delta = current_irradiance.rgb - history_irradiance.rgb;
  if (EE_DDGI_MAX_COMPONENT(history_irradiance.rgb - current_irradiance.rgb) > constants.blend_parameters.w) {
    hysteresis = max(0.0f, hysteresis - 0.75f);
  }
  bool bright_response = EE_DDGI_LUMINANCE(delta) > constants.blend_parameters.z;
  float response_confidence2 = 0.0f;
  if (bright_response) {
    response_confidence2 = previous_bright_response_confidence * previous_bright_response_confidence;
    hysteresis = lerp(hysteresis, min(hysteresis, 0.5f), response_confidence2);
    delta *= lerp(0.25f, 1.0f, response_confidence2);
  }

  float min_darkening_step = 1.0f / 1024.0f;
  float3 lerp_delta = (1.0f - hysteresis) * delta;
  if (bright_response) {
    float maximum_bright_step = constants.blend_parameters.z * lerp(1.0f, 1.75f, response_confidence2);
    float bright_step_luminance = EE_DDGI_LUMINANCE(max(lerp_delta, float3(0.0f)));
    lerp_delta *= min(1.0f, maximum_bright_step / max(bright_step_luminance, 1e-6f));
  }
  if (EE_DDGI_MAX_COMPONENT(current_irradiance.rgb) < EE_DDGI_MAX_COMPONENT(history_irradiance.rgb)) {
    lerp_delta = min(max(float3(min_darkening_step), abs(lerp_delta)), abs(delta)) * sign(lerp_delta);
  }
  float bright_response_confidence = bright_response ? min(previous_bright_response_confidence + 0.25f, 1.0f)
                                                     : max(previous_bright_response_confidence - 0.0625f, 0.0f);
  return float4(history_irradiance.rgb + lerp_delta, 1.0f + bright_response_confidence);
}

float EE_DDGI_ENERGY_AWARE_VARIABILITY(float luminance_sigma2, float mean_luminance) {
  if (mean_luminance <= (1.0f / 1024.0f)) {
    return 0.0f;
  }
  float coefficient_of_variation = sqrt(max(luminance_sigma2, 0.0f)) / mean_luminance;
  return coefficient_of_variation * min(mean_luminance / 0.25f, 1.0f);
}

float EE_DDGI_PROBE_MAX_VISIBILITY_DISTANCE() {
  return max(length(float3(length(constants.probe_step_x.xyz), length(constants.probe_step_y.xyz),
                           length(constants.probe_step_z.xyz))) *
                 1.5f,
             0.001f);
}

struct EeDdgiVisibilityAccumulator {
  float weight_sum;
  float first_moment;
  float second_moment;
};

void EE_DDGI_INITIALIZE_VISIBILITY_ACCUMULATOR(out EeDdgiVisibilityAccumulator accumulator) {
  accumulator.weight_sum = 0.0f;
  accumulator.first_moment = 0.0f;
  accumulator.second_moment = 0.0f;
}

void EE_DDGI_ACCUMULATE_VISIBILITY(inout EeDdgiVisibilityAccumulator accumulator, DdgiProbeRayData ray_data,
                                   float3 ray_direction, float3 texel_direction, float max_visibility_distance) {
  float weight =
      pow(EE_DDGI_DIRECTIONAL_WEIGHT(ray_direction, texel_direction), max(constants.blend_parameters.y, 0.0f));
  if (weight <= 0.0f) {
    return;
  }
  float distance = min(abs(EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data)), max_visibility_distance);
  accumulator.weight_sum += weight;
  accumulator.first_moment += distance * weight;
  accumulator.second_moment += distance * distance * weight;
}

float2 EE_DDGI_FINALIZE_VISIBILITY(EeDdgiVisibilityAccumulator accumulator, float max_visibility_distance) {
  if (accumulator.weight_sum <= 0.0f) {
    return float2(max_visibility_distance * 0.5f, max_visibility_distance * max_visibility_distance * 0.5f);
  }
  return float2(accumulator.first_moment, accumulator.second_moment) * (1.0f / (2.0f * accumulator.weight_sum));
}

float2 EE_DDGI_DIRECTIONAL_VISIBILITY_MOMENTS(uint ray_offset, uint first_blend_ray_index, uint ray_count,
                                              float3 texel_direction) {
  float max_visibility_distance = EE_DDGI_PROBE_MAX_VISIBILITY_DISTANCE();
  EeDdgiVisibilityAccumulator accumulator;
  EE_DDGI_INITIALIZE_VISIBILITY_ACCUMULATOR(accumulator);
  for (uint ray_index = first_blend_ray_index; ray_index < ray_count; ++ray_index) {
    DdgiProbeRayData ray_data = EE_DDGI_PROBE_RAY_DATA[ray_offset + ray_index];
    float3 ray_direction =
        EE_DDGI_ROTATED_PROBE_RAY_DIRECTION(ray_index, ray_count, first_blend_ray_index, EE_DDGI_PROBE_RAY_ROTATION());
    EE_DDGI_ACCUMULATE_VISIBILITY(accumulator, ray_data, ray_direction, texel_direction, max_visibility_distance);
  }
  return EE_DDGI_FINALIZE_VISIBILITY(accumulator, max_visibility_distance);
}

void EE_DDGI_COPY_IRRADIANCE_BORDER_TEXELS(uint2 tile_origin, uint tile_size) {
  uint tile_stride = EE_DDGI_ATLAS_TILE_STRIDE(tile_size);
  for (uint y = 0u; y < tile_stride; ++y) {
    for (uint x = 0u; x < tile_stride; ++x) {
      if (x > 0u && x < tile_stride - 1u && y > 0u && y < tile_stride - 1u) {
        continue;
      }
      int2 border_texel = int2(tile_origin + uint2(x, y));
      int2 copy_texel = int2(EE_DDGI_BORDER_COPY_TEXEL(tile_origin, tile_size, uint2(x, y)));
      EE_DDGI_IRRADIANCE_ATLAS[border_texel] = EE_DDGI_IRRADIANCE_ATLAS.Load(copy_texel);
    }
  }
}

void EE_DDGI_COPY_VISIBILITY_BORDER_TEXELS(uint2 tile_origin, uint tile_size) {
  uint tile_stride = EE_DDGI_ATLAS_TILE_STRIDE(tile_size);
  for (uint y = 0u; y < tile_stride; ++y) {
    for (uint x = 0u; x < tile_stride; ++x) {
      if (x > 0u && x < tile_stride - 1u && y > 0u && y < tile_stride - 1u) {
        continue;
      }
      int2 border_texel = int2(tile_origin + uint2(x, y));
      int2 copy_texel = int2(EE_DDGI_BORDER_COPY_TEXEL(tile_origin, tile_size, uint2(x, y)));
      EE_DDGI_VISIBILITY_ATLAS[border_texel] = EE_DDGI_VISIBILITY_ATLAS.Load(copy_texel);
    }
  }
}

void EE_DDGI_WRITE_PROBE_METADATA(uint ray_offset, uint physical_probe_index, uint ray_count, float max_ray_distance,
                                  float relocation_distance, float3 relocation_offset, float previous_inactive) {
  float3 irradiance = float3(0.0f);
  float hit_count = 0.0f;
  float frontface_hit_count = 0.0f;
  float backface_hit_count = 0.0f;
  float average_distance = 0.0f;
  for (uint ray_index = 0u; ray_index < ray_count; ++ray_index) {
    DdgiProbeRayData ray_data = EE_DDGI_PROBE_RAY_DATA[ray_offset + ray_index];
    if (EE_DDGI_PROBE_RAY_INACTIVE(ray_data) || EE_DDGI_PROBE_RAY_MISS(ray_data)) {
      continue;
    }
    hit_count += 1.0f;
    bool backface_hit = EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data);
    float hit_distance = abs(EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data));
    if (backface_hit) {
      backface_hit_count += 1.0f;
    } else {
      frontface_hit_count += 1.0f;
      irradiance += EE_DDGI_PROBE_RAY_RADIANCE(ray_data);
    }
    average_distance += hit_distance;
  }
  if (frontface_hit_count > 0.0f) {
    irradiance /= frontface_hit_count;
  }
  average_distance = hit_count > 0.0f ? average_distance / hit_count : max_ray_distance;
  irradiance =
      pow(min(max(irradiance, float3(0.0f)), float3(64.0f)), float3(1.0f / max(constants.update_parameters.z, 1.0f)));
  if (previous_inactive > 0.5f) {
    irradiance = float3(0.0f);
  }
  uint metadata_offset = physical_probe_index * 3u;
  EE_DDGI_PROBE_METADATA[metadata_offset] = float4(irradiance, hit_count / float(ray_count));
  EE_DDGI_PROBE_METADATA[metadata_offset + 1u] =
      float4(clamp(average_distance / max_ray_distance, 0.0f, 1.0f), backface_hit_count / float(ray_count),
             relocation_distance > 0.0f ? clamp(length(relocation_offset) / relocation_distance, 0.0f, 1.0f) : 0.0f,
             1.0f - previous_inactive);
  EE_DDGI_PROBE_METADATA[metadata_offset + 2u] = float4(relocation_offset, 1.0f - previous_inactive);
}

#if EE_DDGI_PROBE_UPDATE_MODE == EE_DDGI_PROBE_UPDATE_SERIAL
[shader("compute")][numthreads(64, 1, 1)] void main(uint3 group_id : SV_GroupID, uint lane_index : SV_GroupIndex) {
  uint group_index = group_id.x + group_id.y * EE_DDGI_UPDATE_DISPATCH_GROUP_COUNT_X();
  uint probe_index = group_index * EE_DDGI_PROBE_UPDATE_GROUP_SIZE + lane_index;
  uint probe_count = max(constants.probe_count_ray_count_and_tile_sizes.x, 1u);
  if (probe_index >= probe_count) {
    return;
  }
  uint3 total_probe_counts = max(constants.probe_counts_and_rotation.xyz, uint3(1u));
  uint probes_per_layer = total_probe_counts.x * total_probe_counts.y;
  uint3 logical_probe_grid =
      uint3(probe_index % total_probe_counts.x, (probe_index / total_probe_counts.x) % total_probe_counts.y,
            probe_index / probes_per_layer);
  uint3 physical_probe_grid =
      EE_DDGI_SCROLL_PROBE_GRID(logical_probe_grid, constants.probe_scroll_offset.xyz, total_probe_counts);
  uint physical_probe_index = EE_DDGI_PROBE_INDEX(physical_probe_grid, total_probe_counts);
  uint ray_count = max(constants.probe_count_ray_count_and_tile_sizes.y, 1u);
  uint uniform_ray_count = min(max(EE_DDGI_UPDATE_UNIFORM_RAY_COUNT(), 1u), ray_count);
  uint irradiance_tile_size = max(constants.probe_count_ray_count_and_tile_sizes.z, 1u);
  uint visibility_tile_size = max(constants.probe_count_ray_count_and_tile_sizes.w, 1u);
  float max_ray_distance = max(constants.update_parameters.x, 0.001f);
  float relocation_distance = max(constants.update_parameters.w, 0.0f);
  uint fixed_ray_count = min(EE_DDGI_UPDATE_FIXED_RAY_COUNT(), uniform_ray_count);
  uint first_blend_ray_index = fixed_ray_count;
  uint irradiance_columns = max(constants.atlas_columns_fixed_ray_count_and_update_mode.x, 1u);
  uint visibility_columns = max(constants.atlas_columns_fixed_ray_count_and_update_mode.y, 1u);
  uint2 irradiance_tile_origin =
      uint2(physical_probe_index % irradiance_columns, physical_probe_index / irradiance_columns) *
      EE_DDGI_ATLAS_TILE_STRIDE(irradiance_tile_size);
  uint2 variability_tile_origin =
      uint2(physical_probe_index % irradiance_columns, physical_probe_index / irradiance_columns) *
      irradiance_tile_size;
  uint2 visibility_tile_origin =
      uint2(physical_probe_index % visibility_columns, physical_probe_index / visibility_columns) *
      EE_DDGI_ATLAS_TILE_STRIDE(visibility_tile_size);
  uint ray_offset = probe_index * ray_count;
  bool update_irradiance = constants.atlas_columns_fixed_ray_count_and_update_mode.w == 1u;
  bool update_visibility = constants.atlas_columns_fixed_ray_count_and_update_mode.w == 2u;
  float4 previous_state = EE_DDGI_PROBE_STATE[physical_probe_index];
  float previous_inactive = clamp(previous_state.w, 0.0f, 1.0f);
  float3 relocation_offset = previous_state.xyz;
  float2 visibility_auxiliary =
      float2(relocation_distance > 0.0f ? clamp(length(relocation_offset) / relocation_distance, 0.0f, 1.0f) : 0.0f,
             1.0f - previous_inactive);

  bool blend_previous_probe_state = previous_inactive <= 0.5f;
  bool newly_exposed_probe =
      EE_DDGI_PROBE_IS_NEWLY_EXPOSED(logical_probe_grid, constants.probe_scroll_delta.xyz, total_probe_counts);
  float history_weight =
      blend_previous_probe_state && !newly_exposed_probe ? clamp(constants.update_parameters.y, 0.0f, 1.0f) : 0.0f;
  if (update_irradiance) {
    EE_DDGI_WRITE_PROBE_METADATA(ray_offset, physical_probe_index, uniform_ray_count, max_ray_distance,
                                 relocation_distance, relocation_offset, previous_inactive);
  }

  if (blend_previous_probe_state) {
    if (update_irradiance) {
      for (uint y = 0u; y < irradiance_tile_size; ++y) {
        for (uint x = 0u; x < irradiance_tile_size; ++x) {
          float3 texel_direction = EE_DDGI_OCTAHEDRAL_DIRECTION(uint2(x, y), irradiance_tile_size);
          float4 directional_irradiance = EE_DDGI_DIRECTIONAL_IRRADIANCE(ray_offset, first_blend_ray_index,
                                                                         uniform_ray_count, ray_count, texel_direction);
          int2 atlas_texel = int2(irradiance_tile_origin + uint2(1u + x, 1u + y));
          int2 variability_texel = int2(variability_tile_origin + uint2(x, y));
          float4 history_irradiance = EE_DDGI_IRRADIANCE_ATLAS.Load(atlas_texel);
          if (directional_irradiance.a <= 0.0f) {
            continue;
          }
          float3 irradiance_sample = directional_irradiance.rgb;
          directional_irradiance =
              EE_DDGI_BLEND_IRRADIANCE_HISTORY(directional_irradiance, history_irradiance, history_weight);
          float3 irradiance_sigma2 =
              (irradiance_sample - history_irradiance.rgb) * (irradiance_sample - directional_irradiance.rgb);
          float luminance_sigma2 = EE_DDGI_LUMINANCE(irradiance_sigma2);
          float mean_luminance = EE_DDGI_LUMINANCE(directional_irradiance.rgb);
          float coefficient_of_variation = EE_DDGI_ENERGY_AWARE_VARIABILITY(luminance_sigma2, mean_luminance);
          EE_DDGI_IRRADIANCE_ATLAS[atlas_texel] = directional_irradiance;
          EE_DDGI_PROBE_VARIABILITY_ATLAS[variability_texel] = coefficient_of_variation;
        }
      }
      DeviceMemoryBarrier();
      EE_DDGI_COPY_IRRADIANCE_BORDER_TEXELS(irradiance_tile_origin, irradiance_tile_size);
    }

    if (update_visibility) {
      for (uint y = 0u; y < visibility_tile_size; ++y) {
        for (uint x = 0u; x < visibility_tile_size; ++x) {
          float3 texel_direction = EE_DDGI_OCTAHEDRAL_DIRECTION(uint2(x, y), visibility_tile_size);
          float2 directional_visibility = EE_DDGI_DIRECTIONAL_VISIBILITY_MOMENTS(ray_offset, first_blend_ray_index,
                                                                                 uniform_ray_count, texel_direction);
          int2 atlas_texel = int2(visibility_tile_origin + uint2(1u + x, 1u + y));
          float2 history_visibility = EE_DDGI_VISIBILITY_ATLAS.Load(atlas_texel);
          float2 visibility_moments = lerp(directional_visibility, history_visibility, history_weight);
          EE_DDGI_VISIBILITY_ATLAS[atlas_texel] = visibility_moments;
        }
      }
      DeviceMemoryBarrier();
      EE_DDGI_COPY_VISIBILITY_BORDER_TEXELS(visibility_tile_origin, visibility_tile_size);
    }
  } else if (update_irradiance) {
    for (uint y = 0u; y < irradiance_tile_size; ++y) {
      for (uint x = 0u; x < irradiance_tile_size; ++x) {
        EE_DDGI_PROBE_VARIABILITY_ATLAS[int2(variability_tile_origin + uint2(x, y))] = 0.0f;
      }
    }
  }
}
#else
uint2 EE_DDGI_PARALLEL_BORDER_TEXEL(uint tile_size, uint border_index) {
  uint tile_stride = EE_DDGI_ATLAS_TILE_STRIDE(tile_size);
  if (border_index < tile_stride) {
    return uint2(border_index, 0u);
  }
  if (border_index < 2u * tile_stride) {
    return uint2(border_index - tile_stride, tile_stride - 1u);
  }
  uint column_index = border_index - 2u * tile_stride;
  if (column_index < tile_size) {
    return uint2(0u, column_index + 1u);
  }
  return uint2(tile_stride - 1u, column_index - tile_size + 1u);
}

void EE_DDGI_COPY_IRRADIANCE_BORDER_TEXELS_PARALLEL(uint2 tile_origin, uint tile_size, uint lane_index) {
  uint border_texel_count = 4u * tile_size + 4u;
  for (uint border_index = lane_index; border_index < border_texel_count;
       border_index += EE_DDGI_PROBE_UPDATE_GROUP_SIZE) {
    uint2 border_texel = EE_DDGI_PARALLEL_BORDER_TEXEL(tile_size, border_index);
    int2 destination = int2(tile_origin + border_texel);
    int2 source = int2(EE_DDGI_BORDER_COPY_TEXEL(tile_origin, tile_size, border_texel));
    EE_DDGI_IRRADIANCE_ATLAS[destination] = EE_DDGI_IRRADIANCE_ATLAS.Load(source);
  }
}

void EE_DDGI_COPY_VISIBILITY_BORDER_TEXELS_PARALLEL(uint2 tile_origin, uint tile_size, uint lane_index) {
  uint border_texel_count = 4u * tile_size + 4u;
  for (uint border_index = lane_index; border_index < border_texel_count;
       border_index += EE_DDGI_PROBE_UPDATE_GROUP_SIZE) {
    uint2 border_texel = EE_DDGI_PARALLEL_BORDER_TEXEL(tile_size, border_index);
    int2 destination = int2(tile_origin + border_texel);
    int2 source = int2(EE_DDGI_BORDER_COPY_TEXEL(tile_origin, tile_size, border_texel));
    EE_DDGI_VISIBILITY_ATLAS[destination] = EE_DDGI_VISIBILITY_ATLAS.Load(source);
  }
}

void EE_DDGI_STORE_IRRADIANCE_TEXEL(uint2 tile_origin, uint2 variability_origin, uint tile_size, uint texel_index,
                                    float4 directional_irradiance, float history_weight) {
  if (directional_irradiance.a <= 0.0f) {
    return;
  }
  uint2 texel = uint2(texel_index % tile_size, texel_index / tile_size);
  int2 atlas_texel = int2(tile_origin + uint2(1u) + texel);
  int2 variability_texel = int2(variability_origin + texel);
  float4 history_irradiance = EE_DDGI_IRRADIANCE_ATLAS.Load(atlas_texel);
  float3 irradiance_sample = directional_irradiance.rgb;
  directional_irradiance = EE_DDGI_BLEND_IRRADIANCE_HISTORY(directional_irradiance, history_irradiance, history_weight);
  float3 irradiance_sigma2 =
      (irradiance_sample - history_irradiance.rgb) * (irradiance_sample - directional_irradiance.rgb);
  float luminance_sigma2 = EE_DDGI_LUMINANCE(irradiance_sigma2);
  float mean_luminance = EE_DDGI_LUMINANCE(directional_irradiance.rgb);
  float coefficient_of_variation = EE_DDGI_ENERGY_AWARE_VARIABILITY(luminance_sigma2, mean_luminance);
  EE_DDGI_IRRADIANCE_ATLAS[atlas_texel] = directional_irradiance;
  EE_DDGI_PROBE_VARIABILITY_ATLAS[variability_texel] = coefficient_of_variation;
}

void EE_DDGI_STORE_VISIBILITY_TEXEL(uint2 tile_origin, uint tile_size, uint texel_index, float2 directional_visibility,
                                    float history_weight, float2 visibility_auxiliary) {
  uint2 texel = uint2(texel_index % tile_size, texel_index / tile_size);
  int2 atlas_texel = int2(tile_origin + uint2(1u) + texel);
  float2 history_visibility = EE_DDGI_VISIBILITY_ATLAS.Load(atlas_texel);
  float2 visibility_moments = lerp(directional_visibility, history_visibility, history_weight);
  EE_DDGI_VISIBILITY_ATLAS[atlas_texel] = visibility_moments;
}

void EE_DDGI_UPDATE_IRRADIANCE_DIRECT(uint ray_offset, uint fixed_ray_count, uint uniform_ray_count,
                                      uint total_ray_count, uint2 tile_origin, uint2 variability_origin, uint tile_size,
                                      float history_weight, uint lane_index) {
  uint texel_count = tile_size * tile_size;
  for (uint texel_index = lane_index; texel_index < texel_count; texel_index += EE_DDGI_PROBE_UPDATE_GROUP_SIZE) {
    uint2 texel = uint2(texel_index % tile_size, texel_index / tile_size);
    float3 texel_direction = EE_DDGI_OCTAHEDRAL_DIRECTION(texel, tile_size);
    float4 directional_irradiance = EE_DDGI_DIRECTIONAL_IRRADIANCE(ray_offset, fixed_ray_count, uniform_ray_count,
                                                                   total_ray_count, texel_direction);
    EE_DDGI_STORE_IRRADIANCE_TEXEL(tile_origin, variability_origin, tile_size, texel_index, directional_irradiance,
                                   history_weight);
  }
}

void EE_DDGI_UPDATE_VISIBILITY_DIRECT(uint ray_offset, uint first_blend_ray_index, uint ray_count, uint2 tile_origin,
                                      uint tile_size, float history_weight, float2 visibility_auxiliary,
                                      uint lane_index) {
  uint texel_count = tile_size * tile_size;
  for (uint texel_index = lane_index; texel_index < texel_count; texel_index += EE_DDGI_PROBE_UPDATE_GROUP_SIZE) {
    uint2 texel = uint2(texel_index % tile_size, texel_index / tile_size);
    float3 texel_direction = EE_DDGI_OCTAHEDRAL_DIRECTION(texel, tile_size);
    float2 directional_visibility =
        EE_DDGI_DIRECTIONAL_VISIBILITY_MOMENTS(ray_offset, first_blend_ray_index, ray_count, texel_direction);
    EE_DDGI_STORE_VISIBILITY_TEXEL(tile_origin, tile_size, texel_index, directional_visibility, history_weight,
                                   visibility_auxiliary);
  }
}

#  if EE_DDGI_PROBE_USE_SHARED_RAYS
void EE_DDGI_CACHE_RAY_CHUNK(uint ray_offset, uint chunk_base, uint chunk_count, uint uniform_ray_count,
                             uint fixed_ray_count, uint lane_index) {
  bool mixture_weighted = constants.probe_count_ray_count_and_tile_sizes.y > uniform_ray_count;
  for (uint chunk_index = lane_index; chunk_index < chunk_count; chunk_index += EE_DDGI_PROBE_UPDATE_GROUP_SIZE) {
    uint ray_index = chunk_base + chunk_index;
    EE_DDGI_SHARED_PROBE_RAY_DATA[chunk_index] =
        EE_DDGI_PROBE_RAY_DATA[ray_offset + ray_index].radiance_and_signed_distance;
    if (mixture_weighted) {
      DdgiProbeRaySampleInfo sample_info = EE_DDGI_PROBE_RAY_SAMPLE_INFO[ray_offset + ray_index];
      EE_DDGI_SHARED_PROBE_RAY_DIRECTIONS[chunk_index] =
          float4(EE_DDGI_PROBE_RAY_DIRECTION(sample_info), EE_DDGI_PROBE_RAY_INVERSE_PDF(sample_info));
    } else {
      EE_DDGI_SHARED_PROBE_RAY_DIRECTIONS[chunk_index] =
          float4(EE_DDGI_ROTATED_PROBE_RAY_DIRECTION(ray_index, uniform_ray_count, fixed_ray_count,
                                                     EE_DDGI_PROBE_RAY_ROTATION()),
                 1.0f);
    }
  }
}

void EE_DDGI_UPDATE_IRRADIANCE_SHARED(uint ray_offset, uint fixed_ray_count, uint uniform_ray_count,
                                      uint total_ray_count, uint2 tile_origin, uint2 variability_origin, uint tile_size,
                                      float history_weight, uint lane_index) {
  uint texel_count = tile_size * tile_size;
  bool mixture_weighted = total_ray_count > uniform_ray_count;
  uint blend_ray_count = total_ray_count > fixed_ray_count ? total_ray_count - fixed_ray_count : 0u;
  uint uniform_blend_ray_count = uniform_ray_count > fixed_ray_count ? uniform_ray_count - fixed_ray_count : 0u;
  uint max_backface_count =
      max(1u, uint(float(uniform_blend_ray_count) * clamp(constants.blend_parameters.x, 0.0f, 1.0f)));
  for (uint batch_base = 0u; batch_base < texel_count; batch_base += EE_DDGI_PROBE_UPDATE_GROUP_SIZE) {
    uint texel_index = batch_base + lane_index;
    bool active_texel = texel_index < texel_count;
    uint2 texel = uint2(texel_index % tile_size, texel_index / tile_size);
    float3 texel_direction = EE_DDGI_OCTAHEDRAL_DIRECTION(texel, tile_size);
    EeDdgiIrradianceAccumulator accumulator;
    EE_DDGI_INITIALIZE_IRRADIANCE_ACCUMULATOR(accumulator);
    for (uint chunk_base = fixed_ray_count; chunk_base < total_ray_count;
         chunk_base += EE_DDGI_PROBE_UPDATE_SHARED_RAY_COUNT) {
      uint chunk_count = min(EE_DDGI_PROBE_UPDATE_SHARED_RAY_COUNT, total_ray_count - chunk_base);
      EE_DDGI_CACHE_RAY_CHUNK(ray_offset, chunk_base, chunk_count, uniform_ray_count, fixed_ray_count, lane_index);
      GroupMemoryBarrierWithGroupSync();
      if (active_texel) {
        for (uint chunk_index = 0u; chunk_index < chunk_count; ++chunk_index) {
          DdgiProbeRayData ray_data;
          ray_data.radiance_and_signed_distance = EE_DDGI_SHARED_PROBE_RAY_DATA[chunk_index];
          EE_DDGI_ACCUMULATE_IRRADIANCE(accumulator, ray_data, EE_DDGI_SHARED_PROBE_RAY_DIRECTIONS[chunk_index].xyz,
                                        EE_DDGI_SHARED_PROBE_RAY_DIRECTIONS[chunk_index].w, texel_direction,
                                        max_backface_count, chunk_base + chunk_index < uniform_ray_count);
        }
      }
      GroupMemoryBarrierWithGroupSync();
    }
    if (active_texel) {
      EE_DDGI_STORE_IRRADIANCE_TEXEL(tile_origin, variability_origin, tile_size, texel_index,
                                     EE_DDGI_FINALIZE_IRRADIANCE(accumulator, blend_ray_count, mixture_weighted),
                                     history_weight);
    }
  }
}

void EE_DDGI_UPDATE_VISIBILITY_SHARED(uint ray_offset, uint first_blend_ray_index, uint ray_count, uint2 tile_origin,
                                      uint tile_size, float history_weight, float2 visibility_auxiliary,
                                      uint lane_index) {
  uint texels_per_lane = 4u;
  uint texel_count = tile_size * tile_size;
  float max_visibility_distance = EE_DDGI_PROBE_MAX_VISIBILITY_DISTANCE();
  for (uint batch_base = 0u; batch_base < texel_count;
       batch_base += EE_DDGI_PROBE_UPDATE_GROUP_SIZE * texels_per_lane) {
    uint texel_indices[4];
    bool active_texels[4];
    float3 texel_directions[4];
    EeDdgiVisibilityAccumulator accumulators[4];
    for (uint texel_slot = 0u; texel_slot < texels_per_lane; ++texel_slot) {
      texel_indices[texel_slot] = batch_base + lane_index + texel_slot * EE_DDGI_PROBE_UPDATE_GROUP_SIZE;
      active_texels[texel_slot] = texel_indices[texel_slot] < texel_count;
      uint2 texel = uint2(texel_indices[texel_slot] % tile_size, texel_indices[texel_slot] / tile_size);
      texel_directions[texel_slot] = EE_DDGI_OCTAHEDRAL_DIRECTION(texel, tile_size);
      EE_DDGI_INITIALIZE_VISIBILITY_ACCUMULATOR(accumulators[texel_slot]);
    }
    for (uint chunk_base = first_blend_ray_index; chunk_base < ray_count;
         chunk_base += EE_DDGI_PROBE_UPDATE_SHARED_RAY_COUNT) {
      uint chunk_count = min(EE_DDGI_PROBE_UPDATE_SHARED_RAY_COUNT, ray_count - chunk_base);
      EE_DDGI_CACHE_RAY_CHUNK(ray_offset, chunk_base, chunk_count, ray_count, first_blend_ray_index, lane_index);
      GroupMemoryBarrierWithGroupSync();
      for (uint chunk_index = 0u; chunk_index < chunk_count; ++chunk_index) {
        DdgiProbeRayData ray_data;
        ray_data.radiance_and_signed_distance = EE_DDGI_SHARED_PROBE_RAY_DATA[chunk_index];
        for (uint texel_slot = 0u; texel_slot < texels_per_lane; ++texel_slot) {
          if (active_texels[texel_slot]) {
            EE_DDGI_ACCUMULATE_VISIBILITY(accumulators[texel_slot], ray_data,
                                          EE_DDGI_SHARED_PROBE_RAY_DIRECTIONS[chunk_index].xyz,
                                          texel_directions[texel_slot], max_visibility_distance);
          }
        }
      }
      GroupMemoryBarrierWithGroupSync();
    }
    for (uint texel_slot = 0u; texel_slot < texels_per_lane; ++texel_slot) {
      if (active_texels[texel_slot]) {
        EE_DDGI_STORE_VISIBILITY_TEXEL(tile_origin, tile_size, texel_indices[texel_slot],
                                       EE_DDGI_FINALIZE_VISIBILITY(accumulators[texel_slot], max_visibility_distance),
                                       history_weight, visibility_auxiliary);
      }
    }
  }
}
#  endif

[shader("compute")][numthreads(64, 1, 1)] void main(uint3 group_id : SV_GroupID, uint lane_index : SV_GroupIndex) {
  uint probe_index = group_id.x + group_id.y * EE_DDGI_UPDATE_DISPATCH_GROUP_COUNT_X();
  uint probe_count = max(constants.probe_count_ray_count_and_tile_sizes.x, 1u);
  if (probe_index >= probe_count) {
    return;
  }
  uint3 total_probe_counts = max(constants.probe_counts_and_rotation.xyz, uint3(1u));
  uint probes_per_layer = total_probe_counts.x * total_probe_counts.y;
  uint3 logical_probe_grid =
      uint3(probe_index % total_probe_counts.x, (probe_index / total_probe_counts.x) % total_probe_counts.y,
            probe_index / probes_per_layer);
  uint3 physical_probe_grid =
      EE_DDGI_SCROLL_PROBE_GRID(logical_probe_grid, constants.probe_scroll_offset.xyz, total_probe_counts);
  uint physical_probe_index = EE_DDGI_PROBE_INDEX(physical_probe_grid, total_probe_counts);
  uint ray_count = max(constants.probe_count_ray_count_and_tile_sizes.y, 1u);
  uint uniform_ray_count = min(max(EE_DDGI_UPDATE_UNIFORM_RAY_COUNT(), 1u), ray_count);
  uint fixed_ray_count = min(EE_DDGI_UPDATE_FIXED_RAY_COUNT(), uniform_ray_count);
  uint first_blend_ray_index = fixed_ray_count;
  uint ray_offset = probe_index * ray_count;
  float4 previous_state = EE_DDGI_PROBE_STATE[physical_probe_index];
  float previous_inactive = clamp(previous_state.w, 0.0f, 1.0f);
  float3 relocation_offset = previous_state.xyz;
  float relocation_distance = max(constants.update_parameters.w, 0.0f);
  float2 visibility_auxiliary =
      float2(relocation_distance > 0.0f ? clamp(length(relocation_offset) / relocation_distance, 0.0f, 1.0f) : 0.0f,
             1.0f - previous_inactive);
  bool blend_previous_probe_state = previous_inactive <= 0.5f;
  bool newly_exposed_probe =
      EE_DDGI_PROBE_IS_NEWLY_EXPOSED(logical_probe_grid, constants.probe_scroll_delta.xyz, total_probe_counts);
  float history_weight =
      blend_previous_probe_state && !newly_exposed_probe ? clamp(constants.update_parameters.y, 0.0f, 1.0f) : 0.0f;

#  if EE_DDGI_PROBE_UPDATE_MODE == EE_DDGI_PROBE_UPDATE_IRRADIANCE
  uint tile_size = max(constants.probe_count_ray_count_and_tile_sizes.z, 1u);
  uint atlas_columns = max(constants.atlas_columns_fixed_ray_count_and_update_mode.x, 1u);
  uint2 tile_origin = uint2(physical_probe_index % atlas_columns, physical_probe_index / atlas_columns) *
                      EE_DDGI_ATLAS_TILE_STRIDE(tile_size);
  uint2 variability_origin =
      uint2(physical_probe_index % atlas_columns, physical_probe_index / atlas_columns) * tile_size;
  if (lane_index == 0u) {
    EE_DDGI_WRITE_PROBE_METADATA(ray_offset, physical_probe_index, uniform_ray_count,
                                 max(constants.update_parameters.x, 0.001f), relocation_distance, relocation_offset,
                                 previous_inactive);
  }
  if (blend_previous_probe_state) {
#    if EE_DDGI_PROBE_USE_SHARED_RAYS
    EE_DDGI_UPDATE_IRRADIANCE_SHARED(ray_offset, first_blend_ray_index, uniform_ray_count, ray_count, tile_origin,
                                     variability_origin, tile_size, history_weight, lane_index);
#    else
    EE_DDGI_UPDATE_IRRADIANCE_DIRECT(ray_offset, first_blend_ray_index, uniform_ray_count, ray_count, tile_origin,
                                     variability_origin, tile_size, history_weight, lane_index);
#    endif
    GroupMemoryBarrierWithGroupSync();
    EE_DDGI_COPY_IRRADIANCE_BORDER_TEXELS_PARALLEL(tile_origin, tile_size, lane_index);
  } else {
    uint texel_count = tile_size * tile_size;
    for (uint texel_index = lane_index; texel_index < texel_count; texel_index += EE_DDGI_PROBE_UPDATE_GROUP_SIZE) {
      uint2 texel = uint2(texel_index % tile_size, texel_index / tile_size);
      EE_DDGI_PROBE_VARIABILITY_ATLAS[int2(variability_origin + texel)] = 0.0f;
    }
  }
#  elif EE_DDGI_PROBE_UPDATE_MODE == EE_DDGI_PROBE_UPDATE_VISIBILITY
  uint tile_size = max(constants.probe_count_ray_count_and_tile_sizes.w, 1u);
  uint atlas_columns = max(constants.atlas_columns_fixed_ray_count_and_update_mode.y, 1u);
  uint2 tile_origin = uint2(physical_probe_index % atlas_columns, physical_probe_index / atlas_columns) *
                      EE_DDGI_ATLAS_TILE_STRIDE(tile_size);
  if (blend_previous_probe_state) {
#    if EE_DDGI_PROBE_USE_SHARED_RAYS
    EE_DDGI_UPDATE_VISIBILITY_SHARED(ray_offset, first_blend_ray_index, uniform_ray_count, tile_origin, tile_size,
                                     history_weight, visibility_auxiliary, lane_index);
#    else
    EE_DDGI_UPDATE_VISIBILITY_DIRECT(ray_offset, first_blend_ray_index, uniform_ray_count, tile_origin, tile_size,
                                     history_weight, visibility_auxiliary, lane_index);
#    endif
    GroupMemoryBarrierWithGroupSync();
    EE_DDGI_COPY_VISIBILITY_BORDER_TEXELS_PARALLEL(tile_origin, tile_size, lane_index);
  }
#  endif
}
#endif
