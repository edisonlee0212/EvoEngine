// @evoengine-dialect native
__exported import EvoEngine.PerFrame;
__exported import EvoEngine.DDGI;

public interface IDdgiGatherResources {
  float4 loadProbeState(uint volume_index, uint probe_index);
  float4 sampleIrradiance(uint volume_index, float2 uv);
  float4 sampleVisibility(uint volume_index, float2 uv);
}
public struct EeDdgiGatherResult {
  public float3 irradiance;
  public float3 linear_irradiance;
  public float coverage;
  public float confidence;
  public float visibility;
};
public EeDdgiGatherResult EE_DDGI_EMPTY_GATHER() {
  EeDdgiGatherResult result;
  result.irradiance = float3(0.0f);
  result.linear_irradiance = float3(0.0f);
  result.coverage = 0.0f;
  result.confidence = 0.0f;
  result.visibility = 1.0f;
  return result;
}
public float2 EE_DDGI_ATLAS_SIZE(const uint probe_count, const uint tile_resolution, const uint columns) {
  const uint safe_columns = max(columns, 1u);
  const uint rows = (max(probe_count, 1u) + safe_columns - 1u) / safe_columns;
  const uint tile_stride = max(tile_resolution, 1u) + 2u;
  return float2(safe_columns * tile_stride, max(rows, 1u) * tile_stride);
}
public float EE_DDGI_VOLUME_COVERAGE(const DdgiVolumeInfo volume, const float3 position) {
  const uint3 probe_counts = max(uint3(volume.probe_counts.xyz), uint3(1u));
  const float3 probe_coordinate = EE_DDGI_PROBE_COORDINATE(position, volume.first_probe.xyz, volume.probe_step_x.xyz,
                                                           volume.probe_step_y.xyz, volume.probe_step_z.xyz);
  return EE_DDGI_VOLUME_BLEND_WEIGHT(probe_coordinate, probe_counts, volume.probe_step_x.xyz, volume.probe_step_y.xyz,
                                     volume.probe_step_z.xyz);
}
public float EE_DDGI_VOLUME_BOUNDARY_WEIGHT(const DdgiVolumeInfo volume, const float3 position) {
  const uint3 probe_counts = max(uint3(volume.probe_counts.xyz), uint3(1u));
  const float3 probe_coordinate = EE_DDGI_PROBE_COORDINATE(position, volume.first_probe.xyz, volume.probe_step_x.xyz,
                                                           volume.probe_step_y.xyz, volume.probe_step_z.xyz);
  const float3 max_probe_coordinate = float3(probe_counts - uint3(1u));
  if (any(probe_coordinate < float3(0.0f)) || any(probe_coordinate > max_probe_coordinate)) {
    return 1.0f;
  }
  const float3 distance_to_boundary = min(probe_coordinate, max_probe_coordinate - probe_coordinate);
  const float nearest_boundary = min(distance_to_boundary.x, min(distance_to_boundary.y, distance_to_boundary.z));
  return clamp(1.0f - nearest_boundary, 0.0f, 1.0f);
}
public EeDdgiGatherResult EE_DDGI_GATHER_VOLUME_IRRADIANCE<T : IDdgiGatherResources>(T resources, const uint volume_index, const DdgiVolumeInfo volume,
                                                    const float3 normal, const float3 view_direction,
                                                    const float3 position, const bool gather_linear_irradiance) {
  EeDdgiGatherResult result = EE_DDGI_EMPTY_GATHER();
  if (volume.lighting_parameters.x <= 0.0f) {
    return result;
  }
  const uint3 probe_counts = max(uint3(volume.probe_counts.xyz), uint3(1u));
  const uint probe_count = probe_counts.x * probe_counts.y * probe_counts.z;
  const float3 biased_position = EE_DDGI_SURFACE_BIASED_POSITION(
      position, normal, view_direction, volume.volume_parameters.z, volume.volume_parameters.w);
  const float3 volume_probe_coordinate = EE_DDGI_PROBE_COORDINATE(
      position, volume.first_probe.xyz, volume.probe_step_x.xyz, volume.probe_step_y.xyz, volume.probe_step_z.xyz);
  result.coverage = EE_DDGI_VOLUME_BLEND_WEIGHT(volume_probe_coordinate, probe_counts, volume.probe_step_x.xyz,
                                                volume.probe_step_y.xyz, volume.probe_step_z.xyz);
  if (isnan(result.coverage) || isinf(result.coverage) || result.coverage <= 0.0f) {
    result.coverage = 0.0f;
    return result;
  }
  const float3 biased_probe_coordinate =
      EE_DDGI_PROBE_COORDINATE(biased_position, volume.first_probe.xyz, volume.probe_step_x.xyz,
                               volume.probe_step_y.xyz, volume.probe_step_z.xyz);
  const uint irradiance_tile_size = max(volume.atlas_parameters.x, 1u);
  const uint irradiance_atlas_columns = max(volume.atlas_parameters.y, 1u);
  const uint visibility_tile_size = max(volume.atlas_parameters.z, 1u);
  const uint visibility_atlas_columns = max(volume.atlas_parameters.w, 1u);
  const float2 irradiance_atlas_size = EE_DDGI_ATLAS_SIZE(probe_count, irradiance_tile_size, irradiance_atlas_columns);
  const float2 visibility_atlas_size = EE_DDGI_ATLAS_SIZE(probe_count, visibility_tile_size, visibility_atlas_columns);
  const int3 probe_scroll_offset = volume.probe_scroll_and_priority.xyz;
  const float irradiance_gamma = max(volume.probe_counts.w, 1.0f);
  const float visibility_bias = max(volume.volume_parameters.y, 0.0f);
  const float3 max_probe_grid = float3(probe_counts - uint3(1u));
  const float3 base_probe_grid = clamp(floor(biased_probe_coordinate), float3(0.0f), max_probe_grid);
  const float3 base_probe_world_position =
      EE_DDGI_PROBE_WORLD_POSITION(base_probe_grid, volume.first_probe.xyz, volume.probe_step_x.xyz,
                                   volume.probe_step_y.xyz, volume.probe_step_z.xyz);
  const float3 base_probe_to_biased_position = biased_position - base_probe_world_position;
  const float3 probe_fraction =
      clamp(float3(EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position, volume.probe_step_x.xyz),
                   EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position, volume.probe_step_y.xyz),
                   EE_DDGI_AXIS_COORDINATE(base_probe_to_biased_position, volume.probe_step_z.xyz)),
            float3(0.0f), float3(1.0f));
  float3 irradiance_sum = float3(0.0f);
  float3 linear_irradiance_sum = float3(0.0f);
  float radiance_weight_sum = 0.0f;
  float active_probe_weight_sum = 0.0f;
  float valid_probe_weight_sum = 0.0f;
  float visibility_sum = 0.0f;
  for (uint z = 0u; z < 2u; ++z) {
    for (uint y = 0u; y < 2u; ++y) {
      for (uint x = 0u; x < 2u; ++x) {
        const uint3 corner = uint3(x, y, z);
        const float3 corner_weight =
            max(float3(0.001f), lerp(float3(1.0f) - probe_fraction, probe_fraction, float3(corner)));
        const float trilinear_weight = corner_weight.x * corner_weight.y * corner_weight.z;
        const float3 probe_grid = clamp(base_probe_grid + float3(corner), float3(0.0f), max_probe_grid);
        const uint probe_index = EE_DDGI_SCROLL_PROBE_INDEX(uint3(probe_grid), probe_scroll_offset, probe_counts);
        const float4 probe_state = resources.loadProbeState(volume_index, probe_index);
        if (probe_state.w > 0.5f) {
          continue;
        }
        active_probe_weight_sum += trilinear_weight;
        const float3 probe_position =
            EE_DDGI_PROBE_WORLD_POSITION(probe_grid, volume.first_probe.xyz, volume.probe_step_x.xyz,
                                         volume.probe_step_y.xyz, volume.probe_step_z.xyz) +
            probe_state.xyz;
        const float3 surface_to_probe = probe_position - position;
        const float probe_distance = length(surface_to_probe);
        const float3 biased_surface_to_probe = probe_position - biased_position;
        const float biased_probe_distance = length(biased_surface_to_probe);
        const float3 surface_to_probe_direction = probe_distance > 0.001f ? surface_to_probe / probe_distance : normal;
        const float3 biased_surface_to_probe_direction =
            biased_probe_distance > 0.001f ? biased_surface_to_probe / biased_probe_distance : normal;
        const float2 irradiance_atlas_uv = EE_DDGI_ATLAS_UV(probe_index, irradiance_atlas_columns, irradiance_tile_size,
                                                            normal, irradiance_atlas_size);
        const float2 visibility_atlas_uv = EE_DDGI_ATLAS_UV(probe_index, visibility_atlas_columns, visibility_tile_size,
                                                            -biased_surface_to_probe_direction, visibility_atlas_size);
        const float4 irradiance = resources.sampleIrradiance(volume_index, irradiance_atlas_uv);
        const float4 visibility_sample = resources.sampleVisibility(volume_index, visibility_atlas_uv);
        if (any(isnan(irradiance)) || any(isinf(irradiance)) || any(isnan(visibility_sample)) ||
            any(isinf(visibility_sample))) {
          continue;
        }
        const float irradiance_validity = clamp(irradiance.a, 0.0f, 1.0f);
        if (irradiance_validity <= 0.0f) {
          continue;
        }
        const float valid_probe_weight = trilinear_weight * irradiance_validity;
        valid_probe_weight_sum += valid_probe_weight;
        const float probe_visibility =
            EE_DDGI_CHEBYSHEV_VISIBILITY(visibility_sample.rg, biased_probe_distance, visibility_bias);
        const float clamped_probe_visibility =
            isnan(probe_visibility) || isinf(probe_visibility) ? 0.0f : clamp(probe_visibility, 0.0f, 1.0f);
        visibility_sum += clamped_probe_visibility * valid_probe_weight;
        const float wrap_shading = (dot(surface_to_probe_direction, normal) + 1.0f) * 0.5f;
        float visibility_weight = wrap_shading * wrap_shading + 0.2f;
        visibility_weight *= max(0.05f, clamped_probe_visibility);
        visibility_weight = EE_DDGI_CRUSH_LOW_WEIGHT(max(0.000001f, visibility_weight));
        const float sample_weight = valid_probe_weight * visibility_weight;
        const float3 encoded_irradiance = max(irradiance.rgb, float3(0.0f));
        irradiance_sum += pow(encoded_irradiance, float3(irradiance_gamma * 0.5f)) * sample_weight;
        if (gather_linear_irradiance) {
          linear_irradiance_sum += pow(encoded_irradiance, float3(irradiance_gamma)) * sample_weight;
        }
        radiance_weight_sum += sample_weight;
      }
    }
  }
  if (isnan(active_probe_weight_sum) || isinf(active_probe_weight_sum) || active_probe_weight_sum <= 0.0f ||
      isnan(valid_probe_weight_sum) || isinf(valid_probe_weight_sum) || valid_probe_weight_sum <= 0.0f ||
      isnan(radiance_weight_sum) || isinf(radiance_weight_sum) || radiance_weight_sum <= 0.0f) {
    return result;
  }
  float3 decoded_irradiance = irradiance_sum / radiance_weight_sum;
  decoded_irradiance *= decoded_irradiance * (2.0f * EE_DDGI_PI);
  const float3 linear_irradiance = gather_linear_irradiance
                                       ? linear_irradiance_sum / radiance_weight_sum * (2.0f * EE_DDGI_PI)
                                       : float3(0.0f);
  if (any(isnan(decoded_irradiance)) || any(isinf(decoded_irradiance)) || any(isnan(linear_irradiance)) ||
      any(isinf(linear_irradiance))) {
    return result;
  }
  result.irradiance = decoded_irradiance;
  result.linear_irradiance = linear_irradiance;
  result.confidence = clamp(valid_probe_weight_sum / active_probe_weight_sum, 0.0f, 1.0f);
  result.visibility = clamp(visibility_sum / valid_probe_weight_sum, 0.0f, 1.0f);
  return result;
}
public float EE_DDGI_GATHER_WEIGHT(const EeDdgiGatherResult gather_result) {
  const float weight = gather_result.coverage * gather_result.confidence;
  return isnan(weight) || isinf(weight) ? 0.0f : clamp(weight, 0.0f, 1.0f);
}
public float EE_DDGI_GATHER_VISIBILITY(const EeDdgiGatherResult gather_result) {
  const float visibility = gather_result.visibility;
  return isnan(visibility) || isinf(visibility) ? 1.0f : clamp(visibility, 0.0f, 1.0f);
}
public float3 EE_DDGI_PROBE_BLEND_LOSS(const EeDdgiGatherResult gather_result) {
  return max(gather_result.linear_irradiance - gather_result.irradiance, float3(0.0f)) *
         EE_DDGI_GATHER_WEIGHT(gather_result);
}
public float3 EE_DDGI_DIFFUSE_RADIANCE(const EeDdgiGatherResult gather_result, const float3 diffuse_albedo) {
  return diffuse_albedo / EE_DDGI_PI * gather_result.irradiance;
}
public float3 EE_DDGI_WEIGHTED_DIFFUSE(const EeDdgiGatherResult gather_result, const float3 diffuse_albedo) {
  return EE_DDGI_DIFFUSE_RADIANCE(gather_result, diffuse_albedo) * EE_DDGI_GATHER_WEIGHT(gather_result);
}
