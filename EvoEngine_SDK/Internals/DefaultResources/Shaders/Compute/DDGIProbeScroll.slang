// @evoengine-dialect native
import EvoEngine.DDGI;

struct EeDdgiProbeScrollConstants {
  uint4 probe_counts_and_irradiance_tile_size;
  uint4 atlas_columns_visibility_tile_and_probe_count;
  int4 probe_scroll_offset;
  int4 probe_scroll_delta;
};

[[vk::push_constant]]
ConstantBuffer<EeDdgiProbeScrollConstants> constants;

[[vk::binding(1, 0)]]
[vk::image_format("rgba16f")]
RWTexture2D<float4> EE_DDGI_IRRADIANCE_ATLAS;

[[vk::binding(2, 0)]]
[vk::image_format("rg16f")]
RWTexture2D<float2> EE_DDGI_VISIBILITY_ATLAS;

[[vk::binding(3, 0)]]
RWStructuredBuffer<float4> EE_DDGI_PROBE_METADATA;

[[vk::binding(4, 0)]]
RWStructuredBuffer<float4> EE_DDGI_PROBE_STATE;

[[vk::binding(5, 0)]]
[vk::image_format("r16f")]
RWTexture2D<float> EE_DDGI_PROBE_VARIABILITY_ATLAS;

void EE_DDGI_CLEAR_IRRADIANCE_TILE(uint2 tile_origin, uint tile_size) {
  uint tile_stride = EE_DDGI_ATLAS_TILE_STRIDE(tile_size);
  for (uint y = 0u; y < tile_stride; ++y) {
    for (uint x = 0u; x < tile_stride; ++x) {
      EE_DDGI_IRRADIANCE_ATLAS[int2(tile_origin + uint2(x, y))] = float4(0.0f);
    }
  }
}

void EE_DDGI_CLEAR_VISIBILITY_TILE(uint2 tile_origin, uint tile_size) {
  uint tile_stride = EE_DDGI_ATLAS_TILE_STRIDE(tile_size);
  for (uint y = 0u; y < tile_stride; ++y) {
    for (uint x = 0u; x < tile_stride; ++x) {
      EE_DDGI_VISIBILITY_ATLAS[int2(tile_origin + uint2(x, y))] = float2(1.0f, 0.0f);
    }
  }
}

void EE_DDGI_CLEAR_VARIABILITY_TILE(uint2 tile_origin, uint tile_size) {
  for (uint y = 0u; y < tile_size; ++y) {
    for (uint x = 0u; x < tile_size; ++x) {
      EE_DDGI_PROBE_VARIABILITY_ATLAS[int2(tile_origin + uint2(x, y))] = 0.0f;
    }
  }
}

[shader("compute")]
[numthreads(64, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint logical_probe_index = dispatch_thread_id.x;
  uint3 probe_counts = max(constants.probe_counts_and_irradiance_tile_size.xyz, uint3(1u));
  uint probe_count = max(constants.atlas_columns_visibility_tile_and_probe_count.w, 1u);
  if (logical_probe_index >= probe_count) {
    return;
  }

  uint probes_per_layer = probe_counts.x * probe_counts.y;
  uint3 logical_probe_grid =
      uint3(logical_probe_index % probe_counts.x, (logical_probe_index / probe_counts.x) % probe_counts.y,
            logical_probe_index / probes_per_layer);
  if (!EE_DDGI_PROBE_IS_NEWLY_EXPOSED(logical_probe_grid, constants.probe_scroll_delta.xyz, probe_counts)) {
    return;
  }

  uint3 physical_probe_grid = EE_DDGI_SCROLL_PROBE_GRID(logical_probe_grid, constants.probe_scroll_offset.xyz,
                                                        probe_counts);
  uint physical_probe_index = EE_DDGI_PROBE_INDEX(physical_probe_grid, probe_counts);
  uint irradiance_tile_size = max(constants.probe_counts_and_irradiance_tile_size.w, 1u);
  uint irradiance_columns = max(constants.atlas_columns_visibility_tile_and_probe_count.x, 1u);
  uint visibility_columns = max(constants.atlas_columns_visibility_tile_and_probe_count.y, 1u);
  uint visibility_tile_size = max(constants.atlas_columns_visibility_tile_and_probe_count.z, 1u);
  uint2 irradiance_tile_origin = uint2(physical_probe_index % irradiance_columns,
                                       physical_probe_index / irradiance_columns) *
                                  EE_DDGI_ATLAS_TILE_STRIDE(irradiance_tile_size);
  uint2 variability_tile_origin = uint2(physical_probe_index % irradiance_columns,
                                        physical_probe_index / irradiance_columns) *
                                   irradiance_tile_size;
  uint2 visibility_tile_origin = uint2(physical_probe_index % visibility_columns,
                                       physical_probe_index / visibility_columns) *
                                  EE_DDGI_ATLAS_TILE_STRIDE(visibility_tile_size);

  EE_DDGI_PROBE_STATE[physical_probe_index] = float4(0.0f);
  uint metadata_offset = physical_probe_index * 3u;
  EE_DDGI_PROBE_METADATA[metadata_offset] = float4(0.0f);
  EE_DDGI_PROBE_METADATA[metadata_offset + 1u] = float4(1.0f, 0.0f, 0.0f, 1.0f);
  EE_DDGI_PROBE_METADATA[metadata_offset + 2u] = float4(0.0f);
  EE_DDGI_CLEAR_IRRADIANCE_TILE(irradiance_tile_origin, irradiance_tile_size);
  EE_DDGI_CLEAR_VISIBILITY_TILE(visibility_tile_origin, visibility_tile_size);
  EE_DDGI_CLEAR_VARIABILITY_TILE(variability_tile_origin, irradiance_tile_size);
}
