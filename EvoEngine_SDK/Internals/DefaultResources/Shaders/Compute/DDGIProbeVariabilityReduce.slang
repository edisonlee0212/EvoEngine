// @evoengine-dialect native
struct EeDdgiVariabilityConstants {
  uint4 input_output_extent;
  uint4 atlas_parameters;
};

[[vk::push_constant]]
ConstantBuffer<EeDdgiVariabilityConstants> constants;

[[vk::binding(0, 0)]]
[vk::image_format("r16f")]
RWTexture2D<float> EE_DDGI_PROBE_VARIABILITY_ATLAS;

[[vk::binding(1, 0)]]
StructuredBuffer<float4> EE_DDGI_PROBE_STATE;

[[vk::binding(2, 0)]]
[vk::image_format("rg32f")]
RWTexture2D<float2> EE_DDGI_VARIABILITY_REDUCTION_OUTPUT;

[shader("compute")]
[numthreads(8, 8, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 output_coord = dispatch_thread_id.xy;
  uint2 output_extent = max(constants.input_output_extent.zw, uint2(1u));
  if (any(output_coord >= output_extent)) {
    return;
  }

  uint2 input_extent = max(constants.input_output_extent.xy, uint2(1u));
  uint tile_size = max(constants.atlas_parameters.x, 1u);
  uint atlas_columns = max(constants.atlas_parameters.y, 1u);
  uint probe_count = max(constants.atlas_parameters.z, 1u);
  uint2 base_coord = output_coord * 16u;
  float variability_sum = 0.0f;
  float weight_sum = 0.0f;
  for (uint y = 0u; y < 16u; ++y) {
    for (uint x = 0u; x < 16u; ++x) {
      uint2 input_coord = base_coord + uint2(x, y);
      if (any(input_coord >= input_extent)) {
        continue;
      }
      uint2 tile_coord = input_coord / tile_size;
      uint probe_index = tile_coord.x + tile_coord.y * atlas_columns;
      if (probe_index >= probe_count || EE_DDGI_PROBE_STATE[probe_index].w > 0.5f) {
        continue;
      }
      variability_sum += EE_DDGI_PROBE_VARIABILITY_ATLAS.Load(int2(input_coord));
      weight_sum += 1.0f;
    }
  }
  float average = weight_sum > 0.0f ? variability_sum / weight_sum : 0.0f;
  float total_possible_samples = 16.0f * 16.0f;
  float normalized_weight = weight_sum / total_possible_samples;
  EE_DDGI_VARIABILITY_REDUCTION_OUTPUT[int2(output_coord)] = float2(average, normalized_weight);
}
