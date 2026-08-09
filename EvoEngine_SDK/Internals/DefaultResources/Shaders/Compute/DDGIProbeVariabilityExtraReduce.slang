// @evoengine-dialect native
struct EeDdgiVariabilityExtraReduceConstants {
  uint4 input_output_extent;
};

[[vk::push_constant]]
ConstantBuffer<EeDdgiVariabilityExtraReduceConstants> constants;

[[vk::binding(0, 0)]]
[vk::image_format("rg32f")]
RWTexture2D<float2> EE_DDGI_VARIABILITY_REDUCTION_INPUT;

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
  uint2 base_coord = output_coord * 16u;
  float weighted_sum = 0.0f;
  float weight_sum = 0.0f;
  for (uint y = 0u; y < 16u; ++y) {
    for (uint x = 0u; x < 16u; ++x) {
      uint2 input_coord = base_coord + uint2(x, y);
      if (any(input_coord >= input_extent)) {
        continue;
      }
      float2 sample_value = EE_DDGI_VARIABILITY_REDUCTION_INPUT.Load(int2(input_coord));
      weighted_sum += sample_value.r * sample_value.g;
      weight_sum += sample_value.g;
    }
  }
  float average = weight_sum > 0.0f ? weighted_sum / weight_sum : 0.0f;
  float total_possible_weight = 16.0f * 16.0f;
  float normalized_weight = weight_sum / total_possible_weight;
  EE_DDGI_VARIABILITY_REDUCTION_OUTPUT[int2(output_coord)] = float2(average, normalized_weight);
}
