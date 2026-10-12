import EvoEngine.CameraRayOutputs;
import EvoEngine.NrdTemporalAa;

struct TemporalAaConstants {
  uint camera_index;
  uint history_valid;
  uint total_samples;
  uint frame_samples;
  uint shader_execution_reordering;
  uint max_directional_light_size;
  uint nrd_reblur;
};
[[vk::push_constant]] ConstantBuffer<TemporalAaConstants> constants;
[[vk::binding(23, 2)]] RWTexture2D<float> view_z_image;
[[vk::binding(24, 2)]] RWTexture2D<float4> motion_image;
[[vk::binding(28, 2)]] RWTexture2D<float4> normal_roughness_image;
[[vk::binding(39, 2)]] RWTexture2D<float4> previous_color_image;
[[vk::binding(40, 2)]] RWTexture2D<uint2> previous_geometry_image;
[[vk::binding(41, 2)]] RWTexture2D<float4> next_color_image;
[[vk::binding(42, 2)]] RWTexture2D<uint2> next_geometry_image;

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 thread : SV_DispatchThreadID) {
  uint width, height;
  result_image.GetDimensions(width, height);
  const int2 pixel = int2(thread.xy);
  const int2 size = int2(width, height);
  if (any(pixel >= size)) return;
  const float depth = abs(view_z_image[pixel]);
  const float4 packed = normal_roughness_image[pixel];
  const float3 current = max(result_image[pixel].xyz, 0.0f);
  float3 color = current;
  const float3 motion = motion_image[pixel].xyz;
  const float2 previous_pixel = float2(pixel) + motion.xy * float2(size);
  if (constants.history_valid != 0u && depth < 1e8f && all(isfinite(motion)) &&
      all(previous_pixel >= 0.0f) && all(previous_pixel <= float2(size - 1))) {
    const int2 base = int2(floor(previous_pixel));
    const float2 fraction = frac(previous_pixel);
    const float expected_depth = depth + motion.z;
    float3 history = float3(0.0f);
    float weight_sum = 0.0f;
    // Validate each bilinear tap before filtering across an occlusion/material boundary.
    for (int y = 0; y < 2; ++y) {
      for (int x = 0; x < 2; ++x) {
        const int2 tap = min(base + int2(x, y), size - 1);
        const float4 old_geometry = EE_NRD_TAA_UNPACK_GEOMETRY(previous_geometry_image[tap]);
        const float4 old_color = previous_color_image[tap];
        if (!EE_NRD_TAA_ACCEPT_HISTORY(packed, expected_depth, old_geometry, old_color)) continue;
        const float weight = (x == 0 ? 1.0f - fraction.x : fraction.x) *
                             (y == 0 ? 1.0f - fraction.y : fraction.y);
        history += old_color.xyz * weight;
        weight_sum += weight;
      }
    }
    if (weight_sum > 0.5f) {
      float3 minimum = current, maximum = current;
      for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
          const int2 tap = clamp(pixel + int2(x, y), int2(0), size - 1);
          const float3 neighbor = max(result_image[tap].xyz, 0.0f);
          minimum = min(minimum, neighbor);
          maximum = max(maximum, neighbor);
        }
      }
      color = lerp(current, clamp(history / weight_sum, minimum, maximum), 0.9f * weight_sum);
    }
  }
  next_color_image[pixel] = float4(clamp(color, 0.0f, 65504.0f), packed.w);
  next_geometry_image[pixel] = EE_NRD_TAA_PACK_GEOMETRY(packed.xyz, depth);
}
