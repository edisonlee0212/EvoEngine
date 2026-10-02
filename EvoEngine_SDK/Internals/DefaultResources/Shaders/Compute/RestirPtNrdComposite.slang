import EvoEngine.CameraRayOutputs;

struct NrdCompositePushConstants {
  uint camera_index;
  uint frame_id;
  uint total_samples;
  uint frame_samples;
  uint shader_execution_reordering;
  uint max_directional_light_size;
};
[[vk::push_constant]] ConstantBuffer<NrdCompositePushConstants> composite_constants;

[[vk::binding(23, 2)]] RWTexture2D<float> nrd_view_z_image;
[[vk::binding(27, 2)]] RWTexture2D<float4> nrd_excluded_signal_image;
[[vk::binding(29, 2)]] RWTexture2D<float4> nrd_denoised_diffuse_image;
[[vk::binding(30, 2)]] RWTexture2D<float4> nrd_denoised_specular_image;
[[vk::binding(32, 2)]] RWTexture2D<float4> nrd_diffuse_factor_image;
[[vk::binding(33, 2)]] RWTexture2D<float4> nrd_specular_factor_image;

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  if (pixel.x >= width || pixel.y >= height)
    return;

  // RELAX skips guide misses, leaving its lobe outputs undefined there.
  if (nrd_view_z_image[pixel] == 1e8f) {
    result_image[pixel] = float4(max(radiance_history_image[pixel].xyz, float3(0.0f)), 1.0f);
    return;
  }

  const float3 lobes = nrd_denoised_diffuse_image[pixel].xyz * nrd_diffuse_factor_image[pixel].xyz +
                       nrd_denoised_specular_image[pixel].xyz * nrd_specular_factor_image[pixel].xyz;
  float3 excluded = nrd_excluded_signal_image[pixel].xyz;
  if (composite_constants.total_samples > composite_constants.frame_samples) {
    const float3 weights = float3(0.2126f, 0.7152f, 0.0722f);
    const float remaining = max(dot(max(radiance_history_image[pixel].xyz, float3(0.0f)), weights) -
                                    dot(lobes, weights), 0.0f);
    excluded *= min(1.0f, remaining / max(dot(excluded, weights), 1e-6f));
  }
  float3 result = max(lobes + excluded, float3(0.0f));
  if (composite_constants.total_samples > composite_constants.frame_samples) {
    const float denoised_weight = min(1.0f, sqrt(32.0f / float(composite_constants.total_samples)));
    result = lerp(max(radiance_history_image[pixel].xyz, float3(0.0f)), result, denoised_weight);
  }
  result_image[pixel] = float4(result, 1.0f);
}
