import EvoEngine.CameraRayOutputs;

struct NrdCompositePushConstants {
  uint camera_index;
  uint frame_id;
  uint total_samples;
  uint frame_samples;
  uint shader_execution_reordering;
  uint max_directional_light_size;
  uint nrd_reblur;
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

  float3 diffuse = nrd_denoised_diffuse_image[pixel].xyz;
  float3 specular = nrd_denoised_specular_image[pixel].xyz;
  if (composite_constants.nrd_reblur != 0u) {
    const float diffuse_t = diffuse.x - diffuse.z;
    const float specular_t = specular.x - specular.z;
    diffuse = max(float3(diffuse_t + diffuse.y, diffuse.x + diffuse.z, diffuse_t - diffuse.y), 0.0f);
    specular = max(float3(specular_t + specular.y, specular.x + specular.z, specular_t - specular.y), 0.0f);
  }
  const float3 lobes = diffuse * nrd_diffuse_factor_image[pixel].xyz +
                       specular * nrd_specular_factor_image[pixel].xyz;
  float3 excluded = nrd_excluded_signal_image[pixel].xyz;
  if (composite_constants.total_samples > composite_constants.frame_samples) {
    const float3 weights = float3(0.2126f, 0.7152f, 0.0722f);
    const float remaining = max(dot(max(radiance_history_image[pixel].xyz, float3(0.0f)), weights) -
                                    dot(lobes, weights), 0.0f);
    excluded *= min(1.0f, remaining / max(dot(excluded, weights), 1e-6f));
  }
  result_image[pixel] = float4(max(lobes + excluded, float3(0.0f)), 1.0f);
}
