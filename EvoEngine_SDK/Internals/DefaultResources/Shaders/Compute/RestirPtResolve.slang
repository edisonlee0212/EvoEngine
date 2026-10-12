import EvoEngine.CameraRayIntegrator;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
#ifdef EE_ENABLE_NRD_SIGNALS
[[vk::binding(25, 2)]] RWTexture2D<float4> nrd_diffuse_signal_image;
[[vk::binding(26, 2)]] RWTexture2D<float4> nrd_specular_signal_image;
[[vk::binding(27, 2)]] RWTexture2D<float4> nrd_excluded_signal_image;
[[vk::binding(31, 2)]] RWTexture2D<float2> nrd_lobe_hit_distance_image;
[[vk::binding(32, 2)]] RWTexture2D<float4> nrd_diffuse_factor_image;
[[vk::binding(33, 2)]] RWTexture2D<float4> nrd_specular_factor_image;
[[vk::binding(34, 2)]] RWByteAddressBuffer nrd_accumulation_buffer;
#endif

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel_coordinate = dispatch_thread_id.xy;
  const uint2 image_size = uint2(width, height);
  if (any(pixel_coordinate >= image_size)) {
    return;
  }
#ifdef EE_ENABLE_NRD_SIGNALS
  const RestirPtPathReservoir candidate =
      EE_RESTIR_PT_LOAD(candidate_buffer, pixel_coordinate.y * width + pixel_coordinate.x);
  const float3 fresh_radiance =
      EE_CAMERA_RESOLVE_RESTIR_RADIANCE_PIXEL(pixel_coordinate, EE_RESTIR_PT_RESOLVE_CANDIDATE(candidate));
  float3 diffuse = float3(0.0f);
  float3 specular = float3(0.0f);
  float3 excluded = float3(0.0f);
  EE_RESTIR_PT_ADD_SIGNAL(candidate, fresh_radiance, diffuse, specular, excluded);
  const float2 lobe_hit_distance = nrd_lobe_hit_distance_image[pixel_coordinate];
  float4 diffuse_signal = float4(diffuse, lobe_hit_distance.x);
  float4 specular_signal = float4(specular, lobe_hit_distance.y);
  float4 excluded_signal = float4(excluded, 0.0f);
  EE_CAMERA_ACCUMULATE_RESTIR_NRD_SIGNALS(nrd_accumulation_buffer, pixel_coordinate.y * width + pixel_coordinate.x,
                                           diffuse_signal, specular_signal, excluded_signal);
  nrd_diffuse_signal_image[pixel_coordinate] =
      float4(diffuse_signal.xyz / nrd_diffuse_factor_image[pixel_coordinate].xyz, diffuse_signal.w);
  nrd_specular_signal_image[pixel_coordinate] =
      float4(specular_signal.xyz / nrd_specular_factor_image[pixel_coordinate].xyz, specular_signal.w);
  nrd_excluded_signal_image[pixel_coordinate] = excluded_signal;
#else
  EE_CAMERA_RESOLVE_RESTIR_CANDIDATE_PIXEL(pixel_coordinate, image_size, candidate_buffer);
#endif
}
