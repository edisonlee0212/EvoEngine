import EvoEngine.CameraRayIntegrator;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
[[vk::binding(11, 2)]] RWByteAddressBuffer primary_surface_buffer;
[[vk::binding(12, 2)]] RWByteAddressBuffer resolved_buffer;
[[vk::binding(19, 2)]] RWByteAddressBuffer next_history_buffer;
#ifdef EE_ENABLE_NRD_SIGNALS
[[vk::binding(25, 2)]] RWTexture2D<float4> nrd_diffuse_signal_image;
[[vk::binding(26, 2)]] RWTexture2D<float4> nrd_specular_signal_image;
[[vk::binding(27, 2)]] RWTexture2D<float4> nrd_excluded_signal_image;
#endif

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  if (pixel.x >= width || pixel.y >= height) return;
  const uint index = pixel.y * width + pixel.x;
  RestirPtPathReservoir candidate = EE_RESTIR_PT_LOAD(candidate_buffer, index);
  const float3 radiance = EE_RESTIR_PT_RESOLVE_CANDIDATE(candidate);
#ifdef EE_ENABLE_NRD_SIGNALS
  float3 diffuse = 0.0f;
  float3 specular = 0.0f;
  float3 excluded = 0.0f;
  EE_RESTIR_PT_ADD_SIGNAL(candidate, radiance, diffuse, specular, excluded);
#endif
  const RestirPtPrimarySurface surface = EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, index);
  if (surface.material_id == EE_RESTIR_PT_INVALID_INSTANCE || candidate.target_density <= 0.0f)
    candidate = EE_RESTIR_PT_EMPTY_RESERVOIR();
  EE_RESTIR_PT_STORE(next_history_buffer, index, candidate);
  resolved_buffer.Store4(index * EE_RESTIR_PT_RESOLVED_BYTES, asuint(float4(radiance, 0.0f)));
#ifdef EE_ENABLE_NRD_SIGNALS
  nrd_diffuse_signal_image[pixel] = float4(diffuse, 0.0f);
  nrd_specular_signal_image[pixel] = float4(specular, 0.0f);
  nrd_excluded_signal_image[pixel] = float4(excluded, 0.0f);
#endif
}
