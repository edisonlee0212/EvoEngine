import EvoEngine.CameraRayIntegrator;

[[vk::binding(12, 2)]] RWByteAddressBuffer resolved_buffer;
[[vk::binding(22, 2)]] RWByteAddressBuffer fresh_path_buffer;
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
  float3 radiance =
      asfloat(resolved_buffer.Load4((pixel_coordinate.y * width + pixel_coordinate.x) * EE_RESTIR_PT_RESOLVED_BYTES)).xyz;
  if (EE_CAMERA_RESTIR_FRESH_ACCUMULATION_ENABLED()) {
    const float fresh_weight = saturate(float(EE_CAMERA_RESTIR_ACCUMULATED_SAMPLES()) / 1023.0f);
    const float3 fresh_radiance = asfloat(fresh_path_buffer.Load4(
        (pixel_coordinate.y * width + pixel_coordinate.x) * EE_RESTIR_PT_RESOLVED_BYTES)).xyz;
    radiance = fresh_weight >= 1.0f ? fresh_radiance : lerp(radiance, fresh_radiance, fresh_weight);
  }
#ifdef EE_ENABLE_NRD_SIGNALS
  float3 diffuse = nrd_diffuse_signal_image[pixel_coordinate].xyz;
  float3 specular = nrd_specular_signal_image[pixel_coordinate].xyz;
  float3 excluded = nrd_excluded_signal_image[pixel_coordinate].xyz;
  const float3 fresh_radiance = EE_CAMERA_RESOLVE_RESTIR_RADIANCE_PIXEL(pixel_coordinate, radiance);
  const float raw_luminance = dot(radiance, float3(1.0f / 3.0f));
  const float fresh_luminance = dot(fresh_radiance, float3(1.0f / 3.0f));
  const float scale = raw_luminance > 0.0f && isfinite(raw_luminance) ? fresh_luminance / raw_luminance : 0.0f;
  const float2 lobe_hit_distance = nrd_lobe_hit_distance_image[pixel_coordinate];
  float4 diffuse_signal = float4(diffuse * scale, lobe_hit_distance.x);
  float4 specular_signal = float4(specular * scale, lobe_hit_distance.y);
  float4 excluded_signal = float4(excluded * scale, 0.0f);
  EE_CAMERA_ACCUMULATE_RESTIR_NRD_SIGNALS(nrd_accumulation_buffer, pixel_coordinate.y * width + pixel_coordinate.x,
                                           diffuse_signal, specular_signal, excluded_signal);
  nrd_diffuse_signal_image[pixel_coordinate] =
      float4(diffuse_signal.xyz / nrd_diffuse_factor_image[pixel_coordinate].xyz, diffuse_signal.w);
  nrd_specular_signal_image[pixel_coordinate] =
      float4(specular_signal.xyz / nrd_specular_factor_image[pixel_coordinate].xyz, specular_signal.w);
  nrd_excluded_signal_image[pixel_coordinate] = excluded_signal;
#else
  EE_CAMERA_RESOLVE_RESTIR_RADIANCE_PIXEL(pixel_coordinate, radiance);
#endif
}
