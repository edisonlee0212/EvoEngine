import EvoEngine.CameraRayIntegrator;
import EvoEngine.Random;

[[vk::binding(12, 2)]] RWByteAddressBuffer resolved_buffer;
[[vk::binding(22, 2)]] RWByteAddressBuffer initial_buffer;
#ifdef EE_ENABLE_NRD_SIGNALS
[[vk::binding(25, 2)]] RWTexture2D<float4> nrd_diffuse_signal_image;
[[vk::binding(26, 2)]] RWTexture2D<float4> nrd_specular_signal_image;
[[vk::binding(27, 2)]] RWTexture2D<float4> nrd_excluded_signal_image;
[[vk::binding(31, 2)]] RWTexture2D<float2> nrd_lobe_hit_distance_image;
[[vk::binding(32, 2)]] RWTexture2D<float4> nrd_diffuse_factor_image;
[[vk::binding(33, 2)]] RWTexture2D<float4> nrd_specular_factor_image;
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
#ifdef EE_ENABLE_NRD_SIGNALS
  float3 diffuse = nrd_diffuse_signal_image[pixel_coordinate].xyz;
  float3 specular = nrd_specular_signal_image[pixel_coordinate].xyz;
  float3 excluded = nrd_excluded_signal_image[pixel_coordinate].xyz;
  uint initial_class = 2u;
  bool use_initial = false;
#endif
  const Camera camera = EE_CAMERA_RESTIR_ACTIVE_CAMERA();
  if (camera.restir_enhanced_mode != 0u && camera.accumulate_samples != 0u &&
      camera.firefly_clamp_threshold > 0.0f) {
    const float4 initial_record = asfloat(initial_buffer.Load4((pixel_coordinate.y * width + pixel_coordinate.x) *
                                                               EE_RESTIR_PT_RESOLVED_BYTES));
    const float3 initial = initial_record.xyz;
#ifdef EE_ENABLE_NRD_SIGNALS
    initial_class = uint(initial_record.w);
#endif
    uint seed = EE_XXHASH32(uint3(pixel_coordinate, EE_CAMERA_RESTIR_FRAME_ID()));
    if (EE_PCG_RANDOM(seed) < 0.5f * saturate((float(EE_CAMERA_RESTIR_ACCUMULATED_SAMPLES()) - 32.0f) / 128.0f)) {
      radiance = initial;
#ifdef EE_ENABLE_NRD_SIGNALS
      use_initial = true;
#endif
    }
    float neighbors[8];
    uint count = 0u;
    for (int dy = -2; dy <= 2; dy += 2) {
      for (int dx = -2; dx <= 2; dx += 2) {
        if (dx == 0 && dy == 0) continue;
        const int2 position = clamp(int2(pixel_coordinate) + int2(dx, dy), int2(0), int2(image_size) - 1);
        const float3 sample = asfloat(resolved_buffer.Load4((uint(position.y) * width + uint(position.x)) *
                                                          EE_RESTIR_PT_RESOLVED_BYTES)).xyz;
        neighbors[count++] = dot(max(sample, 0.0f), float3(0.2126f, 0.7152f, 0.0722f));
      }
    }
    for (uint i = 1u; i < 8u; ++i) {
      const float value = neighbors[i];
      uint j = i;
      while (j > 0u && neighbors[j - 1u] > value) {
        neighbors[j] = neighbors[j - 1u];
        --j;
      }
      neighbors[j] = value;
    }
    const float cap = max(0.1f * camera.firefly_clamp_threshold, 2.0f * (neighbors[3] + neighbors[4]));
    if (dot(max(radiance, 0.0f), float3(0.2126f, 0.7152f, 0.0722f)) > cap) {
      radiance = initial;
#ifdef EE_ENABLE_NRD_SIGNALS
      use_initial = true;
#endif
    }
  }
#ifdef EE_ENABLE_NRD_SIGNALS
  if (use_initial) {
    diffuse = initial_class == 0u ? radiance : float3(0.0f);
    specular = initial_class == 1u ? radiance : float3(0.0f);
    excluded = initial_class == 2u ? radiance : float3(0.0f);
  }
  const float3 fresh_radiance = EE_CAMERA_RESOLVE_RESTIR_RADIANCE_PIXEL(pixel_coordinate, radiance);
  const float raw_luminance = dot(radiance, float3(1.0f / 3.0f));
  const float fresh_luminance = dot(fresh_radiance, float3(1.0f / 3.0f));
  const float scale = raw_luminance > 0.0f && isfinite(raw_luminance) ? fresh_luminance / raw_luminance : 0.0f;
  const float2 lobe_hit_distance = nrd_lobe_hit_distance_image[pixel_coordinate];
  nrd_diffuse_signal_image[pixel_coordinate] =
      float4(diffuse * scale / nrd_diffuse_factor_image[pixel_coordinate].xyz, lobe_hit_distance.x);
  nrd_specular_signal_image[pixel_coordinate] =
      float4(specular * scale / nrd_specular_factor_image[pixel_coordinate].xyz, lobe_hit_distance.y);
  nrd_excluded_signal_image[pixel_coordinate] = float4(excluded * scale, 0.0f);
#else
  EE_CAMERA_RESOLVE_RESTIR_RADIANCE_PIXEL(pixel_coordinate, radiance);
#endif
}
