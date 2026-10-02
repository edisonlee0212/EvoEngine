#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
#ifndef EE_RESTIR_PT_PROFILE_GENERATED
#  define EE_RESTIR_PT_PROFILE_GENERATED 0
#endif
#ifndef EE_RESTIR_PT_SINGLE_SAMPLE
#  define EE_RESTIR_PT_SINGLE_SAMPLE 0
#endif
#ifdef EE_ENABLE_NRD_SIGNALS
#  define EE_RESTIR_PT_WRITE_NRD_NORMAL true
#else
#  define EE_RESTIR_PT_WRITE_NRD_NORMAL false
#endif
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
[[vk::binding(11, 2)]] RWByteAddressBuffer primary_surface_buffer;
[[vk::binding(22, 2)]] RWByteAddressBuffer initial_buffer;
#ifdef EE_ENABLE_NRD_SIGNALS
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
  float2 lobe_hit_distance;
  float3 diffuse_factor;
  float3 specular_factor;
  EE_CAMERA_RENDER_RESTIR_CANDIDATE_PIXEL<CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>,
                                          EE_GLTF_COMPILED_FEATURE_MASK,
                                          EE_RESTIR_PT_PROFILE_GENERATED != 0,
                                          EE_RESTIR_PT_SINGLE_SAMPLE != 0,
                                          EE_RESTIR_PT_WRITE_NRD_NORMAL>(pixel_coordinate, image_size,
                                                                          candidate_buffer, primary_surface_buffer,
                                                                          lobe_hit_distance, diffuse_factor,
                                                                          specular_factor);
#ifdef EE_ENABLE_NRD_SIGNALS
  nrd_lobe_hit_distance_image[pixel_coordinate] = lobe_hit_distance;
  nrd_diffuse_factor_image[pixel_coordinate] = float4(diffuse_factor, 0.0f);
  nrd_specular_factor_image[pixel_coordinate] = float4(specular_factor, 0.0f);
#endif
  const Camera camera = EE_CAMERA_RESTIR_ACTIVE_CAMERA();
  if (camera.restir_enhanced_mode != 0u && camera.accumulate_samples != 0u &&
      camera.firefly_clamp_threshold > 0.0f) {
    const uint index = pixel_coordinate.y * width + pixel_coordinate.x;
    const RestirPtPathReservoir candidate = EE_RESTIR_PT_LOAD(candidate_buffer, index);
    const float3 initial = EE_RESTIR_PT_RESOLVE_CANDIDATE(candidate);
    initial_buffer.Store4(index * EE_RESTIR_PT_RESOLVED_BYTES,
                          asuint(float4(initial, float(EE_RESTIR_PT_SIGNAL_CLASS(candidate)))));
  }
}
