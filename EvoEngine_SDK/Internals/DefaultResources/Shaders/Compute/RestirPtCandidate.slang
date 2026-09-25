#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
#ifndef EE_RESTIR_PT_PROFILE_GENERATED
#  define EE_RESTIR_PT_PROFILE_GENERATED 0
#endif
#ifndef EE_RESTIR_PT_SINGLE_SAMPLE
#  define EE_RESTIR_PT_SINGLE_SAMPLE 0
#endif
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
[[vk::binding(11, 2)]] RWByteAddressBuffer primary_surface_buffer;

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel_coordinate = dispatch_thread_id.xy;
  const uint2 image_size = uint2(width, height);
  if (any(pixel_coordinate >= image_size)) {
    return;
  }
  EE_CAMERA_RENDER_RESTIR_CANDIDATE_PIXEL<CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>,
                                          EE_GLTF_COMPILED_FEATURE_MASK,
                                          EE_RESTIR_PT_PROFILE_GENERATED != 0,
                                          EE_RESTIR_PT_SINGLE_SAMPLE != 0>(pixel_coordinate, image_size,
                                                                          candidate_buffer, primary_surface_buffer);
}
