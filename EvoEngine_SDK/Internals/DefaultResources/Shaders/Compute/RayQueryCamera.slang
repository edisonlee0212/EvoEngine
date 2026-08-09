// @evoengine-dialect native
#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel_coordinate = dispatch_thread_id.xy;
  const uint2 image_size = uint2(width, height);
  if (any(pixel_coordinate >= image_size)) {
    return;
  }
  EE_CAMERA_RENDER_PIXEL<CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>,
                         EE_GLTF_COMPILED_FEATURE_MASK>(pixel_coordinate, image_size);
}
