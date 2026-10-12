#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;

#ifdef EE_ENABLE_NRD_SIGNALS
[[vk::binding(25, 2)]] RWTexture2D<float4> nrd_diffuse_signal_image;
[[vk::binding(26, 2)]] RWTexture2D<float4> nrd_specular_signal_image;
[[vk::binding(27, 2)]] RWTexture2D<float4> nrd_excluded_signal_image;
#endif

[shader("raygeneration")] void main() {
  const uint2 pixel_coordinate = DispatchRaysIndex().xy;
  const uint2 image_size = DispatchRaysDimensions().xy;
  CameraRayNrdPixel nrd_pixel;
  EE_CAMERA_RENDER_PIXEL<CameraRayTracingTraversalPolicy, EE_GLTF_COMPILED_FEATURE_MASK>(
      pixel_coordinate, image_size, nrd_pixel);
#ifdef EE_ENABLE_NRD_SIGNALS
  nrd_diffuse_signal_image[pixel_coordinate] = nrd_pixel.diffuse;
  nrd_specular_signal_image[pixel_coordinate] = nrd_pixel.specular;
  nrd_excluded_signal_image[pixel_coordinate] = nrd_pixel.excluded;
#endif
}
