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

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  if (any(pixel >= uint2(width, height))) return;
  CameraRayNrdPixel nrd_pixel;
  if (!EE_CAMERA_RESOLVE_SUBSURFACE_SPATIAL_PIXEL<CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>,
      EE_GLTF_COMPILED_FEATURE_MASK>(pixel, uint2(width, height), nrd_pixel)) return;
#ifdef EE_ENABLE_NRD_SIGNALS
  nrd_diffuse_signal_image[pixel] += nrd_pixel.diffuse;
  nrd_specular_signal_image[pixel] += nrd_pixel.specular;
  nrd_excluded_signal_image[pixel] += nrd_pixel.excluded;
#endif
}
