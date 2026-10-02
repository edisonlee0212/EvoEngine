#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;

#ifdef EE_ENABLE_NRD_SIGNALS
#define EE_STABLE_NRD_GUIDE true
#else
#define EE_STABLE_NRD_GUIDE false
#endif

#ifdef EE_ENABLE_NRD_SIGNALS
[[vk::binding(11, 2)]] RWByteAddressBuffer primary_surface_buffer;
[[vk::binding(25, 2)]] RWTexture2D<float4> nrd_diffuse_signal_image;
[[vk::binding(26, 2)]] RWTexture2D<float4> nrd_specular_signal_image;
[[vk::binding(27, 2)]] RWTexture2D<float4> nrd_excluded_signal_image;
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
  CameraRayNrdPixel nrd_pixel;
  EE_CAMERA_RENDER_PIXEL<CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>,
                         EE_GLTF_COMPILED_FEATURE_MASK, EE_STABLE_NRD_GUIDE>(pixel_coordinate, image_size, nrd_pixel);
#ifdef EE_ENABLE_NRD_SIGNALS
  const uint pixel_index = pixel_coordinate.y * width + pixel_coordinate.x;
  EE_RESTIR_PT_STORE_PRIMARY_SURFACE(primary_surface_buffer, pixel_index, nrd_pixel.primary_surface);
  if (EE_CAMERA_NRD_NORMAL_OUTPUT_ENABLED())
    ray_normal_image[int2(pixel_coordinate)] = nrd_pixel.normal_roughness;
  nrd_diffuse_factor_image[pixel_coordinate] = float4(nrd_pixel.diffuse_factor, 0.0f);
  nrd_specular_factor_image[pixel_coordinate] = float4(nrd_pixel.specular_factor, 0.0f);
  nrd_diffuse_signal_image[pixel_coordinate] = float4(nrd_pixel.diffuse.xyz / nrd_pixel.diffuse_factor,
                                                     nrd_pixel.diffuse.w);
  nrd_specular_signal_image[pixel_coordinate] = float4(nrd_pixel.specular.xyz / nrd_pixel.specular_factor,
                                                      nrd_pixel.specular.w);
  nrd_excluded_signal_image[pixel_coordinate] = nrd_pixel.excluded;
#endif
}
