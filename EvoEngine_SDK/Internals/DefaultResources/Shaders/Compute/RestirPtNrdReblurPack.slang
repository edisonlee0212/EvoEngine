import EvoEngine.CameraRayIntegrator;
import EvoEngine.Nrd;

[[vk::binding(32, 2)]] RWTexture2D<float4> nrd_diffuse_factor_image;
[[vk::binding(33, 2)]] RWTexture2D<float4> nrd_specular_factor_image;

[[vk::binding(31, 2)]] RWTexture2D<float2> nrd_lobe_hit_distance_image;
[[vk::binding(23, 2)]] RWTexture2D<float> nrd_view_z_image;
[[vk::binding(25, 2)]] RWTexture2D<float4> nrd_diffuse_signal_image;
[[vk::binding(26, 2)]] RWTexture2D<float4> nrd_specular_signal_image;
[[vk::binding(28, 2)]] RWTexture2D<float4> nrd_normal_roughness_image;

float4 PackReblurSignal(float4 signal, float view_z, float roughness) {
  const float norm_hit_distance = isfinite(signal.w) ? REBLUR_FrontEnd_GetNormHitDist(signal.w, view_z, float3(3.0f, 0.1f, 20.0f), roughness) : 0.0f;
  return REBLUR_FrontEnd_PackRadianceAndNormHitDist(signal.xyz, norm_hit_distance, true);
}

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  nrd_view_z_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  if (any(pixel >= uint2(width, height))) return;
  const float view_z = nrd_view_z_image[pixel];
  if (view_z == 1e8f) return;
  const float roughness = abs(2.0f * nrd_normal_roughness_image[pixel].z - 1.0f);
  float4 diffuse = nrd_diffuse_signal_image[pixel];
  float4 specular = nrd_specular_signal_image[pixel];
  if (EE_CAMERA_NRD_PATH_TRACING_ENABLED()) {
    diffuse.xyz /= nrd_diffuse_factor_image[pixel].xyz;
    specular.xyz /= nrd_specular_factor_image[pixel].xyz;
  }
  diffuse.w = nrd_lobe_hit_distance_image[pixel].x;
  specular.w = nrd_lobe_hit_distance_image[pixel].y;
  if (EE_CAMERA_NRD_REBLUR_ENABLED()) {
    diffuse = PackReblurSignal(diffuse, view_z, 1.0f);
    specular = PackReblurSignal(specular, view_z, roughness);
  }
  nrd_diffuse_signal_image[pixel] = diffuse;
  nrd_specular_signal_image[pixel] = specular;
}
