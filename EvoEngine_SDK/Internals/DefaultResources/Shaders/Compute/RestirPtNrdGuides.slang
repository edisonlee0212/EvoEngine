import EvoEngine.CameraRayIntegrator;
import EvoEngine.RestirPtPreviousSurface;
import EvoEngine.CameraRayTraversalPolicies;

[[vk::binding(11, 2)]] RWByteAddressBuffer primary_surface_buffer;
[[vk::binding(23, 2)]] RWTexture2D<float> view_z_image;
[[vk::binding(24, 2)]] RWTexture2D<float4> motion_image;
[[vk::binding(28, 2)]] RWTexture2D<float4> nrd_normal_roughness_image;

float3 EE_RESTIR_PT_NRD_PACK_NORMAL_ROUGHNESS(float3 normal, const float roughness) {
  // Matches NRD_FrontEnd_PackNormalAndRoughness for R10G10B10A2 and linear roughness.
  normal /= dot(abs(normal), float3(1.0f));
  float3 packed;
  packed.y = normal.y * 0.5f + 0.5f;
  packed.x = normal.x * 0.5f + packed.y;
  packed.y -= normal.x * 0.5f;
  const float signed_roughness = (normal.z < 0.0f ? -1.0f : 1.0f) * max(saturate(roughness), 1.5f / 512.0f);
  packed.z = signed_roughness * 0.5f + 0.5f;
  return packed;
}

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  view_z_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  if (any(pixel >= uint2(width, height))) return;

  RestirPtPrimarySurface current =
      EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, pixel.y * width + pixel.x);
  float4 raw_normal = ray_normal_image[int2(pixel)];
  const Camera camera = EE_CAMERA_RESTIR_ACTIVE_CAMERA();
  if (camera.restir_enhanced_mode != 0u)
    EE_CAMERA_NRD_TRACE_CENTER_GUIDE<CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>,
                                     EE_GLTF_COMPILED_FEATURE_MASK>(pixel, uint2(width, height), current, raw_normal);
  const bool valid_normal = all(isfinite(raw_normal)) && dot(raw_normal.xyz, raw_normal.xyz) > 1e-8f;
  const float3 packed_normal = EE_RESTIR_PT_NRD_PACK_NORMAL_ROUGHNESS(
      valid_normal ? normalize(raw_normal.xyz) : float3(0.0f, 0.0f, 1.0f),
      valid_normal ? raw_normal.w : 1.0f);
  nrd_normal_roughness_image[int2(pixel)] = float4(packed_normal, 0.0f);
  if (current.material_id == EE_RESTIR_PT_INVALID_INSTANCE || !all(isfinite(current.position))) {
    view_z_image[int2(pixel)] = 1e8f;
    motion_image[int2(pixel)] = float4(0.0f);
    return;
  }

  const float view_z = mul(float4(current.position, 1.0f), camera.view).z;
  if (!isfinite(view_z)) {
    view_z_image[int2(pixel)] = 1e8f;
    motion_image[int2(pixel)] = float4(2.0f, 2.0f, 0.0f, 0.0f);
    return;
  }
  view_z_image[int2(pixel)] = view_z;
  RestirPtPrimarySurface previous;
  if (!EE_RESTIR_PT_PREVIOUS_SURFACE(current, previous)) {
    motion_image[int2(pixel)] = float4(2.0f, 2.0f, 0.0f, 0.0f);
    return;
  }

  const float4 current_clip = mul(float4(current.position, 1.0f), camera.unjittered_projection_view);
  const float4 previous_clip = mul(float4(previous.position, 1.0f), camera.previous_unjittered_projection_view);
  if (current_clip.w <= 0.0f || previous_clip.w <= 0.0f ||
      any(!isfinite(current_clip)) || any(!isfinite(previous_clip))) {
    motion_image[int2(pixel)] = float4(2.0f, 2.0f, 0.0f, 0.0f);
    return;
  }

  const float2 motion = 0.5f * (previous_clip.xy / previous_clip.w - current_clip.xy / current_clip.w);
  const float3 previous_camera_position = camera.previous_inverse_view[3].xyz;
  const float3 previous_camera_back = camera.previous_inverse_view[2].xyz;
  const float previous_view_z = dot(previous.position - previous_camera_position, previous_camera_back);
  const float3 motion_2_5d = float3(motion, previous_view_z - view_z);
  motion_image[int2(pixel)] = all(isfinite(motion_2_5d))
                                  ? float4(motion_2_5d, 0.0f)
                                  : float4(2.0f, 2.0f, 0.0f, 0.0f);
}
