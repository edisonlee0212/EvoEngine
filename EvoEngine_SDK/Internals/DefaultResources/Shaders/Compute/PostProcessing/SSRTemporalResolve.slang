import EvoEngine.Cameras;
import EvoEngine.SSRConstants;

[[vk::binding(0, 2)]] Sampler2D inCurrentReflectionConfidence;
[[vk::binding(1, 2)]] Sampler2D inHistoryReflectionConfidence;
[[vk::binding(2, 2)]] Sampler2D inMotionVectors;
[[vk::binding(3, 2)]] Sampler2D inHistoryGeometry;
[[vk::binding(4, 2)]] Sampler2D inHistoryMaterial;

[[vk::binding(5, 2)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outReflectionConfidence;
[[vk::binding(6, 2)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outGeometry;
[[vk::binding(7, 2)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outMaterial;

[[vk::binding(17, 1)]] Sampler2D inDepth;
[[vk::binding(21, 1)]] Sampler2D inNormalRoughness;
[[vk::binding(22, 1)]] Sampler2D inPbrFlags;

float CameraFarDistance() {
  float4x4 projection = EE_CAMERAS[EE_SSR_CONSTANTS.camera_index].projection;
  return abs(projection[3][2] / (projection[2][2] + 1.0f));
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  uint width = 0;
  uint height = 0;
  outReflectionConfidence.GetDimensions(width, height);
  uint2 resolution = uint2(width, height);
  if (any(pixel >= resolution)) return;

  float2 texel_size = 1.0f / float2(resolution);
  float2 tex_coord = (float2(pixel) + float2(0.5f)) * texel_size;
  float4 current = inCurrentReflectionConfidence.Load(int3(pixel, 0));
  float ndc_depth = inDepth.Load(int3(pixel, 0)).x;
  float4 normal_roughness = inNormalRoughness.Load(int3(pixel, 0));
  float4 pbr_flags = inPbrFlags.Load(int3(pixel, 0));
  float normal_length = length(normal_roughness.xyz);
  float3 normal = normal_length > 0.00001f ? normal_roughness.xyz / normal_length : float3(0.0f);
  float linear_depth = 1.0f;
  if (isfinite(ndc_depth) && ndc_depth < 1.0f) {
    float3 view_position = EE_DEPTH_TO_VIEW_POS(EE_SSR_CONSTANTS.camera_index, tex_coord, ndc_depth);
    linear_depth = clamp(abs(view_position.z) / max(CameraFarDistance(), 0.00001f), 0.0f, 1.0f);
  }
  float4 current_geometry = float4(normal, linear_depth);
  float4 current_material = float4(clamp(normal_roughness.a, 0.0f, 1.0f),
                                   clamp(pbr_flags.yzw, float3(0.0f), float3(1.0f)));
  outGeometry[int2(pixel)] = current_geometry;
  outMaterial[int2(pixel)] = current_material;

  bool can_reproject = EE_SSR_CONSTANTS.temporal_enabled != 0 && EE_SSR_CONSTANTS.temporal_history_valid != 0 &&
                       EE_SSR_CONSTANTS.debug_mode == 0 && ndc_depth < 1.0f && normal_length > 0.00001f;
  float4 motion = inMotionVectors.Load(int3(pixel, 0));
  float2 history_uv = tex_coord + motion.xy * texel_size;
  can_reproject = can_reproject && all(isfinite(motion)) && all(history_uv > float2(0.0f)) &&
                  all(history_uv < float2(1.0f));
  if (!can_reproject) {
    outReflectionConfidence[int2(pixel)] = current;
    return;
  }

  float4 history_geometry = inHistoryGeometry.SampleLevel(history_uv, 0.0f);
  float4 history_material = inHistoryMaterial.SampleLevel(history_uv, 0.0f);
  float expected_history_depth = linear_depth + motion.z;
  float depth_tolerance = max(0.002f, 0.02f * max(expected_history_depth, linear_depth));
  bool geometry_valid = abs(history_geometry.a - expected_history_depth) <= depth_tolerance &&
                        dot(normal, history_geometry.xyz) >= 0.9f;
  bool material_valid = abs(current_material.x - history_material.x) <= 0.1f &&
                        all(abs(current_material.yzw - history_material.yzw) <= float3(0.1f));
  if (!geometry_valid || !material_valid) {
    outReflectionConfidence[int2(pixel)] = current;
    return;
  }

  float4 history = inHistoryReflectionConfidence.SampleLevel(history_uv, 0.0f);
  if (!all(isfinite(history)) || history.a <= 0.0001f) {
    outReflectionConfidence[int2(pixel)] = current;
    return;
  }

  float3 neighborhood_min = float3(3.402823466e+38f);
  float3 neighborhood_max = float3(-3.402823466e+38f);
  bool neighborhood_valid = false;
  for (int y = -1; y <= 1; ++y) {
    for (int x = -1; x <= 1; ++x) {
      int2 sample_pixel = clamp(int2(pixel) + int2(x, y), int2(0), int2(resolution) - int2(1));
      float4 sample_value = inCurrentReflectionConfidence.Load(int3(sample_pixel, 0));
      if (sample_value.a <= 0.0001f || !all(isfinite(sample_value))) continue;
      neighborhood_min = min(neighborhood_min, sample_value.rgb);
      neighborhood_max = max(neighborhood_max, sample_value.rgb);
      neighborhood_valid = true;
    }
  }
  float3 clipped_history = neighborhood_valid ? clamp(history.rgb, neighborhood_min, neighborhood_max) : history.rgb;
  float history_weight = min(0.9f, history.a * 0.9f);
  float3 resolved = current.a > 0.0001f ? lerp(current.rgb, clipped_history, history_weight) : clipped_history;
  float confidence = max(current.a, history.a * 0.95f);
  outReflectionConfidence[int2(pixel)] =
      all(isfinite(resolved)) ? float4(max(resolved, float3(0.0f)), clamp(confidence, 0.0f, 1.0f)) : current;
}
