import EvoEngine.PerFrame;
import EvoEngine.SSRConstants;

[[vk::binding(17, 1)]]
Sampler2D inDepth;

[[vk::binding(21, 1)]]
Sampler2D inNormalRoughness;

[[vk::binding(0, 2)]]
Sampler2D inColor;

[[vk::binding(0, 3)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outSrcColor;

[[vk::binding(1, 3)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outDstColorVisibility;

bool IsUvInBounds(float2 tex_coord) {
  return all(tex_coord >= float2(0.0f)) && all(tex_coord <= float2(1.0f));
}

bool GetViewPosition(float2 tex_coord, out float3 position) {
  position = float3(0.0f);
  if (!IsUvInBounds(tex_coord)) return false;
  float ndc_depth = inDepth.SampleLevel(tex_coord, 0.0f).x;
  if (!isfinite(ndc_depth) || ndc_depth >= 1.0f) return false;
  position = EE_DEPTH_TO_VIEW_POS(EE_SSR_CONSTANTS.camera_index, tex_coord, ndc_depth);
  return all(isfinite(position));
}

bool ProjectViewPosition(float3 position, out float2 tex_coord) {
  tex_coord = float2(0.0f);
  float4 clip = mul(float4(position, 1.0f), EE_CAMERAS[EE_SSR_CONSTANTS.camera_index].projection);
  if (!all(isfinite(clip)) || clip.w <= 0.00001f) return false;
  tex_coord = clip.xy / clip.w * 0.5f + 0.5f;
  return all(isfinite(tex_coord));
}

bool SampleRayDepth(float screen_t, float2 start_uv, float2 end_uv, float start_view_z, float end_view_z,
                    out float2 sample_uv, out float3 scene_position, out float depth_delta) {
  sample_uv = lerp(start_uv, end_uv, screen_t);
  scene_position = float3(0.0f);
  depth_delta = 0.0f;
  if (!GetViewPosition(sample_uv, scene_position)) return false;
  float denominator = lerp(end_view_z, start_view_z, screen_t);
  if (!isfinite(denominator) || abs(denominator) <= 0.00001f) return false;
  float ray_view_z = start_view_z * end_view_z / denominator;
  depth_delta = scene_position.z - ray_view_z;
  return isfinite(ray_view_z) && isfinite(depth_delta);
}

float4 RejectionDebugColor(int reason) {
  if (EE_SSR_CONSTANTS.debug_mode != 3) return float4(0.0f);
  if (reason == 2) return float4(1.0f, 0.75f, 0.0f, 1.0f);
  if (reason == 3) return float4(1.0f, 0.0f, 1.0f, 1.0f);
  if (reason == 4) return float4(1.0f, 0.35f, 0.0f, 1.0f);
  if (reason == 5) return float4(0.0f, 0.4f, 1.0f, 1.0f);
  return float4(0.25f, 0.0f, 0.0f, 1.0f);
}

float4 EncodeHit(float2 hit_uv, float normalized_distance, float confidence, float3 radiance) {
  if (EE_SSR_CONSTANTS.debug_mode == 1) return float4(hit_uv, 0.0f, 1.0f);
  if (EE_SSR_CONSTANTS.debug_mode == 2) return float4(float3(normalized_distance), 1.0f);
  if (EE_SSR_CONSTANTS.debug_mode == 3) return float4(0.0f, 1.0f, 0.0f, 1.0f);
  if (EE_SSR_CONSTANTS.debug_mode == 4) return float4(float3(confidence), 1.0f);
  return max(float4(radiance, confidence), float4(0.0f));
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  uint width = 0;
  uint height = 0;
  outSrcColor.GetDimensions(width, height);
  uint2 resolution = uint2(width, height);
  if (any(pixel >= resolution)) return;

  float2 tex_size = float2(resolution);
  float2 tex_coord = (float2(pixel) + float2(0.5f)) / tex_size;
  outSrcColor[int2(pixel)] = inColor.SampleLevel(tex_coord, 0.0f);
  outDstColorVisibility[int2(pixel)] = RejectionDebugColor(1);

  float3 position_from;
  if (!GetViewPosition(tex_coord, position_from)) return;
  float position_length = length(position_from);
  if (!isfinite(position_length) || position_length <= 0.00001f) return;

  float3 world_normal = inNormalRoughness.SampleLevel(tex_coord, 0.0f).rgb;
  float3 view_normal = mul(float4(world_normal, 0.0f), EE_CAMERAS[EE_SSR_CONSTANTS.camera_index].view).xyz;
  float normal_length = length(view_normal);
  if (!all(isfinite(view_normal)) || normal_length <= 0.00001f) return;
  view_normal /= normal_length;

  float3 unit_position_from = position_from / position_length;
  float source_facing = dot(view_normal, -unit_position_from);
  if (!isfinite(source_facing) || source_facing <= 0.001f) {
    outDstColorVisibility[int2(pixel)] = RejectionDebugColor(5);
    return;
  }
  float3 pivot = reflect(unit_position_from, view_normal);
  float pivot_length = length(pivot);
  if (!all(isfinite(pivot)) || pivot_length <= 0.00001f) return;
  pivot /= pivot_length;

  float ray_length = EE_SSR_CONSTANTS.max_distance;
  float near_view_z = -EE_CAMERA_NEAR(EE_SSR_CONSTANTS.camera_index) * 1.01f;
  if (pivot.z > 0.00001f) {
    ray_length = min(ray_length, (near_view_z - position_from.z) / pivot.z);
  }
  if (!isfinite(ray_length) || ray_length <= EE_SSR_CONSTANTS.start_bias + 0.00001f) return;

  float3 start_view = position_from + pivot * EE_SSR_CONSTANTS.start_bias;
  float3 end_view = position_from + pivot * ray_length;
  float2 start_uv;
  float2 end_uv;
  if (!ProjectViewPosition(start_view, start_uv) || !ProjectViewPosition(end_view, end_uv)) return;

  float2 pixel_delta = (end_uv - start_uv) * tex_size;
  float dominant_span = max(abs(pixel_delta.x), abs(pixel_delta.y));
  if (!isfinite(dominant_span) || dominant_span < 0.5f) return;
  int march_step_count = min(EE_SSR_CONSTANTS.iteration_count, max(1, int(ceil(dominant_span))));

  float previous_t = 0.0f;
  float previous_depth_delta = -EE_SSR_CONSTANTS.thickness;
  float bracket_min = 0.0f;
  float bracket_max = 0.0f;
  bool bracket_found = false;
  float2 sample_uv = start_uv;
  float3 scene_position = position_from;
  float depth_delta = 0.0f;

  for (int i = 0; i < march_step_count; ++i) {
    float screen_t = float(i + 1) / float(march_step_count);
    if (!SampleRayDepth(screen_t, start_uv, end_uv, start_view.z, end_view.z, sample_uv, scene_position,
                        depth_delta)) {
      break;
    }
    if (depth_delta > 0.0f && previous_depth_delta <= 0.0f) {
      bracket_min = previous_t;
      bracket_max = screen_t;
      bracket_found = true;
      break;
    }
    previous_t = screen_t;
    previous_depth_delta = depth_delta;
  }
  if (!bracket_found) {
    outDstColorVisibility[int2(pixel)] = RejectionDebugColor(2);
    return;
  }

  for (int i = 0; i < EE_SSR_CONSTANTS.binary_search_iteration_count; ++i) {
    float midpoint = 0.5f * (bracket_min + bracket_max);
    if (!SampleRayDepth(midpoint, start_uv, end_uv, start_view.z, end_view.z, sample_uv, scene_position,
                        depth_delta)) {
      outDstColorVisibility[int2(pixel)] = RejectionDebugColor(3);
      return;
    }
    if (depth_delta > 0.0f) {
      bracket_max = midpoint;
    } else {
      bracket_min = midpoint;
    }
  }

  if (!SampleRayDepth(bracket_max, start_uv, end_uv, start_view.z, end_view.z, sample_uv, scene_position,
                      depth_delta)) {
    outDstColorVisibility[int2(pixel)] = RejectionDebugColor(3);
    return;
  }
  if (depth_delta < 0.0f || depth_delta >= EE_SSR_CONSTANTS.thickness) {
    outDstColorVisibility[int2(pixel)] = RejectionDebugColor(4);
    return;
  }

  float3 hit_world_normal = inNormalRoughness.SampleLevel(sample_uv, 0.0f).rgb;
  float3 hit_view_normal = mul(float4(hit_world_normal, 0.0f), EE_CAMERAS[EE_SSR_CONSTANTS.camera_index].view).xyz;
  float hit_normal_length = length(hit_view_normal);
  if (!all(isfinite(hit_view_normal)) || hit_normal_length <= 0.00001f) {
    outDstColorVisibility[int2(pixel)] = RejectionDebugColor(5);
    return;
  }
  hit_view_normal /= hit_normal_length;
  float hit_facing = dot(hit_view_normal, -pivot);
  if (!isfinite(hit_facing) || hit_facing <= 0.001f) {
    outDstColorVisibility[int2(pixel)] = RejectionDebugColor(5);
    return;
  }

  float normalized_distance = clamp(length(scene_position - position_from) / EE_SSR_CONSTANTS.max_distance, 0.0f, 1.0f);
  float thickness_confidence = 1.0f - clamp(depth_delta / EE_SSR_CONSTANTS.thickness, 0.0f, 1.0f);
  float distance_confidence = 1.0f - pow(normalized_distance, EE_SSR_CONSTANTS.distance_confidence);
  float grazing_confidence = smoothstep(0.02f, 0.25f, source_facing);
  float edge_distance = min(min(sample_uv.x, 1.0f - sample_uv.x), min(sample_uv.y, 1.0f - sample_uv.y));
  float edge_confidence = smoothstep(0.0f, 0.05f, edge_distance);
  float normal_confidence = smoothstep(0.05f, 0.35f, hit_facing);
  float confidence = thickness_confidence * distance_confidence * grazing_confidence * edge_confidence * normal_confidence;
  confidence = clamp(isfinite(confidence) ? confidence : 0.0f, 0.0f, 1.0f);
  outDstColorVisibility[int2(pixel)] =
      EncodeHit(sample_uv, normalized_distance, confidence, inColor.SampleLevel(sample_uv, 0.0f).rgb);
}
