// @evoengine-dialect native
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

float3 GetNormal(float2 tex_coord) {
  return inNormalRoughness.SampleLevel(tex_coord, 0.0f).rgb;
}

float3 GetViewNormal(float2 tex_coord) {
  return normalize(mul(float4(GetNormal(tex_coord), 0.0f), EE_CAMERAS[EE_SSR_CONSTANTS.camera_index].view).xyz);
}

bool GetViewPosition(float2 tex_coord, out float3 position) {
  float ndc_depth = inDepth.SampleLevel(tex_coord, 0.0f).x;
  if (ndc_depth == 1.0f) return false;
  position = EE_DEPTH_TO_VIEW_POS(EE_SSR_CONSTANTS.camera_index, tex_coord, ndc_depth);
  return true;
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
  float4 src_color = inColor.SampleLevel(tex_coord, 0.0f);
  outSrcColor[int2(pixel)] = src_color;

  int steps = EE_SSR_CONSTANTS.initial_steps;
  float2 uv = float2(0.0f);
  float3 position_from;
  if (!GetViewPosition(tex_coord, position_from)) {
    outDstColorVisibility[int2(pixel)] = float4(0.0f);
    return;
  }
  float3 position_to = position_from;
  float3 unit_position_from = normalize(position_from);
  float3 normal = GetViewNormal(tex_coord);
  float3 pivot = normalize(reflect(unit_position_from, normal));

  float4 start_view = float4(position_from + pivot * 0.0f, 1.0f);
  float4 end_view = float4(position_from + pivot * EE_SSR_CONSTANTS.max_distance, 1.0f);
  float4 start_frag_temp = mul(start_view, EE_CAMERAS[EE_SSR_CONSTANTS.camera_index].projection);
  start_frag_temp.xyz /= start_frag_temp.w;
  start_frag_temp.xy = start_frag_temp.xy * 0.5f + 0.5f;
  float4 end_frag_temp = mul(end_view, EE_CAMERAS[EE_SSR_CONSTANTS.camera_index].projection);
  end_frag_temp.xyz /= end_frag_temp.w;
  end_frag_temp.xy = end_frag_temp.xy * 0.5f + 0.5f;

  float2 start_frag = start_frag_temp.xy * tex_size;
  float2 end_frag = end_frag_temp.xy * tex_size;
  float2 frag = start_frag;
  uv = frag / tex_size;
  float delta_x = end_frag.x - start_frag.x;
  float delta_y = end_frag.y - start_frag.y;
  float2 increment = float2(delta_x, delta_y) / EE_SSR_CONSTANTS.iteration_count;
  float search0 = 0.0f;
  float search1 = 0.0f;
  int hit0 = 0;
  int hit1 = 0;
  float view_distance = start_view.z;
  float depth = EE_SSR_CONSTANTS.thickness;

  for (int i = 0; i < EE_SSR_CONSTANTS.iteration_count; ++i) {
    frag += increment;
    uv = frag / tex_size;
    GetViewPosition(uv, position_to);
    search1 = clamp((frag.y - start_frag.y) / delta_y, 0.0f, 1.0f);
    view_distance = (start_view.z * end_view.z) / lerp(end_view.z, start_view.z, search1);
    depth = position_to.z - view_distance;
    if (depth > 0.0f && depth < EE_SSR_CONSTANTS.thickness) {
      hit0 = 1;
      break;
    }
    search0 = search1;
  }

  search1 = search0 + (search1 - search0) * 0.5f;
  steps *= hit0;
  for (int i = 0; i < steps; ++i) {
    frag = lerp(start_frag, end_frag, search1);
    uv = frag / tex_size;
    GetViewPosition(uv, position_to);
    view_distance = (start_view.z * end_view.z) / lerp(end_view.z, start_view.z, search1);
    depth = position_to.z - view_distance;
    if (depth > 0.0f && depth < EE_SSR_CONSTANTS.thickness) {
      hit1 = 1;
      search1 = search0 + (search1 - search0) * 0.5f;
    } else {
      float temp = search1;
      search1 += (search1 - search0) * 0.5f;
      search0 = temp;
    }
  }

  float visibility = float(hit1) * (1.0f - max(dot(-unit_position_from, pivot), 0.0f)) *
                     (1.0f - clamp(depth / EE_SSR_CONSTANTS.thickness, 0.0f, 1.0f)) *
                     (1.0f - pow(clamp(length(position_to - position_from) / EE_SSR_CONSTANTS.max_distance, 0.0f, 1.0f),
                                  EE_SSR_CONSTANTS.distance_confidence)) *
                     (uv.x < 0.0f || uv.x > 1.0f ? 0.0f : 1.0f) *
                     (uv.y < 0.0f || uv.y > 1.0f ? 0.0f : 1.0f);
  visibility = clamp(visibility, 0.0f, 1.0f);
  outDstColorVisibility[int2(pixel)] = visibility != 0.0f
                                           ? max(float4(inColor.SampleLevel(uv, 0.0f).rgb, visibility), float4(0.0f))
                                           : float4(0.0f);
}
