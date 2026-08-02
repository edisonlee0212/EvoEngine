// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.SSRConstants;

struct SsrReflectInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

struct SsrReflectOutput {
  [[vk::location(0)]]
  float4 source_color : SV_Target0;
  [[vk::location(1)]]
  float4 reflected_color_visibility : SV_Target1;
};

[[vk::binding(17, 1)]]
Sampler2D inDepth;
[[vk::binding(21, 1)]]
Sampler2D inNormalRoughness;
[[vk::binding(0, 2)]]
Sampler2D inColor;

float3 GetNormal(float2 tex_coord) {
  return inNormalRoughness.Sample(tex_coord).rgb;
}

float3 GetViewNormal(float2 tex_coord) {
  return normalize(mul(float4(GetNormal(tex_coord), 0.0f), EE_CAMERAS[EE_SSR_CONSTANTS.camera_index].view).xyz);
}

bool GetViewPosition(float2 tex_coord, out float3 position) {
  float ndc_depth = inDepth.Sample(tex_coord).x;
  if (ndc_depth == 1.0f) return false;
  position = EE_DEPTH_TO_VIEW_POS(EE_SSR_CONSTANTS.camera_index, tex_coord, ndc_depth);
  return true;
}

[shader("fragment")]
SsrReflectOutput main(SsrReflectInput input) {
  SsrReflectOutput output;
  float2 tex_size;
  uint width;
  uint height;
  inColor.GetDimensions(width, height);
  tex_size = float2(width, height);
  float2 tex_coord = input.tex_coord;
  output.source_color = inColor.Sample(tex_coord);

  int steps = EE_SSR_CONSTANTS.initial_steps;
  float2 uv = float2(0.0f);
  float3 position_from;
  if (!GetViewPosition(tex_coord, position_from)) {
    output.reflected_color_visibility = float4(0.0f);
    return output;
  }

  float3 position_to = position_from;
  float3 unit_position_from = normalize(position_from);
  float3 pivot = normalize(reflect(unit_position_from, GetViewNormal(tex_coord)));
  float4 start_view = float4(position_from, 1.0f);
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
  float delta_x = end_frag.x - start_frag.x;
  float delta_y = end_frag.y - start_frag.y;
  float2 increment = float2(delta_x, delta_y) / float(EE_SSR_CONSTANTS.iteration_count);
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
    search1 = lerp((frag.y - start_frag.y) / delta_y, (frag.x - start_frag.x) / delta_x, 0.0f);
    search1 = clamp(search1, 0.0f, 1.0f);
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
      float previous = search1;
      search1 += (search1 - search0) * 0.5f;
      search0 = previous;
    }
  }

  float visibility = float(hit1) * (1.0f - max(dot(-unit_position_from, pivot), 0.0f)) *
                     (1.0f - clamp(depth / EE_SSR_CONSTANTS.thickness, 0.0f, 1.0f)) *
                     (1.0f - pow(clamp(length(position_to - position_from) / EE_SSR_CONSTANTS.max_distance, 0.0f, 1.0f),
                                 EE_SSR_CONSTANTS.distance_confidence)) *
                     (uv.x < 0.0f || uv.x > 1.0f ? 0.0f : 1.0f) *
                     (uv.y < 0.0f || uv.y > 1.0f ? 0.0f : 1.0f);
  visibility = clamp(visibility, 0.0f, 1.0f);
  output.reflected_color_visibility = visibility != 0.0f
                                          ? max(float4(inColor.Sample(uv).rgb, visibility), float4(0.0f))
                                          : float4(0.0f);
  return output;
}
