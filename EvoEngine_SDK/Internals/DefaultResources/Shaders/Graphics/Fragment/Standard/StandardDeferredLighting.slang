import EvoEngine.LightingFixedSet3;

struct DeferredLightingInput {
  [[vk::location(0)]] float2 tex_coord : TEXCOORD0;
};

[[vk::binding(17, 1)]] Sampler2D inDepth;
[[vk::binding(20, 1)]] Sampler2D inBaseColorAO;
[[vk::binding(21, 1)]] Sampler2D inNormalRoughness;
[[vk::binding(22, 1)]] Sampler2D inPbrFlags;
[[vk::binding(23, 1)]] Sampler2D inEmissive;
[[vk::binding(4, 3)]] Sampler2D inAmbientOcclusion;

[shader("fragment")]
float4 main(DeferredLightingInput input) : SV_Target {
  int camera_index = EE_BASIC_CONSTANTS.camera_index;
  int indirect_debug_view = EE_INDIRECT_LIGHTING_DEBUG_VIEW();
  float ndc_depth = inDepth.Sample(input.tex_coord).x;
  float3 world_position = EE_DEPTH_TO_WORLD_POS(camera_index, input.tex_coord, ndc_depth);
  float3 camera_position = EE_CAMERA_POSITION(camera_index);
  float3 sky_color = EE_SKY_COLOR(world_position - camera_position);
  if (ndc_depth == 1.0f)
    return float4(indirect_debug_view == 0 ? sky_color : float3(0.0f), 1.0f);

  float4 base_color_ao = inBaseColorAO.Sample(input.tex_coord);
  float4 normal_roughness = inNormalRoughness.Sample(input.tex_coord);
  float4 pbr_flags = inPbrFlags.Sample(input.tex_coord);
  float4 emissive_sample = inEmissive.Sample(input.tex_coord);
  if (emissive_sample.a < 0.0f)
    return float4(indirect_debug_view == 0 ? base_color_ao.rgb : float3(0.0f), 1.0f);

  float3 normal = normalize(normal_roughness.xyz);
  float depth = EE_LINEARIZE_DEPTH(camera_index, ndc_depth);
  if (indirect_debug_view == 0) {
    float4 shadow_debug_color = EE_FUNC_DIRECTIONAL_SHADOW_DEBUG(depth, world_position);
    if (shadow_debug_color.a > 0.0f) return shadow_debug_color;
  }

  float3 view_direction = normalize(camera_position - world_position);
  float3 direct = EE_FUNC_CALCULATE_LIGHTS(
      true, base_color_ao.rgb, 1.0f, depth, normal, view_direction, world_position, pbr_flags.x,
      normal_roughness.a, pbr_flags.yzw, emissive_sample.a);
  float3 ambient = EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT(
      base_color_ao.rgb, normal, view_direction, world_position, pbr_flags.x, normal_roughness.a, pbr_flags.yzw,
      emissive_sample.a, base_color_ao.a, inAmbientOcclusion.Sample(input.tex_coord).r);
  if (indirect_debug_view != 0) return float4(ambient, 1.0f);

  float3 output_color = direct + emissive_sample.rgb + ambient;
  float fade_ratio = EE_CAMERA_FADE_RATIO(camera_index);
  float far_distance = EE_CAMERA_FAR(camera_index);
  if (depth > far_distance * fade_ratio)
    output_color = lerp(output_color, sky_color,
                        (depth - far_distance * fade_ratio) / (far_distance * (1.0f - fade_ratio)));
  return float4(output_color, 1.0f);
}
