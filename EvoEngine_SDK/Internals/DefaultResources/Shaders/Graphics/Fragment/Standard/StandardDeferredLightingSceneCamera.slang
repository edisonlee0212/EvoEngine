import EvoEngine.LightingFixedSet3;
import EvoEngine.GBuffer;

struct DeferredLightingInput {
  [[vk::location(0)]] float2 tex_coord : TEXCOORD0;
};

[[vk::binding(17, 1)]] Sampler2D inDepth;
[[vk::binding(20, 1)]] Sampler2D inBaseColorAO;
[[vk::binding(21, 1)]] Sampler2D inNormalRoughness;
[[vk::binding(22, 1)]] Sampler2D inPbrFlags;
[[vk::binding(23, 1)]] Sampler2D inEmissive;
[[vk::binding(24, 1)]] Texture2D<uint4> inUtility;
[[vk::binding(4, 3)]] Sampler2D inAmbientOcclusion;

[shader("fragment")]
float4 main(DeferredLightingInput input) : SV_Target {
  int camera_index = EE_BASIC_CONSTANTS.camera_index;
  int indirect_debug_view = EE_INDIRECT_LIGHTING_DEBUG_VIEW();
  float ndc_depth = inDepth.Sample(input.tex_coord).x;
  uint utility_width;
  uint utility_height;
  inUtility.GetDimensions(utility_width, utility_height);
  int2 utility_texel = clamp(int2(input.tex_coord * float2(utility_width, utility_height)), int2(0),
                             int2(utility_width, utility_height) - int2(1));
  uint4 utility = inUtility.Load(int3(utility_texel, 0));
  uint instance_index = utility.x;
  uint material_index = utility.y & EE_GBUFFER_MATERIAL_INDEX_MASK;
  uint info_index = utility.z;
  float4 base_color_ao = inBaseColorAO.Sample(input.tex_coord);
  float4 normal_roughness = inNormalRoughness.Sample(input.tex_coord);
  float4 pbr_flags = inPbrFlags.Sample(input.tex_coord);
  float4 emissive_sample = inEmissive.Sample(input.tex_coord);
  float3 normal = normalize(normal_roughness.xyz);
  float3 world_position = EE_DEPTH_TO_WORLD_POS(camera_index, input.tex_coord, ndc_depth);
  float3 camera_position = EE_CAMERA_POSITION(camera_index);
  float3 sky_color = EE_SKY_COLOR(world_position - camera_position);

  if (ndc_depth == 1.0f) {
    if (indirect_debug_view != 0) return float4(0.0f, 0.0f, 0.0f, 1.0f);
    return float4(sky_color, 1.0f);
  }

  float depth = EE_LINEARIZE_DEPTH(camera_index, ndc_depth);
  if (indirect_debug_view == 0) {
    float4 shadow_debug_color = EE_FUNC_DIRECTIONAL_SHADOW_DEBUG(depth, world_position);
    if (shadow_debug_color.a > 0.0f) return shadow_debug_color;
  }

  float3 base = base_color_ao.rgb;
  float3 material_kernel = abs(EE_KERNEL_BLOCK.uniform_kernel[material_index % EE_MAX_KERNEL_AMOUNT].xyz);
  float3 instance_kernel = abs(EE_KERNEL_BLOCK.uniform_kernel[instance_index % EE_MAX_KERNEL_AMOUNT].xyz);
  float3 info_kernel = abs(EE_KERNEL_BLOCK.uniform_kernel[info_index % EE_MAX_KERNEL_AMOUNT].xyz);
  int debug_visualization = EE_RENDER_INFO.debug_visualization;
  float mode0 = float(debug_visualization == 0 && info_index > 1u);
  float mode1 = float(debug_visualization == 1);
  float mode2 = float(debug_visualization == 2);
  float mode3 = float(debug_visualization == 3);
  float any_debug = indirect_debug_view == 0 ? clamp(mode0 + mode1 + mode2 + mode3, 0.0f, 1.0f) : 0.0f;
  float3 debug_color = base * mode0 + material_kernel * mode1 + instance_kernel * mode2 + info_kernel * mode3;
  float3 albedo = lerp(base_color_ao.rgb, debug_color, any_debug);
  float3 color = indirect_debug_view == 0 ? albedo : float3(0.0f);

  if (emissive_sample.a >= 0.0f) {
    float3 view_direction = normalize(camera_position - world_position);
    float3 direct = EE_FUNC_CALCULATE_LIGHTS(
        true, albedo, 1.0f, depth, normal, view_direction, world_position, pbr_flags.x, normal_roughness.a,
        pbr_flags.yzw, emissive_sample.a);
    float3 ambient = EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_LIGHT(
        albedo, normal, view_direction, world_position, pbr_flags.x, normal_roughness.a, pbr_flags.yzw,
        emissive_sample.a, base_color_ao.a, inAmbientOcclusion.Sample(input.tex_coord).r);
    if (indirect_debug_view != 0) return float4(ambient, 1.0f);
    color = direct + emissive_sample.rgb + ambient;
  }
  if (indirect_debug_view != 0) return float4(0.0f, 0.0f, 0.0f, 1.0f);

  float4 output_color = float4(color, 1.0f);
  float fade_ratio = EE_CAMERA_FADE_RATIO(camera_index);
  float far_distance = EE_CAMERA_FAR(camera_index);
  float fade_start = far_distance * fade_ratio;
  if (depth > fade_start)
    output_color.rgb =
        lerp(output_color.rgb, sky_color, clamp((depth - fade_start) / (far_distance * (1.0f - fade_ratio)), 0.0f, 1.0f));
  return output_color;
}
