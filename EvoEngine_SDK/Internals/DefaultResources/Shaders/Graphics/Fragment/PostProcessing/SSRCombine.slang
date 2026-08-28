import EvoEngine.LightingFixedSet3;
import EvoEngine.SSRConstants;

struct SsrCombineInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

[[vk::binding(0, 1)]]
Sampler2D originalColor;
[[vk::binding(1, 1)]]
Sampler2D reflectedColorVisibility;
[[vk::binding(17, 2)]]
Sampler2D inDepth;
[[vk::binding(20, 2)]]
Sampler2D inBaseColorAO;
[[vk::binding(21, 2)]]
Sampler2D inNormalRoughness;
[[vk::binding(22, 2)]]
Sampler2D inPbrFlags;
[[vk::binding(23, 2)]]
Sampler2D inEmissive;
[[vk::binding(4, 3)]]
Sampler2D inAmbientOcclusion;

float3 FresnelSchlickRoughness(float normal_dot_view, float3 f0, float f90, float roughness) {
  float3 grazing = max(float3(f90 * (1.0f - roughness)), f0);
  return f0 + (grazing - f0) * pow(max(1.0f - clamp(normal_dot_view, 0.0f, 1.0f), 0.0f), 5.0f);
}

float3 ComposeReflection(float2 tex_coord, float3 original, float4 reflected) {
  if (EE_SSR_CONSTANTS.debug_mode != 0) return reflected.rgb;
  float ndc_depth = inDepth.SampleLevel(tex_coord, 0.0f).x;
  if (!isfinite(ndc_depth) || ndc_depth >= 1.0f || reflected.a <= 0.0f || EE_INDIRECT_LIGHTING_DEBUG_VIEW() != 0) {
    return original;
  }

  float4 base_color_ao = inBaseColorAO.SampleLevel(tex_coord, 0.0f);
  float4 normal_roughness = inNormalRoughness.SampleLevel(tex_coord, 0.0f);
  float4 pbr_flags = inPbrFlags.SampleLevel(tex_coord, 0.0f);
  float4 emissive = inEmissive.SampleLevel(tex_coord, 0.0f);
  if (emissive.a < 0.0f) return original;

  float normal_length = length(normal_roughness.xyz);
  if (!isfinite(normal_length) || normal_length <= 0.00001f) return original;
  float3 normal = normal_roughness.xyz / normal_length;
  float roughness = clamp(normal_roughness.a, 0.0f, 1.0f);
  float metallic = clamp(pbr_flags.x, 0.0f, 1.0f);
  float3 f0 = clamp(pbr_flags.yzw, float3(0.0f), float3(1.0f));
  float f90 = clamp(emissive.a, 0.0f, 1.0f);
  float3 world_position = EE_DEPTH_TO_WORLD_POS(EE_SSR_CONSTANTS.camera_index, tex_coord, ndc_depth);
  float3 view_direction = normalize(EE_CAMERA_POSITION(EE_SSR_CONSTANTS.camera_index) - world_position);
  if (!all(isfinite(world_position)) || !all(isfinite(view_direction))) return original;

  float normal_dot_view = max(dot(normal, view_direction), 0.0f);
  float3 fresnel = FresnelSchlickRoughness(normal_dot_view, f0, f90, roughness);
  float trace_confidence = clamp(reflected.a * (1.0f - roughness), 0.0f, 1.0f);

  float screen_visibility = inAmbientOcclusion.SampleLevel(tex_coord, 0.0f).r;
  EeEnvironmentalLighting environment = EE_FUNC_CALCULATE_DDGI_ENVIRONMENTAL_COMPONENTS(
      base_color_ao.rgb, normal, view_direction, world_position, metallic, roughness, f0, f90, base_color_ao.a,
      screen_visibility);
  float2 brdf = EE_SAMPLE_RASTER_BRDF(float2(normal_dot_view, roughness));
  float3 ssr_specular = reflected.rgb * (fresnel * brdf.x + float3(f90 * brdf.y)) * environment.specular_visibility;
  float3 result = original + (ssr_specular - environment.specular) * trace_confidence;
  return max(all(isfinite(result)) ? result : original, float3(0.0f));
}

[shader("fragment")]
float4 main(SsrCombineInput input) : SV_Target {
  float4 original = originalColor.SampleLevel(input.tex_coord, 0.0f);
  float4 reflected = reflectedColorVisibility.SampleLevel(input.tex_coord, 0.0f);
  return float4(ComposeReflection(input.tex_coord, original.rgb, reflected), original.a);
}
