// @evoengine-dialect native
struct SsrCombineInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

[[vk::binding(0, 1)]]
Sampler2D originalColor;
[[vk::binding(1, 1)]]
Sampler2D reflectedColorVisibility;
[[vk::binding(21, 2)]]
Sampler2D inNormalRoughness;
[[vk::binding(22, 2)]]
Sampler2D inPbrFlags;

[shader("fragment")]
float4 main(SsrCombineInput input) : SV_Target {
  float roughness = inNormalRoughness.Sample(input.tex_coord).a;
  float metallic = inPbrFlags.Sample(input.tex_coord).x;
  float4 color = originalColor.Sample(input.tex_coord);
  float4 reflected = reflectedColorVisibility.Sample(input.tex_coord);
  float factor = clamp(reflected.a * metallic * (1.0f - roughness), 0.0f, 1.0f);
  return float4(color.xyz * (1.0f - factor) + reflected.rgb * factor, color.w);
}
