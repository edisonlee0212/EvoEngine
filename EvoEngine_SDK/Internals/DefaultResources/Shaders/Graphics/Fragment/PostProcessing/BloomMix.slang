struct BloomMixInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

[[vk::binding(0, 1)]]
Sampler2D originalColor;

[[vk::binding(1, 1)]]
Sampler2D bloomColor;

[shader("fragment")]
float4 main(BloomMixInput input) : SV_Target {
  float4 color = originalColor.Sample(input.tex_coord);
  return float4(clamp(color.xyz, float3(0.0f), float3(1.0f)) + bloomColor.Sample(input.tex_coord).xyz, color.w);
}
