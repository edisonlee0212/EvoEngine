struct BloomCopyInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

struct BloomCopyOutput {
  [[vk::location(0)]]
  float4 source_color : SV_Target0;
  [[vk::location(1)]]
  float4 result_color : SV_Target1;
};

[[vk::binding(0, 1)]]
Sampler2D inColor;

[shader("fragment")]
BloomCopyOutput main(BloomCopyInput input) {
  BloomCopyOutput output;
  output.source_color = inColor.Sample(input.tex_coord);
  output.result_color = float4(max(float3(0.00001f), output.source_color.xyz - float3(1.0f)), 1.0f);
  return output;
}
