struct HoverVertexOutput
{
  float4 position : SV_Position;
  [[vk::location(0)]] float2 pixelOffset : TEXCOORD0;
  [[vk::location(1)]] nointerpolation float2 starRadius : TEXCOORD1;
  [[vk::location(2)]] nointerpolation float3 color : COLOR0;
};

[shader("fragment")]
float4 main(HoverVertexOutput input) : SV_Target
{
  const float innerDistance = length(input.pixelOffset / (input.starRadius + 2));
  const float outerDistance = length(input.pixelOffset / (input.starRadius + 4));
  const float innerWidth = max(fwidth(innerDistance), 1e-5);
  const float outerWidth = max(fwidth(outerDistance), 1e-5);
  const float coverage = smoothstep(1 - innerWidth * 0.5, 1 + innerWidth * 0.5, innerDistance) *
    (1 - smoothstep(1 - outerWidth * 0.5, 1 + outerWidth * 0.5, outerDistance));
  if (coverage <= 0)
    discard;
  return float4(input.color, coverage);
}
