struct StarVertexOutput
{
  float4 position : SV_Position;
  [[vk::location(0)]] float2 discPosition : TEXCOORD0;
  [[vk::location(1)]] float3 radiance : COLOR0;
};

[shader("fragment")]
float4 main(StarVertexOutput input) : SV_Target
{
  const float radiusSquared = dot(input.discPosition, input.discPosition);
  const float edgeWidth = max(fwidth(radiusSquared), 0.0001);
  if (radiusSquared > 1.0 + edgeWidth)
    discard;
  const float coverage = 1.0 - smoothstep(1.0 - edgeWidth, 1.0 + edgeWidth, radiusSquared);
  const float radial = saturate(1.0 - radiusSquared);
  return float4(input.radiance * radial * radial * coverage, 0.0);
}
