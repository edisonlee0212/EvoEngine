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
  if (radiusSquared >= 1.0)
    discard;
  return float4(input.radiance, 1.0);
}
