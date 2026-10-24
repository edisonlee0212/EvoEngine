struct Input {
  float4 position : SV_Position;
  [[vk::location(0)]] float2 disc : TEXCOORD0;
  [[vk::location(1)]] float3 color : COLOR0;
};

[shader("fragment")]
float4 main(Input input) : SV_Target {
  if (dot(input.disc, input.disc) > 1.0) discard;
  return float4(input.color * 10.0, 1.0);
}
