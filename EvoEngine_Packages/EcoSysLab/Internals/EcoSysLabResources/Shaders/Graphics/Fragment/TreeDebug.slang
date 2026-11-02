struct TreeDebugInput {
  float4 position : SV_Position;
  [[vk::location(0)]] float4 color : TEXCOORD0;
  [[vk::location(1)]] nointerpolation uint2 node : TEXCOORD1;
  [[vk::location(2)]] float length_factor : TEXCOORD2;
  [[vk::location(3)]] nointerpolation float selected_factor : TEXCOORD3;
};
struct TreeDebugOutput {
  float4 color : SV_Target0;
  uint4 pick : SV_Target1;
};
[shader("fragment")]
TreeDebugOutput main(TreeDebugInput input) {
  TreeDebugOutput result;
  result.color = input.color;
  if (input.selected_factor >= 0.0) {
    result.color.rgb = lerp(result.color.rgb, float3(1.0, 0.8, 0.05), 0.65);
    if (abs(input.length_factor - input.selected_factor) < 0.05)
      result.color.rgb = float3(1.0);
  }
  result.pick = uint4(input.node, asuint(saturate(input.length_factor)), 0u);
  return result;
}
