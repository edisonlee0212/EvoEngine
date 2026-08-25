import EvoEngine.PerFrame;

struct EcoSysLabRasterInput {
  [[vk::location(0)]] float3 frag_pos : TEXCOORD0;
  [[vk::location(1)]] float3 normal : TEXCOORD1;
  [[vk::location(2)]] float3 tangent : TEXCOORD2;
  [[vk::location(3)]] float4 color : TEXCOORD3;
};

struct EcoSysLabFragmentOutput {
  [[vk::location(0)]] float4 out_color : SV_Target0;
};
[shader("fragment")]
EcoSysLabFragmentOutput main(EcoSysLabRasterInput fs_in, bool is_front_face : SV_IsFrontFace) {
  EcoSysLabFragmentOutput output;
  output.out_color = float4(fs_in.color);
  return output;
}