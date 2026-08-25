struct CubemapVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
};

struct CubemapVertexOutput {
  float4 position : SV_Position;
  [[vk::location(0)]] float3 world_position : TEXCOORD0;
};

struct CubemapVertexConstants {
  float4x4 projection_view;
};

[[vk::push_constant]] ConstantBuffer<CubemapVertexConstants> constants;

[shader("vertex")]
CubemapVertexOutput main(CubemapVertexInput input) {
  CubemapVertexOutput output;
  output.world_position = input.position;
  output.position = mul(float4(input.position, 1.0f), constants.projection_view);
  return output;
}
