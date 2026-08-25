import EvoEngine.Atmosphere;

struct AtmosphereVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
};

struct AtmosphereVertexOutput {
  float4 position : SV_Position;
  [[vk::location(0)]] float3 world_position : TEXCOORD0;
};

struct AtmosphereVertexConstants {
  float4x4 projection_view;
  Atmosphere atmosphere;
  float3 sun_direction;
  float padding;
};

[[vk::push_constant]] ConstantBuffer<AtmosphereVertexConstants> constants;

[shader("vertex")]
AtmosphereVertexOutput main(AtmosphereVertexInput input) {
  AtmosphereVertexOutput output;
  output.world_position = input.position;
  output.position = mul(float4(input.position, 1.0f), constants.projection_view);
  return output;
}
