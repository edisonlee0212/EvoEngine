// @evoengine-dialect native
import EvoEngine.SMAA;

struct SMAANeighborhoodVertexOutput {
  float4 position : SV_Position;
  [[vk::location(0)]] float2 texcoord : TEXCOORD0;
  [[vk::location(1)]] float4 offset : TEXCOORD1;
};

[shader("vertex")]
SMAANeighborhoodVertexOutput main(uint vertex_index : SV_VertexID) {
  SMAANeighborhoodVertexOutput output;
  float2 uv = float2(float((vertex_index << 1u) & 2u), float(vertex_index & 2u));
  output.texcoord = uv;
  SMAANeighborhoodBlendingVS(uv, output.offset);
  output.position = float4(uv * 2.0f - 1.0f, 0.0f, 1.0f);
  return output;
}
