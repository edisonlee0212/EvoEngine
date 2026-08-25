import EvoEngine.SMAA;

struct SMAAEdgeVertexOutput {
  float4 position : SV_Position;
  [[vk::location(0)]] float2 texcoord : TEXCOORD0;
  [[vk::location(1)]] float4 offset0 : TEXCOORD1;
  [[vk::location(2)]] float4 offset1 : TEXCOORD2;
  [[vk::location(3)]] float4 offset2 : TEXCOORD3;
};

[shader("vertex")]
SMAAEdgeVertexOutput main(uint vertex_index : SV_VertexID) {
  SMAAEdgeVertexOutput output;
  float2 uv = float2(float((vertex_index << 1u) & 2u), float(vertex_index & 2u));
  float4 offsets[3];
  SMAAEdgeDetectionVS(uv, offsets);
  output.texcoord = uv;
  output.offset0 = offsets[0];
  output.offset1 = offsets[1];
  output.offset2 = offsets[2];
  output.position = float4(uv * 2.0f - 1.0f, 0.0f, 1.0f);
  return output;
}
