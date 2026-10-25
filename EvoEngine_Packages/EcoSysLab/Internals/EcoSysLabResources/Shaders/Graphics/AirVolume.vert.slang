struct Output { float4 position : SV_Position; };

[shader("vertex")]
Output main(uint vertex_id : SV_VertexID) {
  Output output;
  output.position = float4(float2((vertex_id << 1) & 2, vertex_id & 2) * 2.0 - 1.0, 0.0, 1.0);
  return output;
}
