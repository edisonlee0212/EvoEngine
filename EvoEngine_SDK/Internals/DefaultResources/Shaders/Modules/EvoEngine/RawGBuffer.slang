import EvoEngine.GBuffer;

public struct RawGBufferOutput {
  [[vk::location(0)]] float4 uv : SV_Target0;
  [[vk::location(1)]] float4 geometric_frame : SV_Target1;
  [[vk::location(2)]] float4 uv_ddx : SV_Target2;
  [[vk::location(3)]] float4 uv_ddy : SV_Target3;
  [[vk::location(4)]] uint4 metadata : SV_Target4;
};

public RawGBufferOutput EE_BUILD_RAW_GBUFFER(float2 uv0, float2 uv1, float3 geometric_normal, float3 tangent,
                                             float tangent_handedness, uint instance_index, uint material_index,
                                             uint info_index, float4 vertex_color,
                                             bool vertex_color_replaces_base_color) {
  RawGBufferOutput output;
  const float3 normal = normalize(geometric_normal);
  output.uv = float4(uv0, uv1);
  output.geometric_frame =
      float4(EE_GBUFFER_OCT_ENCODE(normal), EE_GBUFFER_OCT_ENCODE(tangent));
  output.uv_ddx = float4(ddx(uv0), ddx(uv1));
  output.uv_ddy = float4(ddy(uv0), ddy(uv1));
  output.metadata = EE_GBUFFER_PACK_METADATA(instance_index, material_index, info_index, vertex_color,
                                              vertex_color_replaces_base_color, tangent_handedness < 0.0f);
  return output;
}
