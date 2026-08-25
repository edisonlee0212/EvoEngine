import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;

struct ShadowVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
};

[shader("vertex")]
float4 main(ShadowVertexInput input, uint draw_id : SV_DrawIndex) : SV_Position {
  uint instance_index =
      EE_RASTER_DRAW_INSTANCE_INDEX(draw_id, EE_BASIC_CONSTANTS.instance_index, EE_BASIC_CONSTANTS.meshlet_culling_flags);
  float4 position = mul(float4(input.position, 1.0f), EE_INSTANCES[instance_index].model);
  return mul(position, EE_DIRECTIONAL_LIGHTS[EE_BASIC_CONSTANTS.camera_index]
                           .light_space_matrix[EE_BASIC_CONSTANTS.light_split_index]);
}
