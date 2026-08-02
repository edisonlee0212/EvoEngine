// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.Skinning;

struct SkinnedShadowVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
  [[vk::location(5)]] int4 bone_ids : BLENDINDICES0;
  [[vk::location(6)]] float4 weights : BLENDWEIGHT0;
  [[vk::location(7)]] int4 bone_ids2 : BLENDINDICES1;
  [[vk::location(8)]] float4 weights2 : BLENDWEIGHT1;
};

[shader("vertex")]
float4 main(SkinnedShadowVertexInput input, uint draw_id : SV_DrawIndex) : SV_Position {
  uint instance_index = draw_id + uint(EE_BASIC_CONSTANTS.instance_index);
  float4 position = mul(float4(input.position, 1.0f),
                        EE_SKINNING_MATRIX(input.bone_ids, input.weights, input.bone_ids2, input.weights2));
  position = mul(position, EE_INSTANCES[instance_index].model);
  return mul(position,
             EE_POINT_LIGHTS[EE_BASIC_CONSTANTS.camera_index].light_space_matrix[EE_BASIC_CONSTANTS.light_split_index]);
}
