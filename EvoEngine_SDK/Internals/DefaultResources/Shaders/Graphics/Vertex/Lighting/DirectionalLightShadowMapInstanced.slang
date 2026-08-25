import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.InstancedData;

struct ShadowVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
};

[shader("vertex")]
float4 main(ShadowVertexInput input, uint draw_id : SV_DrawIndex, uint draw_instance : SV_InstanceID) : SV_Position {
  uint instance_index = draw_id + uint(EE_BASIC_CONSTANTS.instance_index);
  float4 position = mul(float4(input.position, 1.0f), EE_INSTANCED_DATA[draw_instance].instance_matrix);
  position = mul(position, EE_INSTANCES[instance_index].model);
  return mul(position, EE_DIRECTIONAL_LIGHTS[EE_BASIC_CONSTANTS.camera_index]
                           .light_space_matrix[EE_BASIC_CONSTANTS.light_split_index]);
}
