import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.Skinning;
import EvoEngine.RasterMotionVectors;

struct SkinnedMotionVectorVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
  [[vk::location(3)]] float2 tex_coord0 : TEXCOORD0;
  [[vk::location(4)]] float4 color : COLOR0;
  [[vk::location(5)]] int4 bone_ids : BLENDINDICES0;
  [[vk::location(6)]] float4 weights : BLENDWEIGHT0;
  [[vk::location(7)]] int4 bone_ids2 : BLENDINDICES1;
  [[vk::location(8)]] float4 weights2 : BLENDWEIGHT1;
  [[vk::location(10)]] float2 tex_coord1 : TEXCOORD1;
};

[[vk::binding(0, 2)]]
StructuredBuffer<float4x4> EE_PREVIOUS_ANIM_BONES;
[[vk::binding(1, 2)]]
StructuredBuffer<PreviousInstance> EE_PREVIOUS_INSTANCES;

float4x4 PreviousSkinningMatrix(SkinnedMotionVectorVertexInput input) {
  float4x4 result = EE_PREVIOUS_ANIM_BONES[input.bone_ids.x] * input.weights.x;
  if (input.bone_ids.y != -1) result += EE_PREVIOUS_ANIM_BONES[input.bone_ids.y] * input.weights.y;
  if (input.bone_ids.z != -1) result += EE_PREVIOUS_ANIM_BONES[input.bone_ids.z] * input.weights.z;
  if (input.bone_ids.w != -1) result += EE_PREVIOUS_ANIM_BONES[input.bone_ids.w] * input.weights.w;
  if (input.bone_ids2.x != -1) result += EE_PREVIOUS_ANIM_BONES[input.bone_ids2.x] * input.weights2.x;
  if (input.bone_ids2.y != -1) result += EE_PREVIOUS_ANIM_BONES[input.bone_ids2.y] * input.weights2.y;
  if (input.bone_ids2.z != -1) result += EE_PREVIOUS_ANIM_BONES[input.bone_ids2.z] * input.weights2.z;
  if (input.bone_ids2.w != -1) result += EE_PREVIOUS_ANIM_BONES[input.bone_ids2.w] * input.weights2.w;
  return result;
}

[shader("vertex")]
RasterMotionVectorOutput main(SkinnedMotionVectorVertexInput input, uint draw_id : SV_DrawIndex) {
  RasterMotionVectorOutput output;
  uint instance_index = draw_id + uint(EE_BASIC_CONSTANTS.instance_index);
  float4 current_world = mul(float4(input.position, 1.0f),
                             EE_SKINNING_MATRIX(input.bone_ids, input.weights, input.bone_ids2, input.weights2));
  current_world = mul(current_world, EE_INSTANCES[instance_index].model);
  float4 previous_world = mul(float4(input.position, 1.0f), PreviousSkinningMatrix(input));
  previous_world = mul(previous_world, EE_PREVIOUS_INSTANCES[instance_index].previous_model);
  output.tex_coord0 = input.tex_coord0;
  output.tex_coord1 = input.tex_coord1;
  output.color = input.color;
  output.current_clip = mul(current_world, EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].unjittered_projection_view);
  output.previous_clip =
      mul(previous_world, EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].previous_unjittered_projection_view);
  output.instance_index = instance_index;
  output.previous_pose_valid = EE_PREVIOUS_INSTANCES[instance_index].flags.y;
  output.position = mul(current_world, EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].projection_view);
  return output;
}
