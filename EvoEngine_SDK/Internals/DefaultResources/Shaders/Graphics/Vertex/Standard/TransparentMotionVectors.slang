import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.RasterMotionVectors;

struct MotionVectorVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
  [[vk::location(3)]] float2 tex_coord0 : TEXCOORD0;
  [[vk::location(4)]] float4 color : COLOR0;
  [[vk::location(10)]] float2 tex_coord1 : TEXCOORD1;
  [[vk::location(11)]] float2 tex_coord2 : TEXCOORD2;
  [[vk::location(12)]] float2 tex_coord3 : TEXCOORD3;
};

[[vk::binding(1, 2)]]
StructuredBuffer<PreviousInstance> EE_PREVIOUS_INSTANCES;

[shader("vertex")]
RasterMotionVectorOutput main(MotionVectorVertexInput input, uint draw_id : SV_DrawIndex) {
  RasterMotionVectorOutput output;
  uint instance_index = draw_id + uint(EE_BASIC_CONSTANTS.instance_index);
  float4 current_world = mul(float4(input.position, 1.0f), EE_INSTANCES[instance_index].model);
  float4 previous_world = mul(float4(input.position, 1.0f), EE_PREVIOUS_INSTANCES[instance_index].previous_model);
  output.tex_coord0 = input.tex_coord0;
  output.tex_coord1 = input.tex_coord1;
  output.tex_coord2 = input.tex_coord2;
  output.tex_coord3 = input.tex_coord3;
  output.color = input.color;
  output.current_clip = mul(current_world, EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].unjittered_projection_view);
  output.previous_clip =
      mul(previous_world, EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].previous_unjittered_projection_view);
  output.instance_index = instance_index;
  output.previous_pose_valid = EE_PREVIOUS_INSTANCES[instance_index].flags.x;
  output.position = mul(current_world, EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].projection_view);
  return output;
}
