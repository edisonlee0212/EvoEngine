// Voxel pass adapted from Godot scene_forward_clustered.glsl::MODE_RENDER_SDF at
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
import EvoEngine.SdfgiVoxel;
[[vk::binding(0, 1)]] ConstantBuffer<SdfgiVoxelData> voxel;
[[vk::push_constant]] ConstantBuffer<SdfgiVoxelPushConstant> draw;

struct SdfgiVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
  [[vk::location(1)]] float3 normal : NORMAL;
  [[vk::location(3)]] float2 uv0 : TEXCOORD0;
  [[vk::location(4)]] float4 color : COLOR0;
  [[vk::location(10)]] float2 uv1 : TEXCOORD1;
};

[shader("vertex")]
SdfgiVoxelVertex main(SdfgiVertexInput input) {
  float4 world = draw.model[0] * input.position.x + draw.model[1] * input.position.y +
                 draw.model[2] * input.position.z + draw.model[3];
  SdfgiVoxelVertex output;
  output.position = voxel.view_projection[0] * world.x + voxel.view_projection[1] * world.y +
                    voxel.view_projection[2] * world.z + voxel.view_projection[3];
  output.world_position = world.xyz;
  output.normal = normalize(float3(draw.normal_basis[0], draw.normal_basis[1], draw.normal_basis[2]) * input.normal.x +
                            float3(draw.normal_basis[3], draw.normal_basis[4], draw.normal_basis[5]) * input.normal.y +
                            float3(draw.normal_basis[6], draw.normal_basis[7], draw.normal_basis[8]) * input.normal.z);
  output.color = input.color;
  output.uv0 = input.uv0;
  output.uv1 = input.uv1;
  return output;
}
