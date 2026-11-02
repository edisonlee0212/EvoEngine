struct GpuTreeNode {
  float4 position_thickness;
  float4 direction_length;
  float4 radial_axis_distance;
  float4 subdivisions;
  float4 bark;
  float4 bark_base;
  uint4 links;
  float4 foliage;
  float4 foliage_rotation;
  uint4 foliage_meta;
};

import EcoSysLab.TreePhysicsData;

[[vk::binding(0, 0)]] RWStructuredBuffer<GpuTreeNode, Std430DataLayout> tree_nodes;
[[vk::binding(1, 0)]] ByteAddressBuffer vertices;
[[vk::binding(2, 0)]] ByteAddressBuffer rods;
[[vk::binding(3, 0)]] StructuredBuffer<uint> node_rods;
[[vk::binding(4, 0)]] StructuredBuffer<GpuTreeNode, Std430DataLayout> rest_nodes;

[[vk::push_constant]] cbuffer DeformParameters {
  uint node_base;
  uint node_count;
  uint node_rods_base;
  uint attribute_arrays;
  float4 model_position;
  float4 model_scale;
  float4 inverse_rotation;
};

float4 QuatMultiply(float4 a, float4 b) {
  return float4(a.w * b.xyz + b.w * a.xyz + cross(a.xyz, b.xyz), a.w * b.w - dot(a.xyz, b.xyz));
}

float3 QuatRotate(float4 q, float3 v) {
  float3 t = 2.0 * cross(q.xyz, v);
  return v + q.w * t + cross(q.xyz, t);
}

[numthreads(64, 1, 1)]
void main(uint3 thread_id : SV_DispatchThreadID) {
  uint i = thread_id.x;
  if (i >= node_count) return;
  GpuTreeNode node = rest_nodes[node_base + i];
  uint rod_index = node_rods[node_rods_base + node.foliage_meta.x];
  if (rod_index == 0xffffffffu) {
    tree_nodes[node_base + i] = node;
    return;
  }
  uint vertex_bytes, rod_bytes;
  vertices.GetDimensions(vertex_bytes);
  rods.GetDimensions(rod_bytes);
  uint vertex_count = vertex_bytes / 64u, rod_count = rod_bytes / 80u;
  bool arrays = attribute_arrays != 0;
  uint4 links = rods.Load<uint4>(16u * PhysicsAttributeIndex(rod_index,0,rod_count,5,arrays));
  float4 rotation = rods.Load<float4>(16u * PhysicsAttributeIndex(rod_index,2,rod_count,5,arrays));
  float3 start = vertices.Load<float4>(16u * PhysicsAttributeIndex(links.x,0,vertex_count,4,arrays)).xyz;
  float3 end = vertices.Load<float4>(16u * PhysicsAttributeIndex(links.y,0,vertex_count,4,arrays)).xyz;
  float3 local_start = QuatRotate(inverse_rotation, start - model_position.xyz) / model_scale.xyz;
  float3 local_displacement = QuatRotate(inverse_rotation, end - start) / model_scale.xyz;
  float length = length(local_displacement);
  if (length < 1e-6) {
    node.links.z = 0xffffffffu;
    node.links.w = 0xffffffffu;
    tree_nodes[node_base + i] = node;
    return;
  }
  const float4 axis_to_shoot_inverse = float4(0, -0.70710678118, 0, 0.70710678118);
  float4 local_frame = QuatMultiply(inverse_rotation, rotation);
  float4 current_rotation = normalize(QuatMultiply(local_frame, axis_to_shoot_inverse));
  float4 rest_rotation = node.foliage_rotation;
  float4 delta = QuatMultiply(current_rotation, float4(-rest_rotation.xyz, rest_rotation.w));
  node.position_thickness.xyz = local_start;
  node.direction_length = float4(local_displacement / length, length);
  node.radial_axis_distance.xyz = QuatRotate(delta, node.radial_axis_distance.xyz);
  node.foliage_rotation = current_rotation;
  tree_nodes[node_base + i] = node;
}
