import EcoSysLab.GpuTreeMesh;

struct FoliageTaskPayload {
  uint node_index;
  uint leaf_count;
};

[numthreads(1, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID) {
  uint index = node_base + group_id.x;
  GpuTreeNode node = tree_nodes[index];
  GpuTreeFoliageParams params = tree_foliage_params[node.links.y];
  FoliageTaskPayload payload;
  payload.node_index = index;
  payload.leaf_count = 0u;
  if (node.links.w == 0xffffffffu || node.foliage_meta.y == 0u || params.flags.x != geometry_pass ||
      node.direction_length.w <= 1e-6 || dot(node.direction_length.xyz, node.direction_length.xyz) <= 1e-8 ||
      node.foliage.z > params.filters.x || node.radial_axis_distance.w < params.filters.y ||
      node.foliage.y > params.filters.z) {
    DispatchMesh(0u, 1u, 1u, payload);
    return;
  }
  payload.leaf_count = min(65535u * 16u, uint(max(0.0, ceil(params.size_count.w * node.foliage.x))));
  DispatchMesh((payload.leaf_count + 15u) / 16u, 1u, 1u, payload);
}
