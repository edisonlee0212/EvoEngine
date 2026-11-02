import EcoSysLab.GpuTreeMesh;
struct TreeShadowPayload { uint node_index; };
[numthreads(1, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID) {
  TreeShadowPayload payload;
  payload.node_index = node_base + group_id.x;
  GpuTreeNode node = tree_nodes[payload.node_index];
  bool valid = node.links.z != 0xffffffffu && node.direction_length.w > 1e-6 &&
               dot(node.direction_length.xyz, node.direction_length.xyz) > 1e-8;
  DispatchMesh(valid ? 1u : 0u, 1u, 1u, payload);
}
