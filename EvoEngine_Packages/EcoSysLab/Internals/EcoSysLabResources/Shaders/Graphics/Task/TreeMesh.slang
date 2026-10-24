import EcoSysLab.GpuTreeMesh;

struct TreeTaskPayload {
  uint node_index;
  uint rings;
  uint start_radial;
  uint radial;
};

[numthreads(1, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID) {
  uint node_index = node_base + group_id.x;
  GpuTreeNode node = tree_nodes[node_index];
  TreeTaskPayload payload;
  payload.node_index = node_index;
  if (node.links.z == 0xffffffffu || node.direction_length.w <= 1e-6 ||
      dot(node.direction_length.xyz, node.direction_length.xyz) <= 1e-8) {
    DispatchMesh(0u, 1u, 1u, payload);
    return;
  }
  payload.radial = TreeRadialCount(node);
  payload.start_radial = payload.radial;
  if (node.links.x != 0xffffffffu) {
    GpuTreeNode parent = tree_nodes[node.links.x];
    if (parent.links.z != 0xffffffffu && parent.direction_length.w > 1e-6)
      payload.start_radial = TreeRadialCount(parent);
  }
  uint requested_rings = max(1u, uint(ceil(node.direction_length.w / node.subdivisions.y)));
  // Twenty triangles per mesh workgroup fit the 64-vertex and 40-primitive limits.
  payload.rings = min(requested_rings, max(1u, (65535u * 20u - payload.start_radial + payload.radial) /
                                                   (2u * payload.radial)));
  uint triangle_count = payload.start_radial + payload.radial +
                        (payload.rings - 1u) * 2u * payload.radial;
  DispatchMesh((triangle_count + 19u) / 20u, 1, 1, payload);
}
