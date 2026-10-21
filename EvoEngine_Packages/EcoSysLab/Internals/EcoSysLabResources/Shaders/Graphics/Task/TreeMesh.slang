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
  if (node.links.z == 0xffffffffu) {
    DispatchMesh(0u, 1u, 1u, payload);
    return;
  }
  payload.radial = TreeRadialCount(node);
  payload.start_radial = node.links.x == 0xffffffffu ? payload.radial : TreeRadialCount(tree_nodes[node.links.x]);
  uint requested_rings = max(1u, uint(ceil(node.direction_length.w / node.subdivisions.y)));
  // Twenty triangles per mesh workgroup fit the 64-vertex and 40-primitive limits.
  payload.rings = min(requested_rings, max(1u, (65535u * 20u - payload.start_radial + payload.radial) /
                                                   (2u * payload.radial)));
  uint triangle_count = payload.start_radial + payload.radial +
                        (payload.rings - 1u) * 2u * payload.radial;
  DispatchMesh((triangle_count + 19u) / 20u, 1, 1, payload);
}
