#ifdef EE_TREE_DEBUG
import EcoSysLab.TreeDebug;
#else
import EvoEngine.PerFrame;
import EvoEngine.RasterVertex;
#endif
import EcoSysLab.GpuTreeMesh;

struct TreeTaskPayload {
  uint node_index;
  uint rings;
  uint start_radial;
  uint radial;
};

struct TreeRasterVertex {
  float4 position : SV_Position;
#ifdef EE_TREE_DEBUG
  [[vk::location(0)]] float4 color : TEXCOORD0;
  [[vk::location(1)]] nointerpolation uint2 node : TEXCOORD1;
  [[vk::location(2)]] float length_factor : TEXCOORD2;
  [[vk::location(3)]] nointerpolation float selected_factor : TEXCOORD3;
#else
  [[vk::location(0)]] float3 normal : TEXCOORD0;
  [[vk::location(1)]] float3 tangent : TEXCOORD1;
  [[vk::location(2)]] float2 uv : TEXCOORD2;
  [[vk::location(3)]] nointerpolation uint instance_index : TEXCOORD3;
  [[vk::location(4)]] nointerpolation uint material_index : TEXCOORD4;
#endif
};

[outputtopology("triangle")]
[numthreads(32, 1, 1)]
[shader("mesh")]
void main(in payload TreeTaskPayload task, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<TreeRasterVertex, 60> vertices, OutputIndices<uint3, 20> triangles) {
  uint total_triangles = task.start_radial + task.radial + (task.rings - 1u) * 2u * task.radial;
  uint first_triangle = group_id.x * 20u;
  uint group_triangles = min(20u, total_triangles - first_triangle);
  SetMeshOutputCounts(group_triangles * 3u, group_triangles);
  if (lane_id >= group_triangles) return;
  GpuTreeNode node = tree_nodes[task.node_index];
  uint first_count = task.start_radial + task.radial;
  uint triangle_index = first_triangle + lane_id;
  uint span = triangle_index < first_count ? 0u : 1u + (triangle_index - first_count) / (2u * task.radial);
  uint triangle = triangle_index < first_count ? triangle_index : (triangle_index - first_count) % (2u * task.radial);
  uint start_count = span == 0u ? task.start_radial : task.radial;
  uint3 samples, ring_ids;
  TreeZipper(triangle, start_count, task.radial, samples, ring_ids);
  bool parent_valid = false;
  GpuTreeNode start_node = node;
  if (span == 0u && node.links.x != 0xffffffffu) {
    GpuTreeNode parent = tree_nodes[node.links.x];
    parent_valid = parent.links.z != 0xffffffffu && parent.direction_length.w > 1e-6;
    if (parent_valid) start_node = parent;
  }
  float t0 = span == 0u && parent_valid ? 1.0 : float(span) / float(task.rings);
  float t1 = float(span + 1u) / float(task.rings);
#ifdef EE_TREE_DEBUG
  TreeDebugState debug = tree_debug_state[0];
  float4x4 model = debug.model;
  float4x4 projection = debug.projection_view;
#else
  uint instance_index = tree_instance_indices[node.links.y].branch;
  float4x4 model = EE_INSTANCES[instance_index].model;
  float4x4 projection = EE_CAMERAS[camera_index].projection_view;
  if (pass_index == 1u) projection = EE_POINT_LIGHTS[light_index].light_space_matrix[light_face];
  if (pass_index == 2u) projection = EE_SPOT_LIGHTS[light_index].light_space_matrix;
  if (pass_index == 3u) projection = EE_DIRECTIONAL_LIGHTS[light_index].light_space_matrix[light_face];
#endif
  [unroll]
  for (uint i = 0u; i < 3u; ++i) {
    bool start = ring_ids[i] == 0u;
    GpuTreeNode ring_node = start ? start_node : node;
    uint count = start ? start_count : task.radial;
    TreeRingVertex ring = EvaluateTreeRing(ring_node, start ? t0 : t1, samples[i], count);
    uint vertex_index = lane_id * 3u + i;
    vertices[vertex_index].position = mul(mul(float4(ring.position, 1.0), model), projection);
#ifdef EE_TREE_DEBUG
    float length_factor = float(span + ring_ids[i]) / float(task.rings);
    float4 color = tree_debug_colors[task.node_index];
    bool shoot = node.foliage_meta.y != 0u;
    bool selected = node.foliage_meta.x == (shoot ? debug.selected.x : debug.selected.y);
    vertices[vertex_index].color = color;
    vertices[vertex_index].node = uint2(node.foliage_meta.x + 1u, node.foliage_meta.y);
    vertices[vertex_index].length_factor = length_factor;
    vertices[vertex_index].selected_factor = selected ? (shoot ? debug.selection.x : debug.selection.y) : -1.0;
#else
    vertices[vertex_index].normal = EE_RASTER_WORLD_NORMAL(model, ring.normal);
    vertices[vertex_index].tangent = normalize(mul(ring.tangent, float3x3(model)));
    vertices[vertex_index].uv = ring.uv;
    vertices[vertex_index].instance_index = instance_index;
    vertices[vertex_index].material_index = node.links.z;
#endif
  }
  triangles[lane_id] = uint3(lane_id * 3u, lane_id * 3u + 2u, lane_id * 3u + 1u);
}
