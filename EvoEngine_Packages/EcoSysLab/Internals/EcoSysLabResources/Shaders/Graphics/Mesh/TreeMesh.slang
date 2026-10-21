import EvoEngine.PerFrame;
import EvoEngine.RasterVertex;
import EcoSysLab.GpuTreeMesh;

struct TreeTaskPayload {
  uint node_index;
  uint rings;
  uint start_radial;
  uint radial;
};

struct TreeRasterVertex {
  float4 position : SV_Position;
  [[vk::location(0)]] float3 normal : TEXCOORD0;
  [[vk::location(1)]] float3 tangent : TEXCOORD1;
  [[vk::location(2)]] float2 uv : TEXCOORD2;
  [[vk::location(3)]] nointerpolation uint instance_index : TEXCOORD3;
  [[vk::location(4)]] nointerpolation uint material_index : TEXCOORD4;
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
  GpuTreeNode start_node = span == 0u && node.links.x != 0xffffffffu ? tree_nodes[node.links.x] : node;
  float t0 = span == 0u && node.links.x != 0xffffffffu ? 1.0 : float(span) / float(task.rings);
  float t1 = float(span + 1u) / float(task.rings);
  uint instance_index = tree_instance_indices[node.links.y].branch;
  float4x4 model = EE_INSTANCES[instance_index].model;
  float4x4 projection = EE_CAMERAS[camera_index].projection_view;
  if (pass_index == 1u) projection = EE_POINT_LIGHTS[light_index].light_space_matrix[light_face];
  if (pass_index == 2u) projection = EE_SPOT_LIGHTS[light_index].light_space_matrix;
  if (pass_index == 3u) projection = EE_DIRECTIONAL_LIGHTS[light_index].light_space_matrix[light_face];
  [unroll]
  for (uint i = 0u; i < 3u; ++i) {
    bool start = ring_ids[i] == 0u;
    GpuTreeNode ring_node = start ? start_node : node;
    uint count = start ? start_count : task.radial;
    TreeRingVertex ring = EvaluateTreeRing(ring_node, start ? t0 : t1, samples[i], count);
    uint vertex_index = lane_id * 3u + i;
    vertices[vertex_index].position = mul(mul(float4(ring.position, 1.0), model), projection);
    vertices[vertex_index].normal = EE_RASTER_WORLD_NORMAL(model, ring.normal);
    vertices[vertex_index].tangent = normalize(mul(ring.tangent, float3x3(model)));
    vertices[vertex_index].uv = ring.uv;
    vertices[vertex_index].instance_index = instance_index;
    vertices[vertex_index].material_index = node.links.z;
  }
  triangles[lane_id] = uint3(lane_id * 3u, lane_id * 3u + 2u, lane_id * 3u + 1u);
}
