import EvoEngine.PerFrame;
import EvoEngine.RasterVertex;
import EcoSysLab.GpuTreeMesh;
struct TreeRasterVertex {
  float4 position:SV_Position;
  [[vk::location(0)]] float3 normal:TEXCOORD0;
  [[vk::location(1)]] float3 tangent:TEXCOORD1;
  [[vk::location(2)]] float2 uv:TEXCOORD2;
  [[vk::location(3)]] nointerpolation uint instance_index:TEXCOORD3;
  [[vk::location(4)]] nointerpolation uint material_index:TEXCOORD4;
};
[outputtopology("triangle")]
[numthreads(32,1,1)]
[shader("mesh")]
void main(in payload TreeBatchPayload batch,uint3 group_id:SV_GroupID,uint lane:SV_GroupIndex,
          OutputVertices<TreeRasterVertex,22> vertices,OutputIndices<uint3,20> triangles) {
  uint node=0;
  while(group_id.x>=batch.ends[node]) ++node;
  TreeTaskPayload task=batch.nodes[node];
  uint group=group_id.x-(node==0 ? 0 : batch.ends[node-1]);
  uint4 range=TreeMeshletRange(task,group);
  uint4 bounds=TreeMeshletBounds(task,range);
  uint vertex_count=TreeMeshletVertexCount(task,range,bounds);
  SetMeshOutputCounts(vertex_count,range.y);
  GpuTreeNode tree_node=tree_nodes[task.node_index];
  uint instance=tree_instance_indices[tree_node.links.y].branch;
  float4x4 model=EE_INSTANCES[instance].model;
  if(lane<vertex_count) {
    TreeRingVertex ring=TreeKeyVertex(task,TreeMeshletVertexKey(task,range,bounds,lane));
    uint index=lane;
    vertices[index].position=mul(mul(float4(ring.position,1),model),EE_CAMERAS[camera_index].projection_view);
    vertices[index].normal=EE_RASTER_WORLD_NORMAL(model,ring.normal);
    vertices[index].tangent=normalize(mul(ring.tangent,float3x3(model)));
    vertices[index].uv=ring.uv;
    vertices[index].instance_index=instance;
    vertices[index].material_index=tree_node.links.z;
  }
  if(lane<range.y) {
    uint3 keys=TreeTriangleKeys(task,range.x+lane);
    triangles[lane]=uint3(TreeMeshletVertexIndex(task,range,bounds,keys.x),
                         TreeMeshletVertexIndex(task,range,bounds,keys.z),
                         TreeMeshletVertexIndex(task,range,bounds,keys.y));
  }
}
