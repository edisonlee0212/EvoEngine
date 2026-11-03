#ifdef EE_TREE_DEBUG
import EcoSysLab.TreeDebug;
#else
import EvoEngine.PerFrame;
import EvoEngine.RasterVertex;
#endif
import EcoSysLab.GpuTreeMesh;
struct TreeRasterVertex {
  float4 position:SV_Position;
#ifdef EE_TREE_DEBUG
  [[vk::location(0)]] float4 color:TEXCOORD0;
  [[vk::location(1)]] nointerpolation uint2 node:TEXCOORD1;
  [[vk::location(2)]] float length_factor:TEXCOORD2;
  [[vk::location(3)]] nointerpolation float selected_factor:TEXCOORD3;
#else
  [[vk::location(0)]] float3 normal:TEXCOORD0;
  [[vk::location(1)]] float3 tangent:TEXCOORD1;
  [[vk::location(2)]] float2 uv:TEXCOORD2;
  [[vk::location(3)]] nointerpolation uint instance_index:TEXCOORD3;
  [[vk::location(4)]] nointerpolation uint material_index:TEXCOORD4;
#endif
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
#ifdef EE_TREE_DEBUG
  float4x4 model=tree_debug_state[0].model;
  float4x4 projection=tree_debug_state[0].projection_view;
#else
  uint instance=tree_instance_indices[tree_node.links.y].branch;
  float4x4 model=EE_INSTANCES[instance].model;
  float4x4 projection=EE_CAMERAS[camera_index].projection_view;
#endif
  if(lane<vertex_count) {
    uint key=TreeMeshletVertexKey(task,range,bounds,lane);
    TreeRingVertex ring=TreeKeyVertex(task,key);
    uint index=lane;
    vertices[index].position=mul(mul(float4(ring.position,1),model),projection);
#ifdef EE_TREE_DEBUG
    vertices[index].color=tree_debug_colors[task.node_index];
    vertices[index].node=uint2(tree_node.foliage_meta.x+1,tree_node.foliage_meta.y);
    uint ring_index=key/257;
    float t0,t1;
    TreeSpanParameters(task,ring_index==0 ? 0 : ring_index-1,t0,t1);
    vertices[index].length_factor=ring_index==0 ? t0 : t1;
    TreeDebugState debug=tree_debug_state[0];
    bool shoot=tree_node.foliage_meta.y!=0;
    bool selected=tree_node.foliage_meta.x==(shoot ? debug.selected.x : debug.selected.y);
    vertices[index].selected_factor=selected ? (shoot ? debug.selection.x : debug.selection.y) : -1;
#else
    vertices[index].normal=EE_RASTER_WORLD_NORMAL(model,ring.normal);
    vertices[index].tangent=normalize(mul(ring.tangent,float3x3(model)));
    vertices[index].uv=ring.uv;
    vertices[index].instance_index=instance;
    vertices[index].material_index=tree_node.links.z;
#endif
  }
  if(lane<range.y) {
    uint3 keys=TreeTriangleKeys(task,range.x+lane);
    triangles[lane]=uint3(TreeMeshletVertexIndex(task,range,bounds,keys.x),
                         TreeMeshletVertexIndex(task,range,bounds,keys.z),
                         TreeMeshletVertexIndex(task,range,bounds,keys.y));
  }
}
