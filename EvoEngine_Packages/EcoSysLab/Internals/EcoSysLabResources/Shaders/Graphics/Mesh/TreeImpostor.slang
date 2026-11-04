#ifdef EE_TREE_DEBUG
import EcoSysLab.TreeDebug;
#else
import EvoEngine.PerFrame;
#endif
import EcoSysLab.TreeImpostor;
[outputtopology("triangle")]
[numthreads(4,1,1)]
[shader("mesh")]
void main(in payload TreeBatchPayload batch,uint3 group_id:SV_GroupID,uint lane:SV_GroupIndex,
          OutputVertices<TreeImpostorOutput,4> vertices,OutputIndices<uint3,2> triangles) {
  uint entry=0;
  while(group_id.x>=batch.ends[entry]) ++entry;
  uint index=batch.nodes[entry].node_index;
  GpuTreeNode node=tree_nodes[index];
#ifdef EE_TREE_DEBUG
  float4x4 model=tree_debug_state[0].model;
  float4x4 projection=tree_debug_state[0].projection_view;
  float4x4 inverse_projection=tree_debug_state[0].inverse_projection_view;
#else
  float4x4 model=EE_INSTANCES[tree_instance_indices[node.links.y].branch].model;
  float4x4 projection=EE_CAMERAS[camera_index].projection_view;
  float4x4 inverse_projection=EE_CAMERAS[camera_index].inverse_projection_view;
#endif
  float4 points[4]; uint count=TreeShadowPoints(node,points);
  float2 low,high; float radius;
  if(!TreeImpostorBounds(points,count,mul(model,projection),tessellation.xy,low,high,radius)) {
    SetMeshOutputCounts(0,0); return;
  }
  low=max(low,float2(-1)); high=min(high,float2(1));
  float2 xy=float2((lane&1)!=0 ? high.x : low.x,(lane&2)!=0 ? high.y : low.y);
  float3 x=model[0].xyz,y=model[1].xyz,z=model[2].xyz;
  if(abs(dot(x,cross(y,z)))<1e-12) { SetMeshOutputCounts(0,0); return; }
  SetMeshOutputCounts(4,2);
  float4 near_world=mul(float4(xy,0,1),inverse_projection);
  float4 far_world=mul(float4(xy,0.5,1),inverse_projection);
  vertices[lane].position=float4(xy,0,1);
  vertices[lane].near_position=TreeShadowLocalVector(near_world.xyz/near_world.w-model[3].xyz,x,y,z);
  vertices[lane].far_position=TreeShadowLocalVector(far_world.xyz/far_world.w-model[3].xyz,x,y,z);
  vertices[lane].node_index=index;
  if(lane==0) triangles[0]=uint3(0,1,2);
  if(lane==1) triangles[1]=uint3(2,1,3);
}
