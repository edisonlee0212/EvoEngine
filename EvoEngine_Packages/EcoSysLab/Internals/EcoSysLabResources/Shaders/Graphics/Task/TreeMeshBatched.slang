import EcoSysLab.TreeImpostor;
#ifdef EE_TREE_DEBUG
import EcoSysLab.TreeDebug;
#else
import EvoEngine.Cameras;
import EvoEngine.Instances;
#endif
groupshared TreeBatchPayload payload;
[numthreads(32,1,1)]
[shader("amplification")]
void main(uint3 group_id:SV_GroupID,uint lane:SV_GroupIndex) {
  uint index=node_base+group_id.x*padding0+lane;
  TreeTaskPayload task=(TreeTaskPayload)0;
  if(lane<padding0 && index<node_count) {
    GpuTreeNode node=tree_nodes[index];
    if(node.links.z!=0xffffffffu && node.direction_length.w>1e-6 && dot(node.direction_length.xyz,node.direction_length.xyz)>1e-8) {
#ifdef EE_TREE_DEBUG
      float4x4 model=tree_debug_state[0].model;
      float4x4 projection=tree_debug_state[0].projection_view;
      float4x4 unjittered=projection;
#else
      uint instance=tree_instance_indices[node.links.y].branch;
      float4x4 model=EE_INSTANCES[instance].model;
      float4x4 projection=EE_CAMERAS[camera_index].projection_view;
      float4x4 unjittered=EE_CAMERAS[camera_index].unjittered_projection_view;
#endif
      if(TreeNodeVisible(index,mul(model,projection))) {
        bool proxy=TreeUseImpostor(index,mul(model,unjittered),tessellation.xy,impostor.x);
        if(proxy==(impostor.y>0)) {
          if(proxy) { task.node_index=index; task.rings=1; }
          else task=TreeBuildTask(index,mul(model,unjittered),tessellation);
        }
      }
    }
  }
  payload.nodes[lane]=task;
  payload.ends[lane]=task.rings==0 ? 0 : (impostor.y>0 ? 1 : TreeMeshletCount(task));
  GroupMemoryBarrierWithGroupSync();
  if(lane==0) for(uint i=1;i<32;++i) payload.ends[i]+=payload.ends[i-1];
  GroupMemoryBarrierWithGroupSync();
  DispatchMesh(payload.ends[31],1u,1u,payload);
}
