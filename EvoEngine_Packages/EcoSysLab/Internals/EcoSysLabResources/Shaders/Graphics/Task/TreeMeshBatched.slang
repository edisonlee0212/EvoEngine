import EcoSysLab.GpuTreeMesh;
import EvoEngine.Cameras;
import EvoEngine.Instances;
groupshared TreeBatchPayload payload;
[numthreads(32,1,1)]
[shader("amplification")]
void main(uint3 group_id:SV_GroupID,uint lane:SV_GroupIndex) {
  uint index=node_base+group_id.x*padding0+lane;
  TreeTaskPayload task=(TreeTaskPayload)0;
  if(lane<padding0 && index<node_count) {
    GpuTreeNode node=tree_nodes[index];
    if(node.links.z!=0xffffffffu && node.direction_length.w>1e-6 && dot(node.direction_length.xyz,node.direction_length.xyz)>1e-8) {
      uint instance=tree_instance_indices[node.links.y].branch;
      float4x4 model=EE_INSTANCES[instance].model;
      if(TreeNodeVisible(index,mul(model,EE_CAMERAS[camera_index].projection_view)))
        task=TreeBuildTask(index,mul(model,EE_CAMERAS[camera_index].unjittered_projection_view),tessellation);
    }
  }
  payload.nodes[lane]=task;
  payload.ends[lane]=task.rings==0 ? 0 : TreeMeshletCount(task);
  GroupMemoryBarrierWithGroupSync();
  if(lane==0) for(uint i=1;i<32;++i) payload.ends[i]+=payload.ends[i-1];
  GroupMemoryBarrierWithGroupSync();
  DispatchMesh(payload.ends[31],1u,1u,payload);
}
