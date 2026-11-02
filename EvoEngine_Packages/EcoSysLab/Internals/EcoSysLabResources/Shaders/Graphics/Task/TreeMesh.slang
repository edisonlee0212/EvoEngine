import EcoSysLab.GpuTreeMesh;
#ifdef EE_TREE_DEBUG
import EcoSysLab.TreeDebug;
#else
import EvoEngine.Cameras;
import EvoEngine.Instances;
#endif

[numthreads(1, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID) {
  uint index = node_base + group_id.x;
  GpuTreeNode node = tree_nodes[index];
  if (node.links.z == 0xffffffffu || node.direction_length.w <= 1e-6 ||
      dot(node.direction_length.xyz,node.direction_length.xyz) <= 1e-8) {
    TreeTaskPayload empty = (TreeTaskPayload)0;
    DispatchMesh(0u,1u,1u,empty);
    return;
  }
  float4x4 transform = float4x4(1);
  if (tessellation.z > 0) {
#ifdef EE_TREE_DEBUG
    TreeDebugState debug = tree_debug_state[0];
    transform = mul(debug.model,debug.projection_view);
#else
    uint instance = tree_instance_indices[tree_nodes[index].links.y].branch;
    transform = mul(EE_INSTANCES[instance].model,EE_CAMERAS[camera_index].unjittered_projection_view);
#endif
  }
  TreeTaskPayload payload = TreeBuildTask(index,transform,tessellation);
  if (payload.rings == 0) {
    DispatchMesh(0u,1u,1u,payload);
    return;
  }
  uint triangles = payload.start_radial+payload.radial+(payload.rings-1u)*2u*payload.radial;
  DispatchMesh((triangles+19u)/20u,1u,1u,payload);
}
