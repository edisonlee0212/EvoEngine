import EvoEngine.PerFrame;
import EcoSysLab.TreeCanopy;
#ifdef EE_TREE_SHADOW_REFERENCE
#define SHADOW_BATCH 1
#else
#define SHADOW_BATCH 32
#endif
groupshared TreeShadowPayload payload;
groupshared uint visible_count;
[numthreads(SHADOW_BATCH, 1, 1)]
[shader("amplification")]
void main(uint3 group_id : SV_GroupID, uint lane : SV_GroupIndex) {
#ifdef EE_TREE_SHADOW_REFERENCE
  TreeShadowPayload reference;
  reference.node_indices[0] = node_base + group_id.x;
  GpuTreeNode node = tree_nodes[reference.node_indices[0]];
  bool valid = node.links.z != 0xffffffffu && node.direction_length.w > 1e-6 &&
               dot(node.direction_length.xyz,node.direction_length.xyz) > 1e-8;
  DispatchMesh(valid ? 1u : 0u,1u,1u,reference);
#else
  if (lane == 0) visible_count = 0;
  GroupMemoryBarrierWithGroupSync();
  uint index = node_base + group_id.x * SHADOW_BATCH + lane;
  bool valid = false;
  uint output_index=index;
  if (index < node_count) {
    GpuTreeNode node = tree_nodes[index];
    valid = node.links.z != 0xffffffffu && node.direction_length.w > 1e-6 &&
            dot(node.direction_length.xyz,node.direction_length.xyz) > 1e-8;
    uint instance = tree_instance_indices[node.links.y].branch;
    float4x4 transform=mul(EE_INSTANCES[instance].model,
                          EE_DIRECTIONAL_LIGHTS[light_index].light_space_matrix[light_face]);
    uint group=0xffffffffu;
    if(impostor.x>0 && light_face>0) group=tree_shadow_groups[index];
    bool canopy=false;
    if(group!=0xffffffffu && impostor.x>0 && light_face>0) {
      TreeCanopyBound bound=tree_canopy_bounds[group];
      canopy=TreeUseCanopy(bound,transform,impostor.yz,impostor.x,light_face);
      if(canopy) {
        valid=index==tree_shadow_ranges[group].z &&
              TreeBoundsVisible(bound.center.xyz-bound.radii.xyz,bound.center.xyz+bound.radii.xyz,transform);
        output_index=group|0x80000000u;
      }
    }
    if(valid && !canopy) {
      float4 points[4];
      uint count=TreeShadowPoints(node,points);
      valid=TreeShadowVisible(points,count,transform);
    }
  }
  if (valid) {
    uint slot; InterlockedAdd(visible_count,1,slot);
    payload.node_indices[slot] = output_index;
  }
  GroupMemoryBarrierWithGroupSync();
  DispatchMesh(visible_count,1u,1u,payload);
#endif
}
