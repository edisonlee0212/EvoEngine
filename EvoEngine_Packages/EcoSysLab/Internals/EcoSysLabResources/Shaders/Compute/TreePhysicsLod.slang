import EcoSysLab.TreePhysicsData;
import EcoSysLab.TreePhysicsLodData;
struct TreeParameters {
  float4 mechanics;float4 gravity;uint4 control;
  float4x4 previous_model;float4x4 current_model;float4x4 model_delta;
  float4 anchor_previous;float4 anchor_current;float4 rotation_delta;float4 anchor_offset;float4 material;
};
[[vk::binding(0,0)]] StructuredBuffer<TreePhysicsBuffers,Std430DataLayout> full_trees;
[[vk::binding(1,0)]] RWStructuredBuffer<TreeParameters,Std430DataLayout> parameters;
[[vk::binding(2,0)]] StructuredBuffer<uint> active_ids;
[[vk::binding(3,0)]] RWStructuredBuffer<uint2> vertex_groups;
[[vk::binding(4,0)]] RWStructuredBuffer<uint2> rod_groups;
[[vk::binding(5,0)]] RWStructuredBuffer<uint2> coefficient_groups;
[[vk::binding(6,0)]] RWStructuredBuffer<uint4> error_groups;
[[vk::binding(7,0)]] RWStructuredBuffer<uint> dispatch_args;
[[vk::binding(8,0)]] RWStructuredBuffer<TreePhysicsBuffers,Std430DataLayout> selected_trees;
[[vk::binding(9,0)]] StructuredBuffer<uint2> full_vertex_groups;
[[vk::binding(10,0)]] StructuredBuffer<uint2> full_rod_groups;
[[vk::push_constant]] cbuffer Push {
  uint vertex_copy;uint rod_copy;uint tree_count;uint padding;
};
void Setup(uint id) {
  if (full_trees[id].reserved==0) return;
  TreeLodHeader header=PhysicsLoad<TreeLodHeader>(full_trees[id].reserved,0);
  float tolerance=max(0.0,parameters[id].anchor_offset.w);
  uint tick=PhysicsLoad<uint>(header.state,4)+1;
  bool requested=PhysicsLoad<uint>(header.state,2)!=asuint(tolerance);
  bool check=requested || (tolerance>0 && tick%32==0);
  PhysicsStore<uint>(header.state,3,check?(requested?1u:2u):0u);
  PhysicsStore<uint>(header.state,4,tick);
  if (!check || tolerance<=0) return;
  uint count=(max(header.full.counts.x,header.full.counts.y)+63)/64;
  for (uint level=1;level<header.counts.x;++level) {
    TreeLodLevel data=PhysicsLoad<TreeLodLevel>(header.levels,level);
    bool eligible=max(data.metrics.x,data.metrics.z)<=tolerance;
    PhysicsStore<uint>(header.state,8+level,eligible?0u:0x7f800000u);
    if (!eligible) continue;
    uint base;InterlockedAdd(dispatch_args[12],count,base);
    InterlockedMax(dispatch_args[9],min(base+count,65535u));
    for (uint group=0;group<count;++group) error_groups[base+group]=uint4(id,level,group*64,0);
  }
}
void Check(uint3 group,uint lane) {
  // Keep indirect dispatch within Vulkan's guaranteed X limit, even for large forests.
  for (uint job=group.x;job<dispatch_args[12];job+=65535u) {
  uint4 work=error_groups[job];uint i=work.z+lane;
  if (full_trees[work.x].reserved==0) return;
  TreeLodHeader header=PhysicsLoad<TreeLodHeader>(full_trees[work.x].reserved,0);
  TreeLodLevel level=PhysicsLoad<TreeLodLevel>(header.levels,work.y);
  float error=0;
  if (i<header.full.counts.x) {
    TreePhysicsVertex v=PhysicsVertex(header.full,0,i);
    error=length(LodCandidatePosition(header,level,i)-v.position_inverse_mass.xyz)/max(v.velocity_radius.w,1e-6);
  }
  if (i<header.full.counts.y) {
    TreeLodRodMap map=PhysicsLoad<TreeLodRodMap>(level.rod_map,i);
    float4 expected=normalize(LodMultiply(LodCandidateRotation(header,level,map.links.x),map.relative_rotation));
    if (abs(dot(expected,PhysicsRod(header.full,0,i).rotation))<cos(level.metrics.y*0.5)) error=1e30;
  }
  if (!isfinite(error)) error=1e30;
  vk::BufferPointer<PhysicsValue<uint>,4> target=bit_cast<vk::BufferPointer<PhysicsValue<uint>,4>>(header.state+uint64_t(8+work.y)*4);
  InterlockedMax(target.Get().value,asuint(error));
  }
}
void Select(uint id) {
  TreePhysicsBuffers full=full_trees[id];
  TreePhysicsBuffers working=full;
  if (full.reserved!=0) {
  TreeLodHeader header=PhysicsLoad<TreeLodHeader>(full.reserved,0);
  uint previous=PhysicsLoad<uint>(header.state,0),selected=previous;
  uint check=PhysicsLoad<uint>(header.state,3);
  float tolerance=max(0.0,parameters[id].anchor_offset.w);
  if (tolerance<=0) selected=0;
  else if (check!=0) {
    selected=0;
    for (uint level=1;level<header.counts.x;++level) {
      TreeLodLevel data=PhysicsLoad<TreeLodLevel>(header.levels,level);
      float bound=tolerance*((check==2 && level>previous)?0.8:1.0);
      if (max(data.metrics.x,data.metrics.z)<=bound && asfloat(PhysicsLoad<uint>(header.state,8+level))<=bound) selected=level;
    }
  }
  PhysicsStore<uint>(header.state,0,selected);
  PhysicsStore<uint>(header.state,1,previous);
  PhysicsStore<uint>(header.state,2,asuint(tolerance));
  PhysicsStore<uint>(header.state,5,selected!=previous?1u:0u);
  if (selected!=0) {
    working=PhysicsLoad<TreeLodLevel>(header.levels,selected).rest;
    working.vertices[0]=header.scratch.vertices[0];working.vertices[1]=header.scratch.vertices[1];
    working.rods[0]=header.scratch.rods[0];working.rods[1]=header.scratch.rods[1];
    working.predicted=header.scratch.predicted;working.previous=header.scratch.previous;
    working.vertex_coefficients=header.scratch.vertex_coefficients;
    working.rod_coefficients=header.scratch.rod_coefficients;working.angle_coefficients=header.scratch.angle_coefficients;
    working.counts.w=3;working.reserved=full.reserved;
  }
  if (selected!=previous) parameters[id].control.w|=2u;
  }
  selected_trees[id]=working;
  uint count=(working.counts.x+63)/64,base;
  InterlockedAdd(dispatch_args[0],count,base);
  for (uint g=0;g<count;++g) vertex_groups[base+g]=uint2(id,g*64);
  count=(working.counts.y+63)/64;
  InterlockedAdd(dispatch_args[3],count,base);
  for (uint g=0;g<count;++g) rod_groups[base+g]=uint2(id,g*64);
  count=(max(max(working.counts.x,working.counts.y),working.counts.z)+63)/64;
  InterlockedAdd(dispatch_args[6],count,base);
  for (uint g=0;g<count;++g) coefficient_groups[base+g]=uint2(id,g*64);
}
void Transfer(uint3 group,uint lane) {
  uint2 work=coefficient_groups[group.x];uint i=work.y+lane;
  if (full_trees[work.x].reserved==0) return;
  TreeLodHeader header=PhysicsLoad<TreeLodHeader>(full_trees[work.x].reserved,0);
  if (PhysicsLoad<uint>(header.state,5)==0) return;
  uint selected=PhysicsLoad<uint>(header.state,0);
  TreeLodLevel level=PhysicsLoad<TreeLodLevel>(header.levels,selected);
  TreePhysicsBuffers working=selected_trees[work.x];
  if (i<working.counts.x) {
    TreePhysicsVertex v=selected==0?PhysicsVertex(header.full,0,i):PhysicsVertex(level.rest,0,i);
    if (selected!=0) {
      uint original=PhysicsLoad<uint>(level.original_vertices,i);
      v.position_inverse_mass.xyz=PhysicsVertex(header.full,0,original).position_inverse_mass.xyz;
      uint2 range=PhysicsLoad<uint2>(level.velocity_ranges,i);float3 velocity=0;
      for (uint j=0;j<range.y;++j) {
        TreeLodVelocityWeight weight=PhysicsLoad<TreeLodVelocityWeight>(level.velocity_weights,range.x+j);
        velocity+=weight.weight*PhysicsVertex(header.full,0,weight.vertex).velocity_radius.xyz;
      }
      v.velocity_radius.xyz=v.position_inverse_mass.w>0?velocity:float3(0,0,0);
    }
    LodStoreVertex(working,0,i,v,selected!=0);LodStoreVertex(working,1,i,v,selected!=0);
  }
  if (i<working.counts.y) {
    TreePhysicsRod r=selected==0?PhysicsRod(header.full,0,i):PhysicsRod(level.rest,0,i);
    if (selected!=0) r.rotation=LodCandidateRotation(header,level,i);
    LodStoreRod(working,0,i,r,selected!=0);LodStoreRod(working,1,i,r,selected!=0);
  }
}
void ReconstructVertex(uint3 group,uint lane) {
  uint2 work=full_vertex_groups[group.x];uint i=work.y+lane;
  if (full_trees[work.x].reserved==0) return;
  TreeLodHeader header=PhysicsLoad<TreeLodHeader>(full_trees[work.x].reserved,0);
  uint selected=PhysicsLoad<uint>(header.state,0);
  if (selected==0 || i>=header.full.counts.x) return;
  TreeLodLevel level=PhysicsLoad<TreeLodLevel>(header.levels,selected);
  TreePhysicsVertex v=PhysicsVertex(header.full,0,i);
  float3 position,velocity;
  LodMappedMotion(selected_trees[work.x],level,i,vertex_copy,rod_copy,position,velocity);
  TreeParameters tree=parameters[work.x];
  v.velocity_radius.xyz=v.position_inverse_mass.w>0 && tree.control.z!=0?
      (position-v.position_inverse_mass.xyz)/(tree.mechanics.x*float(tree.control.x)):float3(0,0,0);
  v.position_inverse_mass.xyz=position;
  LodStoreVertex(header.full,vertex_copy,i,v,false);
}
void ReconstructRod(uint3 group,uint lane) {
  uint2 work=full_rod_groups[group.x];uint i=work.y+lane;
  if (full_trees[work.x].reserved==0) return;
  TreeLodHeader header=PhysicsLoad<TreeLodHeader>(full_trees[work.x].reserved,0);
  uint selected=PhysicsLoad<uint>(header.state,0);
  if (selected==0 || i>=header.full.counts.y) return;
  TreeLodLevel level=PhysicsLoad<TreeLodLevel>(header.levels,selected);
  TreeLodRodMap map=PhysicsLoad<TreeLodRodMap>(level.rod_map,i);
  TreePhysicsRod r=PhysicsRod(header.full,0,i);
  r.rotation=normalize(LodMultiply(PhysicsRod(selected_trees[work.x],rod_copy,map.links.x).rotation,map.relative_rotation));
  LodStoreRod(header.full,rod_copy,i,r,false);
}
[numthreads(64,1,1)]
void main(uint3 group:SV_GroupID,uint lane:SV_GroupIndex,uint3 thread:SV_DispatchThreadID) {
#if TREE_LOD_PASS == 0
  if (thread.x==0) for (uint i=0;i<13;++i) dispatch_args[i]=(i%3==0)?0u:1u;
#elif TREE_LOD_PASS == 1
  if (thread.x<tree_count) Setup(active_ids[thread.x]);
#elif TREE_LOD_PASS == 2
  Check(group,lane);
#elif TREE_LOD_PASS == 3
  if (thread.x<tree_count) Select(active_ids[thread.x]);
#elif TREE_LOD_PASS == 4
  Transfer(group,lane);
#elif TREE_LOD_PASS == 5
  ReconstructVertex(group,lane);
#elif TREE_LOD_PASS == 6
  ReconstructRod(group,lane);
#endif
}
