import EcoSysLab.TreePhysicsData;

public struct TreeLodVertexMap { public uint4 links; public float4 offset_t; };
public struct TreeLodRodMap { public uint4 links; public float4 relative_rotation; };
public struct TreeLodVelocityWeight { public uint vertex; public float weight; };
public struct TreeLodLevel {
  public TreePhysicsBuffers rest;
  public uint64_t vertex_map;
  public uint64_t rod_map;
  public uint64_t original_vertices;
  public uint64_t velocity_ranges;
  public uint64_t velocity_weights;
  public uint64_t members;
  public float4 metrics;
};
public struct TreeLodHeader {
  public TreePhysicsBuffers full;
  public TreePhysicsBuffers scratch;
  public uint64_t levels;
  public uint64_t state;
  public uint4 counts;
};
public float4 LodMultiply(float4 a,float4 b) {
  return float4(a.w*b.xyz+b.w*a.xyz+cross(a.xyz,b.xyz),a.w*b.w-dot(a.xyz,b.xyz));
}
public float4 LodInverse(float4 q) { return float4(-q.xyz,q.w); }
public float3 LodRotate(float4 q,float3 v) {
  float3 t=2*cross(q.xyz,v);return v+q.w*t+cross(q.xyz,t);
}
public float4 LodCandidateRotation(TreeLodHeader header,TreeLodLevel level,uint rod_id) {
  TreePhysicsRod rod=PhysicsRod(level.rest,0,rod_id);
  uint first=PhysicsLoad<uint>(level.members,rod.incident_angles.z);
  TreeLodRodMap mapping=PhysicsLoad<TreeLodRodMap>(level.rod_map,first);
  return normalize(LodMultiply(PhysicsRod(header.full,0,first).rotation,LodInverse(mapping.relative_rotation)));
}
public float3 LodCandidatePosition(TreeLodHeader header,TreeLodLevel level,uint vertex) {
  TreeLodVertexMap map=PhysicsLoad<TreeLodVertexMap>(level.vertex_map,vertex);
  if (map.links.y!=0xffffffffu)
    return PhysicsVertex(header.full,0,vertex).position_inverse_mass.xyz;
  TreePhysicsRod rod=PhysicsRod(level.rest,0,map.links.x);
  uint a=PhysicsLoad<uint>(level.original_vertices,rod.links.x),b=PhysicsLoad<uint>(level.original_vertices,rod.links.y);
  return lerp(PhysicsVertex(header.full,0,a).position_inverse_mass.xyz,
              PhysicsVertex(header.full,0,b).position_inverse_mass.xyz,map.offset_t.w)+
         LodRotate(LodCandidateRotation(header,level,map.links.x),map.offset_t.xyz);
}
public void LodMappedMotion(TreePhysicsBuffers working,TreeLodLevel level,uint vertex,uint vertex_copy,uint rod_copy,
                            out float3 position,out float3 velocity) {
  TreeLodVertexMap map=PhysicsLoad<TreeLodVertexMap>(level.vertex_map,vertex);
  if (map.links.y!=0xffffffffu) {
    TreePhysicsVertex v=PhysicsVertex(working,vertex_copy,map.links.y);
    position=v.position_inverse_mass.xyz;velocity=v.velocity_radius.xyz;return;
  }
  TreePhysicsRod rod=PhysicsRod(working,rod_copy,map.links.x);
  TreePhysicsVertex a=PhysicsVertex(working,vertex_copy,rod.links.x),b=PhysicsVertex(working,vertex_copy,rod.links.y);
  position=lerp(a.position_inverse_mass.xyz,b.position_inverse_mass.xyz,map.offset_t.w)+LodRotate(rod.rotation,map.offset_t.xyz);
  velocity=lerp(a.velocity_radius.xyz,b.velocity_radius.xyz,map.offset_t.w);
}
public float LodVertexWeight(TreeLodLevel level,uint original_vertex,uint coarse_vertex,uint coarse_rod) {
  TreeLodVertexMap map=PhysicsLoad<TreeLodVertexMap>(level.vertex_map,original_vertex);
  if (map.links.y!=0xffffffffu) return map.links.y==coarse_vertex?1.0:0.0;
  TreePhysicsRod rod=PhysicsRod(level.rest,0,coarse_rod);
  return coarse_vertex==rod.links.x?1.0-map.offset_t.w:map.offset_t.w;
}
public void LodStoreVertex(TreePhysicsBuffers buffers,uint copy,uint i,TreePhysicsVertex v,bool metadata) {
  bool arrays=buffers.counts.w!=0;uint n=buffers.counts.x;
  PhysicsStore<float4>(buffers.vertices[copy],PhysicsAttributeIndex(i,0,n,4,arrays),v.position_inverse_mass);
  PhysicsStore<float4>(buffers.vertices[copy],PhysicsAttributeIndex(i,1,n,4,arrays),v.velocity_radius);
  if (metadata) {
    PhysicsStore<float4>(buffers.vertices[copy],PhysicsAttributeIndex(i,2,n,4,arrays),v.rest_local);
    PhysicsStore<uint4>(buffers.vertices[copy],PhysicsAttributeIndex(i,3,n,4,arrays),v.incident_rods);
  }
}
public void LodStoreRod(TreePhysicsBuffers buffers,uint copy,uint i,TreePhysicsRod r,bool metadata) {
  bool arrays=buffers.counts.w!=0;uint n=buffers.counts.y;
  PhysicsStore<float4>(buffers.rods[copy],PhysicsAttributeIndex(i,2,n,5,arrays),r.rotation);
  if (metadata) {
    PhysicsStore<uint4>(buffers.rods[copy],PhysicsAttributeIndex(i,0,n,5,arrays),r.links);
    PhysicsStore<float4>(buffers.rods[copy],PhysicsAttributeIndex(i,1,n,5,arrays),r.length_scale_mass_radius);
    PhysicsStore<float4>(buffers.rods[copy],PhysicsAttributeIndex(i,3,n,5,arrays),r.rest_rotation);
    PhysicsStore<uint4>(buffers.rods[copy],PhysicsAttributeIndex(i,4,n,5,arrays),r.incident_angles);
  }
}
