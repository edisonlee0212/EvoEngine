import EcoSysLab.TreeCanopy;
groupshared float3 minima[64];
groupshared float3 maxima[64];
[numthreads(64,1,1)]
void main(uint3 group:SV_GroupID,uint lane:SV_GroupIndex) {
  uint4 range=tree_shadow_ranges[group.x];
  float3 low=float3(1e30),high=float3(-1e30);
  for(uint i=lane;i<range.y;i+=64) {
    uint index=tree_shadow_members[range.x+i];
    if(tree_nodes[index].links.z==0xffffffffu || tree_nodes[index].direction_length.w<=1e-6 ||
       dot(tree_nodes[index].direction_length.xyz,tree_nodes[index].direction_length.xyz)<=1e-8) continue;
    float3 a,b;TreeNodeBounds(index,a,b);
    low=min(low,a);high=max(high,b);
  }
  minima[lane]=low;maxima[lane]=high;
  GroupMemoryBarrierWithGroupSync();
  for(uint stride=32;stride>0;stride/=2) {
    if(lane<stride) { minima[lane]=min(minima[lane],minima[lane+stride]);maxima[lane]=max(maxima[lane],maxima[lane+stride]); }
    GroupMemoryBarrierWithGroupSync();
  }
  if(lane==0) {
    TreeCanopyBound bound=(TreeCanopyBound)0;
    if(all(minima[0]<=maxima[0])) {
      bound.center=float4((minima[0]+maxima[0])*0.5,1);
      // The ellipsoid encloses the complete box, including bent branches and attachments.
      bound.radii=float4(max((maxima[0]-minima[0])*0.866025404,float3(1e-5)),0);
    }
    tree_canopy_bounds[group.x]=bound;
  }
}
