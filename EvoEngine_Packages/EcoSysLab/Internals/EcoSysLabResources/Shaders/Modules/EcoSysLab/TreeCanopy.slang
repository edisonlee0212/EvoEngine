__exported import EcoSysLab.TreeShadow;
public struct TreeCanopyBound { public float4 center; public float4 radii; };
[[vk::binding(5,3)]] public StructuredBuffer<uint> tree_shadow_groups;
[[vk::binding(6,3)]] public StructuredBuffer<uint4> tree_shadow_ranges;
[[vk::binding(7,3)]] public StructuredBuffer<uint> tree_shadow_members;
[[vk::binding(8,3)]] public RWStructuredBuffer<TreeCanopyBound> tree_canopy_bounds;

public bool TreeUseCanopy(TreeCanopyBound bound,float4x4 transform,float2 resolution,float threshold,uint cascade) {
  if(cascade==0 || threshold<=0 || bound.center.w==0) return false;
  float4 center=mul(float4(bound.center.xyz,1),transform);
  float4 extent=abs(transform[0])*bound.radii.x+abs(transform[1])*bound.radii.y+abs(transform[2])*bound.radii.z;
  if(center.w<=extent.w+1e-5) return false;
  float2 diameter=(extent.xy+abs(center.xy/center.w)*extent.w)/(center.w-extent.w)*resolution;
  return max(diameter.x,diameter.y)<=threshold;
}
public float TreeCanopyIntersect(float3 origin,float3 direction,float3 center,float3 radii) {
  float3 d=direction/radii;
  float scale=length(d);
  float3 o=(origin-center)/radii;
  if(dot(o,o)<=1) return 0;
  float hit=TreeShadowSphere(o,d/scale,float3(0),1);
  return hit>=1e29 ? 1e30 : hit/scale;
}
