import EcoSysLab.TreeCanopy;
[[vk::binding(0,0)]] RWStructuredBuffer<float4> result;
[numthreads(1,1,1)]
void main() {
  TreeCanopyBound bound=tree_canopy_bounds[0];
  result[0]=bound.center;result[1]=bound.radii;
  if(bound.center.w==0) { result[2]=result[3]=0;return; }
  float4x4 identity=float4x4(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1);
  result[2]=float4(TreeUseCanopy(bound,identity,float2(64),128,0),
                   TreeUseCanopy(bound,identity,float2(64),128,1),
                   TreeUseCanopy(bound,identity,float2(64),64,3),
                   TreeCanopyIntersect(bound.center.xyz+float3(5,0,-2),float3(0,0,1),bound.center.xyz,bound.radii.xyz)>1e29);
  float3 low,high;TreeNodeBounds(0,low,high);
  float3 a,b;TreeNodeBounds(1,a,b);low=min(low,a);high=max(high,b);
  result[3]=float4(TreeCanopyIntersect(bound.center.xyz-float3(0,0,2),float3(0,0,1),bound.center.xyz,bound.radii.xyz),
                   length((high-bound.center.xyz)/bound.radii.xyz),
                   TreeCanopyIntersect(bound.center.xyz,float3(0,0,1),bound.center.xyz,bound.radii.xyz),0);
}
