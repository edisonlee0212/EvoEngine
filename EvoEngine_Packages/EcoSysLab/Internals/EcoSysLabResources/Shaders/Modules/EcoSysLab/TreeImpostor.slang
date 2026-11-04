__exported import EcoSysLab.TreeShadow;
import EcoSysLab.TreeTessellation;

// Conservative projected endpoint-sphere bounds, also used for the raster quad.
public bool TreeImpostorBounds(float4 points[4], uint count, float4x4 transform,
                               float2 resolution, out float2 low, out float2 high, out float radius) {
  low = float2(1e30); high = float2(-1e30); radius = 0;
  for (uint i = 0; i < count; ++i) {
    float4 c = mul(float4(points[i].xyz,1),transform);
    float4 e = points[i].w * (abs(transform[0])+abs(transform[1])+abs(transform[2]));
    if (c.w <= e.w+1e-5 || c.z <= e.z) return false;
    float2 r = (e.xy+abs(c.xy/c.w)*e.w)/(c.w-e.w);
    low = min(low,c.xy/c.w-r); high = max(high,c.xy/c.w+r);
    radius = max(radius,max(r.x*resolution.x,r.y*resolution.y)*0.5);
  }
  return true;
}

public bool TreeUseImpostor(uint index, float4x4 transform, float2 resolution, float threshold) {
  if (threshold <= 0) return false;
  GpuTreeNode node=tree_nodes[index];
  float4 points[4];
  uint count=TreeShadowPoints(node,points);
  float2 low,high; float radius;
  if (!TreeImpostorBounds(points,count,transform,resolution,low,high,radius) || 2*radius>threshold) return false;
  // Keep visibly curved branches as meshes rather than replacing them with coarse chords.
  float3 p0,p1,p2,p3;
  TreeControlPoints(node,p0,p1,p2,p3);
  float4 c0=mul(float4(p0,1),transform),c1=mul(float4(p1,1),transform);
  float4 c2=mul(float4(p2,1),transform),c3=mul(float4(p3,1),transform);
  return TreeScreenRingCount(c0,c1,c2,c3,resolution,0.25,1e30,8u)<=2;
}

public struct TreeImpostorOutput {
  public float4 position : SV_Position;
  [[vk::location(0)]] public noperspective float3 near_position : TEXCOORD0;
  [[vk::location(1)]] public noperspective float3 far_position : TEXCOORD1;
  [[vk::location(2)]] public nointerpolation uint node_index : TEXCOORD2;
};

public float3 TreeImpostorNormal(float3 origin,float3 direction,float hit,float4 a,float4 b) {
  float3 p=origin+direction*hit;
  if(abs(TreeShadowSphere(origin,direction,a.xyz,a.w)-hit)<1e-5) return normalize(p-a.xyz);
  if(abs(TreeShadowSphere(origin,direction,b.xyz,b.w)-hit)<1e-5) return normalize(p-b.xyz);
  float3 axis=b.xyz-a.xyz;
  float length_axis=length(axis);
  axis/=max(length_axis,1e-8);
  float h=dot(p-a.xyz,axis);
  return normalize(p-a.xyz-axis*h-axis*((b.w-a.w)/max(length_axis,1e-8))*(a.w+(b.w-a.w)*h/max(length_axis,1e-8)));
}
public float TreeImpostorLengthFactor(GpuTreeNode node,uint count,uint segment,float along) {
  bool parent_valid=false;
  if(node.links.x!=0xffffffffu) {
    GpuTreeNode parent=tree_nodes[node.links.x];
    parent_valid=parent.links.z!=0xffffffffu && parent.direction_length.w>1e-6;
  }
  float first=parent_valid ? 1.0/max(1.0,ceil(node.direction_length.w/node.subdivisions.y)) : (count==3 ? 0.5 : 1.0);
  float times[4]={0,first,count==4 ? (first+1)*0.5 : 1.0,1.0};
  return lerp(times[segment-1],times[segment],along);
}
