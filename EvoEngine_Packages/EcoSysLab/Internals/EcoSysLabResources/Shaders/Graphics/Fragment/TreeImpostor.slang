import EvoEngine.PerFrame;
import EvoEngine.GBuffer;
import EvoEngine.RasterVertex;
import EcoSysLab.TreeImpostor;
struct TreeImpostorGBuffer {
  [[vk::location(0)]] float4 uv : SV_Target0;
  [[vk::location(1)]] float4 geometric_frame : SV_Target1;
  [[vk::location(2)]] float4 uv_ddx : SV_Target2;
  [[vk::location(3)]] float4 uv_ddy : SV_Target3;
  [[vk::location(4)]] uint4 metadata : SV_Target4;
};
[shader("fragment")]
TreeImpostorGBuffer main(TreeImpostorOutput input,out float depth:SV_Depth) {
  GpuTreeNode node=tree_nodes[input.node_index];
  uint instance_index=tree_instance_indices[node.links.y].branch;
  Instance instance=EE_INSTANCES[instance_index];
  float3 origin=input.near_position,direction=normalize(input.far_position-origin);
  float4 points[4]; uint count=TreeShadowPoints(node,points);
  float hit=1e30; uint segment=1;
  for(uint i=1;i<count;++i) {
    float candidate=TreeShadowIntersect(origin,direction,points[i-1],points[i]);
    if(candidate<hit) { hit=candidate; segment=i; }
  }
  if(hit>=1e29) discard;
  float3 p=origin+direction*hit;
  float4 clip=mul(mul(float4(p,1),instance.model),EE_CAMERAS[camera_index].projection_view);
  depth=clip.z/clip.w;
  if(depth<0 || depth>1) discard;
  float3 normal=TreeImpostorNormal(origin,direction,hit,points[segment-1],points[segment]);
  float3 edge=points[segment].xyz-points[segment-1].xyz;
  float along=saturate(dot(p-points[segment-1].xyz,edge)/max(dot(edge,edge),1e-12));
  bool parent_valid=false;
  if(node.links.x!=0xffffffffu) {
    GpuTreeNode parent=tree_nodes[node.links.x];
    parent_valid=parent.links.z!=0xffffffffu && parent.direction_length.w>1e-6;
  }
  float first=parent_valid ? 1.0/max(1.0,ceil(node.direction_length.w/node.subdivisions.y)) : (count==3 ? 0.5 : 1.0);
  float t=TreeImpostorLengthFactor(node,count,segment,along);
  float3 center,axis,radial;
  EvaluateTreeFrame(node,t,center,axis,radial);
  float distance=node.radial_axis_distance.w+node.direction_length.w*t;
  if(parent_valid && segment==1) {
    GpuTreeNode parent=tree_nodes[node.links.x];
    float3 parent_center,parent_axis,parent_radial;
    EvaluateTreeFrame(parent,1,parent_center,parent_axis,parent_radial);
    float3 blended_axis=lerp(parent_axis,axis,along);
    if(dot(blended_axis,blended_axis)>1e-10) axis=normalize(blended_axis);
    float3 blended=lerp(parent_radial,radial,along);
    blended-=axis*dot(axis,blended);
    if(dot(blended,blended)>1e-10) radial=normalize(blended);
    distance=lerp(parent.radial_axis_distance.w+parent.direction_length.w,
                  node.radial_axis_distance.w+node.direction_length.w*first,along);
  }
  float angle=atan2(dot(normal,cross(axis,radial)),dot(normal,radial))/6.28318530718;
  float2 uv=float2(frac(angle),distance);
  float3 tangent=cross(axis,normal);
  if(dot(tangent,tangent)<1e-10) tangent=radial;
  TreeImpostorGBuffer result;
  result.uv=float4(uv,uv);
  result.geometric_frame=float4(EE_GBUFFER_OCT_ENCODE(EE_RASTER_WORLD_NORMAL(instance.model,normal)),
                               EE_GBUFFER_OCT_ENCODE(normalize(mul(tangent,float3x3(instance.model)))));
  result.uv_ddx=ddx(result.uv); result.uv_ddy=ddy(result.uv);
  result.metadata=EE_GBUFFER_PACK_METADATA(instance_index,node.links.z,instance.info_index,float4(1),false,
                                          EE_TRANSFORM_HANDEDNESS(instance.model)<0);
  // Angular wrap must not select a coarse mip at the UV seam.
  result.uv_ddx.x-=round(result.uv_ddx.x); result.uv_ddx.z=result.uv_ddx.x;
  result.uv_ddy.x-=round(result.uv_ddy.x); result.uv_ddy.z=result.uv_ddy.x;
  return result;
}
