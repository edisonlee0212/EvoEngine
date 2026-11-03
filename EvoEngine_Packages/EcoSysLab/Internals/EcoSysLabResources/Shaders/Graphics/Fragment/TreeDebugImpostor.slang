import EcoSysLab.TreeDebug;
import EcoSysLab.TreeImpostor;
struct TreeDebugImpostorOutput {
  float4 color : SV_Target0;
  uint4 pick : SV_Target1;
};
[shader("fragment")]
TreeDebugImpostorOutput main(TreeImpostorOutput input,out float depth:SV_Depth) {
  GpuTreeNode node=tree_nodes[input.node_index];
  TreeDebugState debug=tree_debug_state[0];
  float3 origin=input.near_position,direction=normalize(input.far_position-origin);
  float4 points[4]; uint count=TreeShadowPoints(node,points);
  float hit=1e30; uint segment=1;
  for(uint i=1;i<count;++i) {
    float candidate=TreeShadowIntersect(origin,direction,points[i-1],points[i]);
    if(candidate<hit) { hit=candidate; segment=i; }
  }
  if(hit>=1e29) discard;
  float3 p=origin+direction*hit;
  float4 clip=mul(mul(float4(p,1),debug.model),debug.projection_view);
  depth=clip.z/clip.w;
  if(depth<0 || depth>1) discard;
  float3 edge=points[segment].xyz-points[segment-1].xyz;
  float along=saturate(dot(p-points[segment-1].xyz,edge)/max(dot(edge,edge),1e-12));
  float factor=TreeImpostorLengthFactor(node,count,segment,along);
  bool shoot=node.foliage_meta.y!=0;
  bool selected=node.foliage_meta.x==(shoot ? debug.selected.x : debug.selected.y);
  TreeDebugImpostorOutput result;
  result.color=tree_debug_colors[input.node_index];
  if(selected) {
    result.color.rgb=lerp(result.color.rgb,float3(1,0.8,0.05),0.65);
    if(abs(factor-(shoot ? debug.selection.x : debug.selection.y))<0.05) result.color.rgb=float3(1);
  }
  result.pick=uint4(node.foliage_meta.x+1,node.foliage_meta.y,asuint(saturate(factor)),0);
  return result;
}
