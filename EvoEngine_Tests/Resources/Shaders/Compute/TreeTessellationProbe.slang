import EcoSysLab.GpuTreeMesh;
import EcoSysLab.TreeTessellation;
[[vk::binding(0,0)]] RWStructuredBuffer<float4> values;

float4 Curve(float4 a,float4 b,float4 c,float4 d,float t) {
  float u=1-t;
  return u*u*u*a+3*u*u*t*b+3*u*t*t*c+t*t*t*d;
}

[numthreads(1,1,1)]
void main() {
  float4x4 near_view=float4x4(1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
  float4x4 far_view=near_view; far_view[0][0]=0.02; far_view[1][1]=0.02;
  float4x4 mid_view=near_view; mid_view[0][0]=0.5; mid_view[1][1]=0.5;
  float4 detail=float4(256,256,0.5,12), disabled=float4(256,256,0,12);
  TreeTaskPayload full=TreeBuildTask(0,near_view,disabled);
  TreeTaskPayload close=TreeBuildTask(0,near_view,detail);
  TreeTaskPayload far=TreeBuildTask(0,far_view,detail);
  TreeTaskPayload bark=TreeBuildTask(3,far_view,detail);
  values[0]=float4(full.radial,close.radial,far.radial,bark.radial);
  values[1]=float4(full.rings,close.rings,far.rings,bark.rings);
  float4x4 clipped=near_view; clipped[3][2]=-0.49;
  TreeTaskPayload clip=TreeBuildTask(0,clipped,detail);
  TreeTaskPayload no_view=TreeBuildTask(0,near_view,float4(0,256,0.5,12));
  values[2]=float4(clip.radial,clip.rings,no_view.radial,no_view.rings);
  TreeTaskPayload parent=TreeBuildTask(0,mid_view,detail);
  TreeTaskPayload child=TreeBuildTask(1,mid_view,detail);
  TreeTaskPayload sibling=TreeBuildTask(2,mid_view,detail);
  values[3]=float4(parent.radial,child.start_radial,child.radial,sibling.start_radial);
  float4 difference=0;
  for(uint i=0;i<=parent.radial;++i) {
    TreeRingVertex a=EvaluateTreeRing(tree_nodes[0],1,i,parent.radial);
    TreeRingVertex b=EvaluateTreeRing(tree_nodes[tree_nodes[1].links.x],1,i,child.start_radial);
    difference=max(difference,float4(length(a.position-b.position),length(a.normal-b.normal),
                                    length(a.tangent-b.tangent),length(a.uv-b.uv)));
  }
  values[4]=difference;
  TreeTaskPayload child_full=TreeBuildTask(1,mid_view,disabled);
  float a0,a1,b0,b1,last0,last1;
  TreeSpanParameters(child_full,0,a0,a1);
  TreeSpanParameters(child,0,b0,b1);
  TreeSpanParameters(child,child.rings-1,last0,last1);
  values[5]=float4(a1,b1,last1,child.rings);
  float4 c0=float4(-0.2,-0.1,0.1,1),c1=float4(0.6,1,0.4,2);
  float4 c2=float4(-0.6,-1,0.5,3),c3=float4(0.2,0.1,0.8,4);
  uint count=TreeScreenRingCount(c0,c1,c2,c3,float2(800,600),0.5,12,1024);
  float error=0;
  for(uint i=0;i<=2048;++i) {
    float t=float(i)/2048, span=min(floor(t*count),float(count-1u));
    float4 a=Curve(c0,c1,c2,c3,span/count),b=Curve(c0,c1,c2,c3,(span+1)/count);
    float4 point=Curve(c0,c1,c2,c3,t);
    float2 interpolated=lerp(a.xy/a.w,b.xy/b.w,t*count-span);
    error=max(error,length((point.xy/point.w-interpolated)*float2(400,300)));
  }
  values[6]=float4(error,count,TreeScreenRadialCount(1e30,0.5,126),
                  TreeScreenRingCount(c0,c1,c2,c3,float2(800,600),0.00001,1,3));
  float4x4 perspective=float4x4(1,0,0,0, 0,1,0,0, 0,0,1,1, 0,0,-0.1,0);
  TreeTaskPayload pn=TreeBuildTask(0,perspective,detail);
  float4x4 distant=perspective; distant[3][2]+=100; distant[3][3]+=100;
  TreeTaskPayload pf=TreeBuildTask(0,distant,detail);
  float4x4 scaled=near_view; scaled[0][0]=4; scaled[0][1]=2; scaled[1][1]=0.25;
  TreeTaskPayload affine=TreeBuildTask(0,scaled,detail);
  scaled[0][0]=-4; scaled[0][1]=-2;
  TreeTaskPayload mirrored=TreeBuildTask(0,scaled,detail);
  values[7]=float4(pn.radial,pf.radial,affine.radial,mirrored.radial);
  TreeTaskPayload resolution=TreeBuildTask(0,near_view,float4(512,512,0.5,12));
  values[8]=float4(resolution.radial,resolution.rings,child.full_rings,sibling.radial);
  float triangles_full=full.start_radial+full.radial+(full.rings-1)*2*full.radial;
  float triangles_far=far.start_radial+far.radial+(far.rings-1)*2*far.radial;
  float largest_step=0;
  for(uint span=1;span<child.rings;++span) {
    float previous0,previous1,current0,current1;
    TreeSpanParameters(child,span-1,previous0,previous1);
    TreeSpanParameters(child,span,current0,current1);
    largest_step=max(largest_step,abs(previous1-current0));
  }
  values[9]=float4(triangles_full,triangles_far,largest_step,b0);

  float4 largest_difference=0;
  uint original_vertices=0,unique_vertices=0;
  for(uint test=0;test<8;++test) {
    TreeTaskPayload task=TreeBuildTask(test==0 ? 0 : 1,mid_view,detail);
    task.radial=test==0 ? 4 : test==1 ? 8 : test==2 ? 32 : 256;
    task.start_radial=test==0 ? 4 : test==1 ? 32 : test==2 ? 8 : 126;
    if(test>=4) {
      task.radial=test==4 ? 6 : test==5 ? 10 : test==6 ? 14 : 4;
      task.start_radial=test==7 ? 256 : task.radial;
    }
    task.rings=3; task.full_rings=5;
    uint total=task.start_radial+task.radial+(task.rings-1)*2*task.radial;
    uint next_triangle=0;
    for(uint group=0;group<TreeMeshletCount(task);++group) {
      uint4 range=TreeMeshletRange(task,group),bounds=TreeMeshletBounds(task,range);
      uint first=range.x,count=range.y,keys[60];
      uint vertex_count=TreeMeshletVertexCount(task,range,bounds);
      if(first!=next_triangle || count==0 || count>20 || vertex_count>22) largest_difference=1e30;
      next_triangle=first+count;
      for(uint i=0;i<count;++i) {
        uint triangle_index=first+i;
        uint first_count=task.start_radial+task.radial;
        uint span=triangle_index<first_count ? 0 : 1+(triangle_index-first_count)/(2*task.radial);
        uint triangle=triangle_index<first_count ? triangle_index : (triangle_index-first_count)%(2*task.radial);
        uint3 samples,ring_ids;
        TreeZipper(triangle,span==0 ? task.start_radial : task.radial,task.radial,samples,ring_ids);
        uint3 encoded=TreeTriangleKeys(task,triangle_index);
        float t0,t1;TreeSpanParameters(task,span,t0,t1);
        GpuTreeNode node=tree_nodes[task.node_index];
        for(uint corner=0;corner<3;++corner) {
          bool start=ring_ids[corner]==0;
          bool parent=span==0 && start && node.links.x!=0xffffffffu;
          TreeRingVertex reference=EvaluateTreeRing(parent ? tree_nodes[node.links.x] : node,
              start ? (parent ? 1.0 : t0) : t1,samples[corner],start && span==0 ? task.start_radial : task.radial);
          TreeRingVertex indexed=TreeKeyVertex(task,encoded[corner]);
          if(!all(isfinite(reference.position)) || !all(isfinite(indexed.position))) largest_difference=1e30;
          largest_difference=max(largest_difference,float4(length(reference.position-indexed.position),
              length(reference.normal-indexed.normal),length(reference.tangent-indexed.tangent),length(reference.uv-indexed.uv)));
          keys[3*i+corner]=encoded[corner];
          uint index=TreeMeshletVertexIndex(task,range,bounds,encoded[corner]);
          if(index>=vertex_count || TreeMeshletVertexKey(task,range,bounds,index)!=encoded[corner]) largest_difference=1e30;
        }
      }
      original_vertices+=count*3;
      uint distinct=0;
      for(uint i=0;i<count*3;++i) {
        bool first=true;
        for(uint j=0;j<i;++j) first=first && keys[i]!=keys[j];
        distinct+=first ? 1 : 0;
      }
      if(distinct!=vertex_count) largest_difference=1e30;
      unique_vertices+=vertex_count;
    }
    if(next_triangle!=total) largest_difference=1e30;
  }
  values[10]=largest_difference;
  TreeTaskPayload seam=TreeBuildTask(0,near_view,disabled);
  TreeRingVertex seam0=TreeKeyVertex(seam,0),seam1=TreeKeyVertex(seam,seam.start_radial);
  values[11]=float4(original_vertices,unique_vertices,abs(seam0.uv.x-seam1.uv.x),length(seam0.position-seam1.position));
  float4x4 outside=near_view;outside[3][0]=3;
  values[12]=float4(TreeNodeVisible(0,near_view),TreeNodeVisible(0,outside),TreeNodeVisible(4,near_view),TreeNodeVisible(0,clipped));
  float4x4 bark_edge=near_view;bark_edge[3][0]=1.12;
  values[13]=float4(TreeNodeVisible(0,bark_edge),TreeNodeVisible(3,bark_edge),TreeNodeVisible(0,scaled),TreeNodeVisible(0,perspective));
  uint rejected=0;
  for(uint i=0;i<6;++i) {
    float3 center=float3(0,0,0.5);center[i/2]=(i&1)==0 ? -2.0 : 2.0;
    rejected+=TreeBoundsVisible(center-0.1,center+0.1,near_view) ? 0 : 1;
  }
  values[14]=float4(rejected,TreeBoundsVisible(float3(-0.1,-0.1,-0.1),float3(0.1,0.1,0.1),near_view),
      TreeBoundsVisible(float3(-2,-0.1,0.4),float3(2,0.1,0.6),near_view),
      TreeBoundsVisible(float3(-0.1,-0.1,-2),float3(0.1,0.1,-1),perspective));
}
