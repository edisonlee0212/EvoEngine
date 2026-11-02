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
}
