import EcoSysLab.TreeImpostor;
[[vk::binding(0,0)]] RWStructuredBuffer<float4> output;
[numthreads(1,1,1)]
void main() {
  float4x4 identity=float4x4(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1);
  float4 points[4];
  for(uint i=0;i<4;++i) points[i]=float4(0,0,0.5,0.01);
  float2 low,high; float radius;
  bool visible=TreeImpostorBounds(points,4,identity,float2(100),low,high,radius);
  output[0]=float4(visible,radius,low.x,high.y);
  points[0].z=0;
  output[1]=float4(TreeImpostorBounds(points,4,identity,float2(100),low,high,radius),0,0,0);
  float4 a=float4(0,0,0,0.2),b=float4(0,2,0,0.1);
  float3 origin=float3(0,1,-2),direction=float3(0,0,1);
  float hit=TreeShadowIntersect(origin,direction,a,b);
  output[2]=float4(TreeImpostorNormal(origin,direction,hit,a,b),hit);
  origin=float3(0,-2,0);direction=float3(0,1,0);
  hit=TreeShadowIntersect(origin,direction,a,b);
  output[3]=float4(TreeImpostorNormal(origin,direction,hit,a,b),hit);
  output[4]=float4(TreeUseImpostor(0,identity,float2(100),3),TreeUseImpostor(0,identity,float2(100),0),
                   TreeUseImpostor(0,identity,float2(100),0.5),TreeUseImpostor(1,identity,float2(100),3));
  output[5]=float4(TreeUseImpostor(2,identity,float2(100),3),TreeUseImpostor(3,identity,float2(100),3),0,0);
  // Every point on transformed spheres must lie within the conservative projected quad.
  float4x4 transform=float4x4(-2,0,0,0,0.2,1,0,0,0,0,1,1,0,0,1,1);
  points[0]=float4(0.2,0.1,2,0.01); points[1]=float4(-0.1,0.2,3,0.02);
  visible=TreeImpostorBounds(points,2,transform,float2(1920,1080),low,high,radius);
  float violation=0;
  for(uint j=0;j<2;++j) for(uint i=0;i<128;++i) {
    float z=1-2*(i+0.5)/128.0,angle=i*2.399963;
    float3 n=float3(sqrt(1-z*z)*cos(angle),sqrt(1-z*z)*sin(angle),z);
    float4 c=mul(float4(points[j].xyz+n*points[j].w,1),transform);
    float2 xy=c.xy/c.w;
    violation=max(violation,max(max(low.x-xy.x,low.y-xy.y),max(xy.x-high.x,xy.y-high.y)));
  }
  output[6]=float4(visible,violation,radius,0);
}
