import EcoSysLab.TreeShadow;
[[vk::binding(0, 0)]] RWStructuredBuffer<float> values;
[numthreads(1,1,1)]
void main() {
  float4 a = float4(0,0,0,0.2), b = float4(0,2,0,0.2);
  values[0] = TreeShadowIntersect(float3(0,1,-2), float3(0,0,1), a, b);
  values[1] = TreeShadowIntersect(float3(0,-2,0), float3(0,1,0), a, b);
  values[2] = TreeShadowIntersect(float3(0.3,1,-2), float3(0,0,1), a, b);
  values[3] = TreeShadowIntersect(float3(0,0,-2), float3(0,0,1), a, a);
  b.w = 0.1;
  values[4] = TreeShadowIntersect(float3(0,1,-2), float3(0,0,1), a, b);
  values[5] = TreeShadowIntersect(float3(0.2,1,-2), float3(0,0,1), a, b);
  values[6] = TreeShadowIntersect(float3(0.1,1,-2), float3(0,0,1), a, b);
  values[8] = TreeShadowIntersect(float3(0,1,0), float3(0,0,1), a, b);
  float3 x = float3(0,2,0), y = float3(-3,0,0), z = float3(0.5,0,4);
  float3 local_origin = TreeShadowLocalVector(y - 2 * z, x,y,z);
  float3 local_ray = normalize(TreeShadowLocalVector(normalize(z), x,y,z));
  values[9] = TreeShadowIntersect(local_origin, local_ray, a, b);
  values[7] = TreeShadowIntersect(float3(0,4,0), float3(0,-1,0), a, b);

  float4x4 identity=float4x4(1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
  float4 points[4];
  for (uint j=0;j<4;++j) points[j]=float4(0,0,0.5,0.1);
  values[10]=TreeShadowVisible(points,4,identity);
  for (uint plane=0;plane<6;++plane) {
    float3 center=float3(0,0,0.5);
    center[plane/2]=(plane&1)==0 ? -2.0 : 2.0;
    for(uint j=0;j<4;++j) points[j]=float4(center,0.1);
    values[11+plane]=TreeShadowVisible(points,4,identity);
  }
  // Spheres straddling the near/far planes and a segment crossing the whole frustum remain visible.
  for(uint j=0;j<4;++j) points[j]=float4(0,0,-0.05,0.1);
  values[17]=TreeShadowVisible(points,4,identity);
  for(uint j=0;j<4;++j) points[j]=float4(0,0,1.05,0.1);
  values[18]=TreeShadowVisible(points,4,identity);
  points[0]=float4(-2,0,0.5,0.1);points[1]=float4(2,0,0.5,0.1);
  values[19]=TreeShadowVisible(points,2,identity);
  // Mirrored, nonuniform scale: transformed sphere reaches the frustum even though its center does not.
  float4x4 transform=float4x4(-4,0,0,0, 0,2,0,0, 0,0,1,0, 0,0,0,1);
  points[0]=float4(0.3,0,0.5,0.1);
  values[20]=TreeShadowVisible(points,1,transform);
  points[0].x=0.5;
  values[21]=TreeShadowVisible(points,1,transform);
}
