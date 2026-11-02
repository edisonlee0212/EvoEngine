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
}
