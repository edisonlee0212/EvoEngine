import EvoEngine.PerFrame;
import EcoSysLab.TreeCanopy;
[shader("fragment")]
float main(TreeShadowOutput input) : SV_Depth {
  float4 points[4] = {input.p0, input.p1, input.p2, input.p3};
  float hit = 1e30;
  if(input.instance_count.y==0) hit=TreeCanopyIntersect(input.origin,input.direction,input.p0.xyz,input.p1.xyz);
  for (uint i = 1; i < input.instance_count.y; ++i)
    hit = min(hit, TreeShadowIntersect(input.origin, input.direction, points[i - 1], points[i]));
  if (hit >= 1e29) discard;
  float3 position = input.origin + input.direction * hit;
  float4 clip = mul(mul(float4(position, 1), EE_INSTANCES[input.instance_count.x].model),
                    EE_DIRECTIONAL_LIGHTS[light_index].light_space_matrix[light_face]);
  float depth = clip.z / clip.w;
  if (depth < 0 || depth > 1) discard;
  return depth;
}
