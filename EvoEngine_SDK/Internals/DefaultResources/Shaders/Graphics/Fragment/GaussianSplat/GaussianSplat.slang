struct GaussianSplatFragmentInput {
  [[vk::location(0)]]
  float3 color : COLOR0;
  [[vk::location(1)]]
  float2 local_coord : TEXCOORD0;
  [[vk::location(2)]]
  nointerpolation float opacity : TEXCOORD1;
  [[vk::location(3)]]
  nointerpolation float cutoff_radius_squared : TEXCOORD2;
};

[shader("fragment")]
float4 main(GaussianSplatFragmentInput input) : SV_Target {
  float radius_squared = dot(input.local_coord, input.local_coord);
  if (radius_squared > input.cutoff_radius_squared) discard;
  float alpha = input.opacity * exp(-0.5f * radius_squared);
  if (alpha <= 1.0f / 255.0f) discard;
  return float4(input.color * alpha, alpha);
}
