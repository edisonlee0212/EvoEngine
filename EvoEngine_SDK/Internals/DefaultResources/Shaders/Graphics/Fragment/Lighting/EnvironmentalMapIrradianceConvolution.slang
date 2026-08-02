// @evoengine-dialect native
struct CubemapFragmentInput {
  [[vk::location(0)]]
  float3 world_pos : TEXCOORD0;
};

[[vk::binding(0, 0)]]
SamplerCube environmentMap;

static const float PI = 3.14159265359f;

[shader("fragment")]
float4 main(CubemapFragmentInput input) : SV_Target {
  float3 n = normalize(input.world_pos);
  float3 irradiance = float3(0.0f);
  float3 up = float3(0.0f, 1.0f, 0.0f);
  float3 right = normalize(cross(up, n));
  up = normalize(cross(n, right));

  float sample_delta = 0.025f;
  float sample_count = 0.0f;
  for (float phi = 0.0f; phi < 2.0f * PI; phi += sample_delta) {
    for (float theta = 0.0f; theta < 0.5f * PI; theta += sample_delta) {
      float3 tangent_sample = float3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
      float3 sample_vector = tangent_sample.x * right + tangent_sample.y * up + tangent_sample.z * n;
      irradiance += environmentMap.Sample(sample_vector).rgb * cos(theta) * sin(theta);
      sample_count++;
    }
  }
  irradiance = PI * irradiance * (1.0f / sample_count);
  return float4(irradiance, 1.0f);
}
