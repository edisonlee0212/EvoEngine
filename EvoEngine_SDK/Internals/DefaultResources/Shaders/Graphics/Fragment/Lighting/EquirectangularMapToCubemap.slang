// @evoengine-dialect native
struct EeEquirectangularToCubemapConstants {
  float4x4 projection_view;
  float environment_pdf_scale;
};

[[vk::push_constant]]
ConstantBuffer<EeEquirectangularToCubemapConstants> constants;

struct CubemapFragmentInput {
  [[vk::location(0)]]
  float3 world_pos : TEXCOORD0;
};

[[vk::binding(0, 0)]]
Sampler2D equirectangularMap;

static const float PI = 3.14159265359f;
static const float2 invAtan = float2(0.1591f, 0.3183f);

float2 SampleSphericalMap(float3 value) {
  float2 uv = float2(atan2(value.z, value.x), asin(value.y));
  uv *= invAtan;
  uv += 0.5f;
  return uv;
}

[shader("fragment")]
float4 main(CubemapFragmentInput input) : SV_Target {
  float2 uv = SampleSphericalMap(normalize(input.world_pos));
  float3 color = equirectangularMap.Sample(uv).rgb;
  float luminance = dot(max(color, float3(0.0f)), float3(0.2126f, 0.7152f, 0.0722f));
  float pdf = constants.environment_pdf_scale > 0.0f ? luminance * constants.environment_pdf_scale
                                                      : 1.0f / (4.0f * PI);
  return float4(color, pdf);
}
