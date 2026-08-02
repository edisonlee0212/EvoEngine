// @evoengine-dialect native
import EvoEngine.PostProcessing;

struct BloomDownsamplingInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

[[vk::push_constant]]
ConstantBuffer<EeBloomDownsamplingPushConstant> constants;

[[vk::binding(0, 1)]]
Sampler2D srcTexture;

float3 PowVec3(float3 value, float power) {
  return float3(pow(value.x, power), pow(value.y, power), pow(value.z, power));
}

float KarisAverage(float3 color) {
  float3 srgb = PowVec3(color, 1.0f / 2.2f);
  return 1.0f / (1.0f + dot(srgb, float3(0.299f, 0.587f, 0.114f)) * 0.25f);
}

[shader("fragment")]
float4 main(BloomDownsamplingInput input) : SV_Target {
  float2 texel = 1.0f / constants.source_resolution;
  float x = texel.x;
  float y = texel.y;
  float2 uv = input.tex_coord;

  float3 a = srcTexture.Sample(float2(uv.x - 2.0f * x, uv.y + 2.0f * y)).rgb;
  float3 b = srcTexture.Sample(float2(uv.x, uv.y + 2.0f * y)).rgb;
  float3 c = srcTexture.Sample(float2(uv.x + 2.0f * x, uv.y + 2.0f * y)).rgb;
  float3 d = srcTexture.Sample(float2(uv.x - 2.0f * x, uv.y)).rgb;
  float3 e = srcTexture.Sample(uv).rgb;
  float3 f = srcTexture.Sample(float2(uv.x + 2.0f * x, uv.y)).rgb;
  float3 g = srcTexture.Sample(float2(uv.x - 2.0f * x, uv.y - 2.0f * y)).rgb;
  float3 h = srcTexture.Sample(float2(uv.x, uv.y - 2.0f * y)).rgb;
  float3 i = srcTexture.Sample(float2(uv.x + 2.0f * x, uv.y - 2.0f * y)).rgb;
  float3 j = srcTexture.Sample(float2(uv.x - x, uv.y + y)).rgb;
  float3 k = srcTexture.Sample(float2(uv.x + x, uv.y + y)).rgb;
  float3 l = srcTexture.Sample(float2(uv.x - x, uv.y - y)).rgb;
  float3 m = srcTexture.Sample(float2(uv.x + x, uv.y - y)).rgb;

  float3 result = e * 0.125f + (a + c + g + i) * 0.03125f + (b + d + f + h) * 0.0625f +
                  (j + k + l + m) * 0.125f;
  if (constants.mip_level == 0) {
    float3 groups[5];
    groups[0] = (a + b + d + e) * (0.125f / 4.0f);
    groups[1] = (b + c + e + f) * (0.125f / 4.0f);
    groups[2] = (d + e + g + h) * (0.125f / 4.0f);
    groups[3] = (e + f + h + i) * (0.125f / 4.0f);
    groups[4] = (j + k + l + m) * (0.5f / 4.0f);
    for (int group = 0; group < 5; ++group) groups[group] *= KarisAverage(groups[group]);
    result = max(groups[0] + groups[1] + groups[2] + groups[3] + groups[4], 0.0001f);
  }
  return float4(result, 1.0f);
}
