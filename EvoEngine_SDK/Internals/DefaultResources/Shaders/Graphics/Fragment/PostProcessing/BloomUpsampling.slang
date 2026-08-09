// @evoengine-dialect native
struct BloomUpsamplingInput {
  [[vk::location(0)]]
  float2 tex_coord : TEXCOORD0;
};

struct BloomUpsamplingConstants {
  float filter_radius;
};

[[vk::push_constant]]
ConstantBuffer<BloomUpsamplingConstants> constants;

[[vk::binding(0, 1)]]
Sampler2D srcTexture;

[shader("fragment")]
float4 main(BloomUpsamplingInput input) : SV_Target {
  float x = constants.filter_radius;
  float y = constants.filter_radius;
  float2 uv = input.tex_coord;
  float4 a = srcTexture.Sample(float2(uv.x - x, uv.y + y));
  float4 b = srcTexture.Sample(float2(uv.x, uv.y + y));
  float4 c = srcTexture.Sample(float2(uv.x + x, uv.y + y));
  float4 d = srcTexture.Sample(float2(uv.x - x, uv.y));
  float4 e = srcTexture.Sample(uv);
  float4 f = srcTexture.Sample(float2(uv.x + x, uv.y));
  float4 g = srcTexture.Sample(float2(uv.x - x, uv.y - y));
  float4 h = srcTexture.Sample(float2(uv.x, uv.y - y));
  float4 i = srcTexture.Sample(float2(uv.x + x, uv.y - y));
  return (e * 4.0f + (b + d + f + h) * 2.0f + a + c + g + i) * (1.0f / 16.0f);
}
