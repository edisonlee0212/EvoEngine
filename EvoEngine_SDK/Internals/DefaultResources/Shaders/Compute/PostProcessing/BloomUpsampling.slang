#include "PostProcessing.slangh"

[[vk::push_constant]]
ConstantBuffer<EeBloomUpsamplingPushConstant> constants;

[[vk::binding(0, 0)]]
Sampler2D srcTexture;

[[vk::binding(1, 0)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outTexture;

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  if (!EePixelInBounds(pixel, constants.target_resolution)) {
    return;
  }

  float2 tex_coord = (float2(pixel) + float2(0.5f)) / float2(constants.target_resolution);
  float x = constants.filter_radius;
  float y = constants.filter_radius;

  float4 a = srcTexture.SampleLevel(float2(tex_coord.x - x, tex_coord.y + y), 0.0f);
  float4 b = srcTexture.SampleLevel(float2(tex_coord.x, tex_coord.y + y), 0.0f);
  float4 c = srcTexture.SampleLevel(float2(tex_coord.x + x, tex_coord.y + y), 0.0f);

  float4 d = srcTexture.SampleLevel(float2(tex_coord.x - x, tex_coord.y), 0.0f);
  float4 e = srcTexture.SampleLevel(float2(tex_coord.x, tex_coord.y), 0.0f);
  float4 f = srcTexture.SampleLevel(float2(tex_coord.x + x, tex_coord.y), 0.0f);

  float4 g = srcTexture.SampleLevel(float2(tex_coord.x - x, tex_coord.y - y), 0.0f);
  float4 h = srcTexture.SampleLevel(float2(tex_coord.x, tex_coord.y - y), 0.0f);
  float4 i = srcTexture.SampleLevel(float2(tex_coord.x + x, tex_coord.y - y), 0.0f);

  float4 upsample = e * 4.0f;
  upsample += (b + d + f + h) * 2.0f;
  upsample += a + c + g + i;
  upsample *= 1.0f / 16.0f;
  outTexture[EeTexel(pixel)] = upsample;
}
