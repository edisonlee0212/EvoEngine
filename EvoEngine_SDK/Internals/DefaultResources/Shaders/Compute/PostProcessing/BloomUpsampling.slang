import EvoEngine.PostProcessing;

[[vk::push_constant]]
ConstantBuffer<EeBloomUpsamplingPushConstant> constants;

[[vk::binding(0, 0)]]
Sampler2D currentTexture;

[[vk::binding(1, 0)]]
Sampler2D bloomTexture;

#if defined(EE_BLOOM_STORAGE_FORMAT_RGBA16F)
[vk::image_format("rgba16f")]
#else
[vk::image_format("rgba32f")]
#endif
[[vk::binding(2, 0)]]
RWTexture2D<float4> outTexture;

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  if (!EePixelInBounds(pixel, constants.target_resolution)) {
    return;
  }

  float2 tex_coord = (float2(pixel) + float2(0.5f)) / float2(constants.target_resolution);
  float2 radius = constants.filter_radius / float2(constants.source_resolution);
  float x = radius.x;
  float y = radius.y;

  float3 a = bloomTexture.SampleLevel(float2(tex_coord.x - x, tex_coord.y + y), 0.0f).xyz;
  float3 b = bloomTexture.SampleLevel(float2(tex_coord.x, tex_coord.y + y), 0.0f).xyz;
  float3 c = bloomTexture.SampleLevel(float2(tex_coord.x + x, tex_coord.y + y), 0.0f).xyz;

  float3 d = bloomTexture.SampleLevel(float2(tex_coord.x - x, tex_coord.y), 0.0f).xyz;
  float3 e = bloomTexture.SampleLevel(float2(tex_coord.x, tex_coord.y), 0.0f).xyz;
  float3 f = bloomTexture.SampleLevel(float2(tex_coord.x + x, tex_coord.y), 0.0f).xyz;

  float3 g = bloomTexture.SampleLevel(float2(tex_coord.x - x, tex_coord.y - y), 0.0f).xyz;
  float3 h = bloomTexture.SampleLevel(float2(tex_coord.x, tex_coord.y - y), 0.0f).xyz;
  float3 i = bloomTexture.SampleLevel(float2(tex_coord.x + x, tex_coord.y - y), 0.0f).xyz;

  float3 upsample = e * 4.0f;
  upsample += (b + d + f + h) * 2.0f;
  upsample += a + c + g + i;
  upsample *= 1.0f / 16.0f;
  float3 current = currentTexture.SampleLevel(tex_coord, 0.0f).xyz;
  outTexture[EeTexel(pixel)] = float4(current + upsample, 1.0f);
}
