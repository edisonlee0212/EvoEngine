import EvoEngine.PostProcessing;

[[vk::push_constant]]
ConstantBuffer<EeBloomDownsamplingPushConstant> constants;

[[vk::binding(0, 0)]]
Sampler2D srcTexture;

[[vk::binding(1, 0)]]
#if defined(EE_BLOOM_STORAGE_FORMAT_RGBA16F)
[vk::image_format("rgba16f")]
#else
[vk::image_format("rgba32f")]
#endif
RWTexture2D<float4> outTexture;

float3 PowVec3(float3 value, float power) {
  return float3(pow(value.x, power), pow(value.y, power), pow(value.z, power));
}

float3 ToSRGB(float3 value) {
  return PowVec3(value, 1.0f / 2.2f);
}

float SrgbToLuma(float3 color) {
  return dot(color, float3(0.299f, 0.587f, 0.114f));
}

float KarisAverage(float3 color) {
  float luma = SrgbToLuma(ToSRGB(color)) * 0.25f;
  return 1.0f / (1.0f + luma);
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  if (!EePixelInBounds(pixel, constants.target_resolution)) {
    return;
  }

  float2 tex_coord = (float2(pixel) + float2(0.5f)) / float2(constants.target_resolution);
  float2 src_texel_size = 1.0f / float2(constants.source_resolution);
  float x = src_texel_size.x;
  float y = src_texel_size.y;

  float3 a = srcTexture.SampleLevel(float2(tex_coord.x - 2.0f * x, tex_coord.y + 2.0f * y), 0.0f).xyz;
  float3 b = srcTexture.SampleLevel(float2(tex_coord.x, tex_coord.y + 2.0f * y), 0.0f).xyz;
  float3 c = srcTexture.SampleLevel(float2(tex_coord.x + 2.0f * x, tex_coord.y + 2.0f * y), 0.0f).xyz;

  float3 d = srcTexture.SampleLevel(float2(tex_coord.x - 2.0f * x, tex_coord.y), 0.0f).xyz;
  float3 e = srcTexture.SampleLevel(float2(tex_coord.x, tex_coord.y), 0.0f).xyz;
  float3 f = srcTexture.SampleLevel(float2(tex_coord.x + 2.0f * x, tex_coord.y), 0.0f).xyz;

  float3 g = srcTexture.SampleLevel(float2(tex_coord.x - 2.0f * x, tex_coord.y - 2.0f * y), 0.0f).xyz;
  float3 h = srcTexture.SampleLevel(float2(tex_coord.x, tex_coord.y - 2.0f * y), 0.0f).xyz;
  float3 i = srcTexture.SampleLevel(float2(tex_coord.x + 2.0f * x, tex_coord.y - 2.0f * y), 0.0f).xyz;

  float3 j = srcTexture.SampleLevel(float2(tex_coord.x - x, tex_coord.y + y), 0.0f).xyz;
  float3 k = srcTexture.SampleLevel(float2(tex_coord.x + x, tex_coord.y + y), 0.0f).xyz;
  float3 l = srcTexture.SampleLevel(float2(tex_coord.x - x, tex_coord.y - y), 0.0f).xyz;
  float3 m = srcTexture.SampleLevel(float2(tex_coord.x + x, tex_coord.y - y), 0.0f).xyz;

  float3 result = e * 0.125f;
  result += (a + c + g + i) * 0.03125f;
  result += (b + d + f + h) * 0.0625f;
  result += (j + k + l + m) * 0.125f;

  if (constants.apply_karis != 0) {
    float3 groups[5];
    groups[0] = (a + b + d + e) * (0.125f / 4.0f);
    groups[1] = (b + c + e + f) * (0.125f / 4.0f);
    groups[2] = (d + e + g + h) * (0.125f / 4.0f);
    groups[3] = (e + f + h + i) * (0.125f / 4.0f);
    groups[4] = (j + k + l + m) * (0.5f / 4.0f);
    groups[0] *= KarisAverage(groups[0]);
    groups[1] *= KarisAverage(groups[1]);
    groups[2] *= KarisAverage(groups[2]);
    groups[3] *= KarisAverage(groups[3]);
    groups[4] *= KarisAverage(groups[4]);
    result = groups[0] + groups[1] + groups[2] + groups[3] + groups[4];
    result = max(result, 0.0001f);
  }

  outTexture[EeTexel(pixel)] = float4(result, 1.0f);
}
