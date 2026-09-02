import EvoEngine.PostProcessing;

struct BloomPrefilterPushConstant {
  uint2 source_resolution;
  uint2 target_resolution;
  float threshold;
  float knee;
  float compression_start;
  float source_ceiling;
};

[[vk::push_constant]]
ConstantBuffer<BloomPrefilterPushConstant> constants;

[[vk::binding(0, 0)]]
Sampler2D inColor;

#if defined(EE_BLOOM_STORAGE_FORMAT_RGBA16F)
[vk::image_format("rgba16f")]
#else
[vk::image_format("rgba32f")]
#endif
[[vk::binding(1, 0)]]
RWTexture2D<float4> outColor;

float3 SampleBloom(float2 uv) {
  return EePrefilterBloom(inColor.SampleLevel(uv, 0.0f).xyz, constants.threshold, constants.knee,
                          constants.compression_start, constants.source_ceiling);
}

float3 DownsampleBox13(float2 uv) {
  float2 texel = 0.5f / float2(constants.source_resolution);
  float3 a = SampleBloom(uv);
  float3 b = SampleBloom(uv + texel * float2(-1.0f, -1.0f));
  float3 c = SampleBloom(uv + texel * float2(-1.0f, 1.0f));
  float3 d = SampleBloom(uv + texel * float2(1.0f, 1.0f));
  float3 e = SampleBloom(uv + texel * float2(1.0f, -1.0f));
  float3 f = SampleBloom(uv + texel * float2(-2.0f, -2.0f));
  float3 g = SampleBloom(uv + texel * float2(-2.0f, 0.0f));
  float3 h = SampleBloom(uv + texel * float2(0.0f, 2.0f));
  float3 i = SampleBloom(uv + texel * float2(2.0f, 2.0f));
  float3 j = SampleBloom(uv + texel * float2(2.0f, -2.0f));
  float3 k = SampleBloom(uv + texel * float2(2.0f, 0.0f));
  float3 l = SampleBloom(uv + texel * float2(-2.0f, 2.0f));
  float3 m = SampleBloom(uv + texel * float2(0.0f, -2.0f));
  return ((b + c + d + e) * 0.5f + (f + g + a + m) * 0.125f + (g + h + i + a) * 0.125f +
          (a + i + j + k) * 0.125f + (m + a + k + l) * 0.125f) * 0.25f;
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  if (!EePixelInBounds(pixel, constants.target_resolution)) {
    return;
  }

  float2 uv = (float2(pixel) + 0.5f) / float2(constants.target_resolution);
  outColor[EeTexel(pixel)] = float4(DownsampleBox13(uv), 1.0f);
}
