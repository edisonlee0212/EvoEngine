import EvoEngine.PostProcessing;

struct BloomPrefilterPushConstant {
  uint2 source_resolution;
  uint2 target_resolution;
  float threshold;
  float knee;
  float2 padding;
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

float3 DownsampleBox13(float2 uv) {
  float2 texel = 0.5f / float2(constants.source_resolution);
  float3 a = inColor.SampleLevel(uv, 0.0f).xyz;
  float3 b = inColor.SampleLevel(uv + texel * float2(-1.0f, -1.0f), 0.0f).xyz;
  float3 c = inColor.SampleLevel(uv + texel * float2(-1.0f, 1.0f), 0.0f).xyz;
  float3 d = inColor.SampleLevel(uv + texel * float2(1.0f, 1.0f), 0.0f).xyz;
  float3 e = inColor.SampleLevel(uv + texel * float2(1.0f, -1.0f), 0.0f).xyz;
  float3 f = inColor.SampleLevel(uv + texel * float2(-2.0f, -2.0f), 0.0f).xyz;
  float3 g = inColor.SampleLevel(uv + texel * float2(-2.0f, 0.0f), 0.0f).xyz;
  float3 h = inColor.SampleLevel(uv + texel * float2(0.0f, 2.0f), 0.0f).xyz;
  float3 i = inColor.SampleLevel(uv + texel * float2(2.0f, 2.0f), 0.0f).xyz;
  float3 j = inColor.SampleLevel(uv + texel * float2(2.0f, -2.0f), 0.0f).xyz;
  float3 k = inColor.SampleLevel(uv + texel * float2(2.0f, 0.0f), 0.0f).xyz;
  float3 l = inColor.SampleLevel(uv + texel * float2(-2.0f, 2.0f), 0.0f).xyz;
  float3 m = inColor.SampleLevel(uv + texel * float2(0.0f, -2.0f), 0.0f).xyz;
  return ((b + c + d + e) * 0.5f + (f + g + a + m) * 0.125f + (g + h + i + a) * 0.125f +
          (a + i + j + k) * 0.125f + (m + a + k + l) * 0.125f) * 0.25f;
}

float3 ApplySoftKnee(float3 color) {
  color = clamp(color, float3(0.0f), float3(20.0f));
  float brightness = max(color.x, max(color.y, color.z));
  float knee = max(constants.knee, 0.0001f);
  float soft = clamp(brightness - (constants.threshold - knee), 0.0f, 2.0f * knee);
  soft = soft * soft * (0.25f / knee);
  return color * (max(soft, brightness - constants.threshold) / max(brightness, 0.0001f));
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  if (!EePixelInBounds(pixel, constants.target_resolution)) {
    return;
  }

  float2 uv = (float2(pixel) + 0.5f) / float2(constants.target_resolution);
  outColor[EeTexel(pixel)] = float4(ApplySoftKnee(DownsampleBox13(uv)), 1.0f);
}
