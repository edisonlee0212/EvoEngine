import EvoEngine.PostProcessing;

struct BloomPushConstant {
  uint2 resolution;
  uint2 bloom_resolution;
  float intensity;
  float3 padding;
};

[[vk::push_constant]]
ConstantBuffer<BloomPushConstant> constants;

[[vk::binding(0, 0)]]
Sampler2D originalColor;

[[vk::binding(1, 0)]]
Sampler2D bloomColor;

[[vk::binding(2, 0)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outColor;

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  if (!EePixelInBounds(pixel, constants.resolution)) {
    return;
  }

  int2 texel = EeTexel(pixel);
  float4 color = originalColor.Load(int3(texel, 0));
  float2 uv = (float2(pixel) + 0.5f) / float2(constants.resolution);
  float2 radius = 0.5f / float2(constants.bloom_resolution);
  float3 bloom = bloomColor.SampleLevel(uv, 0.0f).xyz * 4.0f;
  bloom += bloomColor.SampleLevel(uv + float2(-radius.x, -radius.y), 0.0f).xyz;
  bloom += bloomColor.SampleLevel(uv + float2(0.0f, -radius.y), 0.0f).xyz * 2.0f;
  bloom += bloomColor.SampleLevel(uv + float2(radius.x, -radius.y), 0.0f).xyz;
  bloom += bloomColor.SampleLevel(uv + float2(-radius.x, 0.0f), 0.0f).xyz * 2.0f;
  bloom += bloomColor.SampleLevel(uv + float2(radius.x, 0.0f), 0.0f).xyz * 2.0f;
  bloom += bloomColor.SampleLevel(uv + float2(-radius.x, radius.y), 0.0f).xyz;
  bloom += bloomColor.SampleLevel(uv + float2(0.0f, radius.y), 0.0f).xyz * 2.0f;
  bloom += bloomColor.SampleLevel(uv + float2(radius.x, radius.y), 0.0f).xyz;
  outColor[texel] = float4(color.xyz + constants.intensity * bloom * (1.0f / 16.0f), color.w);
}
