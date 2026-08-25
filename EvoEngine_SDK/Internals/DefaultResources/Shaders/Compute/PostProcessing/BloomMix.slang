import EvoEngine.PostProcessing;

[[vk::push_constant]]
ConstantBuffer<EeResolutionPushConstant> constants;

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
  float4 bloom = bloomColor.Load(int3(texel, 0));
  outColor[texel] = float4(clamp(color.xyz, float3(0.0f), float3(1.0f)) + bloom.xyz, color.w);
}
