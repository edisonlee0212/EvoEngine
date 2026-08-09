// @evoengine-dialect native
import EvoEngine.PostProcessing;

[[vk::push_constant]]
ConstantBuffer<EeResolutionPushConstant> constants;

[[vk::binding(0, 0)]]
Sampler2D inColor;

[[vk::binding(1, 0)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outSourceColor;

[[vk::binding(2, 0)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outResultColor;

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  if (!EePixelInBounds(pixel, constants.resolution)) {
    return;
  }

  int2 texel = EeTexel(pixel);
  float4 color = inColor.Load(int3(texel, 0));
  outSourceColor[texel] = color;
  outResultColor[texel] = float4(max(float3(0.00001f), color.xyz - float3(1.0f)), 1.0f);
}
