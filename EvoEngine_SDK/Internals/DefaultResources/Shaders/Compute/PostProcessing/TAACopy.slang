#include "PostProcessing.slangh"

[[vk::binding(0, 0)]]
Sampler2D inColor;

[[vk::binding(1, 0)]]
[vk::image_format("rgba32f")] RWTexture2D<float4> outColor;

[shader("compute")][numthreads(16, 16, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  uint resolution_x = 0;
  uint resolution_y = 0;
  outColor.GetDimensions(resolution_x, resolution_y);
  if (!EePixelInBounds(pixel, uint2(resolution_x, resolution_y))) {
    return;
  }

  int2 texel = EeTexel(pixel);
  outColor[texel] = inColor.Load(int3(texel, 0));
}
