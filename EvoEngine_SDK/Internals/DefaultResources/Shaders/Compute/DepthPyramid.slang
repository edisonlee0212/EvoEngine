// @evoengine-dialect native
[[vk::binding(0, 0)]]
Sampler2D sourceDepth;

[[vk::binding(1, 0)]]
[vk::image_format("r32f")]
RWTexture2D<float> targetMip;

struct EeDepthPyramidConstants {
  uint2 sourceResolution;
  uint2 targetResolution;
};

[[vk::push_constant]]
ConstantBuffer<EeDepthPyramidConstants> constants;

float FetchDepth(int2 texel) {
  int2 source_max = int2(constants.sourceResolution) - int2(1);
  return sourceDepth.Load(int3(clamp(texel, int2(0), source_max), 0)).r;
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatchThreadId : SV_DispatchThreadID) {
  uint2 pixel = dispatchThreadId.xy;
  if (pixel.x >= constants.targetResolution.x || pixel.y >= constants.targetResolution.y) {
    return;
  }

  uint2 source_begin = pixel * constants.sourceResolution / constants.targetResolution;
  uint2 source_end =
      ((pixel + uint2(1)) * constants.sourceResolution + constants.targetResolution - uint2(1)) /
      constants.targetResolution;
  float depth = FetchDepth(int2(source_begin));
  for (uint y = source_begin.y; y < source_end.y; ++y) {
    for (uint x = source_begin.x; x < source_end.x; ++x) {
      depth = max(depth, FetchDepth(int2(x, y)));
    }
  }
  targetMip[int2(pixel)] = depth;
}
