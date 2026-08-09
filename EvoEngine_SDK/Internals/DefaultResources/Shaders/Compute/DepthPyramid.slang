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

  int2 source_pixel = int2(pixel * 2);
  float depth = FetchDepth(source_pixel);
  if (any(constants.sourceResolution != constants.targetResolution)) {
    depth = min(depth, FetchDepth(source_pixel + int2(1, 0)));
    depth = min(depth, FetchDepth(source_pixel + int2(0, 1)));
    depth = min(depth, FetchDepth(source_pixel + int2(1, 1)));
  }
  targetMip[int2(pixel)] = depth;
}
