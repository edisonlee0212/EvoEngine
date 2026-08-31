[[vk::binding(9, 0)]] Sampler2D<float4> EE_TEST_TEXTURE_2DS[];
[[vk::binding(11, 0)]] RWStructuredBuffer<float4> outputValues;
[[vk::binding(0, 1)]] Sampler2D<float4> fixedTextureA;
[[vk::binding(1, 1)]] Sampler2D<float4> fixedTextureB;

struct ProbePushConstant {
  uint2 textureIndices;
};

[[vk::push_constant]] ProbePushConstant probe;

[numthreads(1, 1, 1)]
void main() {
  const float2 uv = float2(0.5, 0.5);
  outputValues[0] = EE_TEST_TEXTURE_2DS[NonUniformResourceIndex(probe.textureIndices.x)].SampleLevel(uv, 0.0);
  outputValues[1] = fixedTextureA.SampleLevel(uv, 0.0);
  outputValues[2] = EE_TEST_TEXTURE_2DS[NonUniformResourceIndex(probe.textureIndices.y)].SampleLevel(uv, 0.0);
  outputValues[3] = fixedTextureB.SampleLevel(uv, 0.0);
}
