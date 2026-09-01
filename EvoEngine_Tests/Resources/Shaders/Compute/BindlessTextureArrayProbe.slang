[[vk::binding(9, 0)]] Sampler2D<float4> EE_TEST_TEXTURE_2DS[];
[[vk::binding(10, 0)]] SamplerCube<float4> EE_TEST_CUBEMAPS[];
[[vk::binding(11, 0)]] RWStructuredBuffer<float4> outputValues;

struct ProbePushConstant {
  uint2 texture2DIndices;
  uint2 cubemapIndices;
};

[[vk::push_constant]] ProbePushConstant probe;

[numthreads(2, 1, 1)]
void main(uint3 threadId : SV_DispatchThreadID) {
  uint outputIndex = threadId.x;
  uint texture2DIndex = outputIndex == 0 ? probe.texture2DIndices.x : probe.texture2DIndices.y;
  uint cubemapIndex = outputIndex == 0 ? probe.cubemapIndices.x : probe.cubemapIndices.y;
  outputValues[outputIndex] =
      EE_TEST_TEXTURE_2DS[NonUniformResourceIndex(texture2DIndex)].SampleLevel(float2(0.5), 0.0);
  outputValues[outputIndex + 2] =
      EE_TEST_CUBEMAPS[NonUniformResourceIndex(cubemapIndex)].SampleLevel(float3(1.0, 0.0, 0.0), 0.0);
}
