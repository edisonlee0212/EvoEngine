[[vk::binding(0, 0)]]
Sampler2DArrayShadow depth_step;
[[vk::binding(1, 0)]]
RWStructuredBuffer<float> values;

float SampleDepthStep(const float u, const float receiver_depth) {
  return depth_step.SampleCmpLevelZero(float3(u, 0.5f, 0.0f), receiver_depth);
}

[numthreads(1, 1, 1)] void main() {
  values[0] = SampleDepthStep(0.25f, 0.5f);
  values[1] = SampleDepthStep(0.375f, 0.5f);
  values[2] = SampleDepthStep(0.5f, 0.5f);
  values[3] = SampleDepthStep(0.625f, 0.5f);
  values[4] = SampleDepthStep(0.75f, 0.5f);
  values[5] = SampleDepthStep(0.5f, 0.2f);
  values[6] = SampleDepthStep(0.5f, 0.8f);
}
