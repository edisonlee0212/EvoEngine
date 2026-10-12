import EvoEngine.CameraRayOutputs;
[[vk::binding(41, 2)]] RWTexture2D<float4> next_color_image;
[shader("compute")][numthreads(8, 8, 1)] void main(uint3 thread : SV_DispatchThreadID) {
  uint width, height;
  result_image.GetDimensions(width, height);
  if (any(thread.xy >= uint2(width, height))) return;
  result_image[thread.xy] = float4(next_color_image[thread.xy].xyz, 1.0f);
}
