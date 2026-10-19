import EvoEngine.CameraRayOutputs;

struct PtDisplayFireflyPushConstants {
  uint camera_index;
};
[[vk::push_constant]] ConstantBuffer<PtDisplayFireflyPushConstants> constants;

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 thread : SV_DispatchThreadID) {
  uint width, height;
  radiance_history_image.GetDimensions(width, height);
  const int2 pixel = int2(thread.xy);
  if (any(thread.xy >= uint2(width, height))) return;

  const Camera camera = EE_CAMERAS[constants.camera_index];
  const float3 center = max(radiance_history_image[pixel].xyz, 0.0f);
  const float center_luminance = dot(center, float3(0.2126f, 0.7152f, 0.0722f));
  float neighbor_luminance = 0.0f;
  bool has_neighbor = false;
  for (int y = -1; y <= 1; ++y) {
    for (int x = -1; x <= 1; ++x) {
      if (x == 0 && y == 0) continue;
      const int2 neighbor = pixel + int2(x, y);
      if (any(neighbor < 0) || any(neighbor >= int2(width, height))) continue;
      has_neighbor = true;
      const float3 color = max(radiance_history_image[neighbor].xyz, 0.0f);
      neighbor_luminance = max(neighbor_luminance, dot(color, float3(0.2126f, 0.7152f, 0.0722f)));
    }
  }
  const float limit = max(neighbor_luminance, 1e-4f) * camera.pt_display_firefly_ratio;
  const float scale = has_neighbor && center_luminance > limit ? limit / center_luminance : 1.0f;
  result_image[pixel] = float4(center * lerp(1.0f, scale, camera.pt_display_firefly_strength), 1.0f);
}
