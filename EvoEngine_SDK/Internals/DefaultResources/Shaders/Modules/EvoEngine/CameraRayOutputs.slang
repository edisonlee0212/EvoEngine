__exported import EvoEngine.CameraRayOutputTypes;

[[vk::binding(0, 2)]] public RWTexture2D<float4> result_image;
[[vk::binding(1, 2)]] public RWTexture2D<float> ray_hit_distance_image;
[[vk::binding(2, 2)]] public RWTexture2D<float4> radiance_history_image;
[[vk::binding(3, 2)]] public RWTexture2D<float4> convergence_history_image;
[[vk::binding(4, 2)]] public RWTexture2D<float4> ray_albedo_image;
[[vk::binding(5, 2)]] public RWTexture2D<float4> ray_normal_image;
[[vk::binding(6, 2)]] public RWTexture2D<uint> ray_count_image;
[[vk::binding(7, 2)]] public RWTexture2D<uint> ray_path_length_image;
[[vk::binding(8, 2)]] public RWTexture2D<uint> ray_time_image;
[[vk::binding(9, 2)]] public RWTexture2D<float4> ray_debug_image;

void EE_CAMERA_STORE_RAY_OUTPUT_COLOR(const Camera camera, const uint output_flag,
                                      RWTexture2D<float4> output_image, const int2 pixel, const float3 value) {
  if (EE_CAMERA_RAY_OUTPUT_ENABLED(camera, output_flag)) {
    output_image[pixel] = float4(max(value, float3(0.0f)), 1.0f);
  }
}

void EE_CAMERA_STORE_RAY_OUTPUT_UINT(const Camera camera, const uint output_flag,
                                     RWTexture2D<uint> output_image, const int2 pixel, const uint value) {
  if (EE_CAMERA_RAY_OUTPUT_ENABLED(camera, output_flag)) {
    output_image[pixel] = value;
  }
}

public void EE_CAMERA_STORE_RAY_OUTPUT_DIAGNOSTICS(const Camera camera, const int2 pixel,
                                                   const CameraRayOutputDiagnostics diagnostics) {
  EE_CAMERA_STORE_RAY_OUTPUT_COLOR(camera, EE_CAMERA_RAY_OUTPUT_ALBEDO, ray_albedo_image, pixel, diagnostics.albedo);
  EE_CAMERA_STORE_RAY_OUTPUT_COLOR(camera, EE_CAMERA_RAY_OUTPUT_NORMAL, ray_normal_image, pixel, diagnostics.normal);
  EE_CAMERA_STORE_RAY_OUTPUT_UINT(camera, EE_CAMERA_RAY_OUTPUT_RAY_COUNT, ray_count_image, pixel,
                                  diagnostics.ray_count);
  EE_CAMERA_STORE_RAY_OUTPUT_UINT(camera, EE_CAMERA_RAY_OUTPUT_PATH_LENGTH, ray_path_length_image, pixel,
                                  diagnostics.path_length);
  EE_CAMERA_STORE_RAY_OUTPUT_UINT(camera, EE_CAMERA_RAY_OUTPUT_TIME, ray_time_image, pixel,
                                  uint(max(diagnostics.time, 0.0f)));
  EE_CAMERA_STORE_RAY_OUTPUT_COLOR(camera, EE_CAMERA_RAY_OUTPUT_DEBUG, ray_debug_image, pixel, diagnostics.debug);
}
