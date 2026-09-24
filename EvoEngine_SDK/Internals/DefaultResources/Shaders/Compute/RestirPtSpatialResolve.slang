import EvoEngine.CameraRayIntegrator;

[[vk::binding(12, 2)]] RWByteAddressBuffer resolved_buffer;

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel_coordinate = dispatch_thread_id.xy;
  const uint2 image_size = uint2(width, height);
  if (any(pixel_coordinate >= image_size)) {
    return;
  }
  EE_CAMERA_RESOLVE_RESTIR_CANDIDATE_PIXEL(pixel_coordinate, image_size, resolved_buffer);
}
