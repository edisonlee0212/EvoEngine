import EvoEngine.CameraRayIntegrator;

[[vk::binding(15, 2)]] RWByteAddressBuffer previous_history_buffer;
[[vk::binding(18, 2)]] RWByteAddressBuffer duplication_buffer;

groupshared uint2 origins[24u * 24u];

[shader("compute")][numthreads(8, 8, 1)]
void main(uint3 pixel : SV_DispatchThreadID, uint3 group : SV_GroupID,
          uint3 local : SV_GroupThreadID, uint thread : SV_GroupIndex) {
  uint width, height;
  result_image.GetDimensions(width, height);
  const bool history_valid = EE_CAMERA_SUBSURFACE_HISTORY_VALID();
  const int2 tile_origin = int2(group.xy * 8u) - 8;
  for (uint i = thread; i < 576u; i += 64u) {
    const int2 p = tile_origin + int2(i % 24u, i / 24u);
    origins[i] = history_valid && all(p >= int2(0)) && all(p < int2(width, height))
        ? previous_history_buffer.Load2((uint(p.y) * width + uint(p.x)) * EE_RESTIR_SSS_HISTORY_BYTES + 104u)
        : uint2(0xffffffffu);
  }
  GroupMemoryBarrierWithGroupSync();
  if (pixel.x >= width || pixel.y >= height) return;
  const uint index = pixel.y * width + pixel.x;
  if (!history_valid) {
    duplication_buffer.Store(index * 4u, 0u);
    return;
  }
  const RestirSssDirectSample center = EE_RESTIR_SSS_LOAD(previous_history_buffer, index);
  if (!(center.target_density > 0.0f) || center.origin_pixel == 0xffffffffu) {
    duplication_buffer.Store(index * 4u, 0u);
    return;
  }
  uint matches = 0u, valid = 0u;
  for (int y = -8; y <= 8; ++y)
    for (int x = -8; x <= 8; ++x) {
      if (x == 0 && y == 0) continue;
      const int2 p = int2(pixel.xy) + int2(x, y);
      if (any(p < int2(0)) || any(p >= int2(width, height))) continue;
      const uint2 t = uint2(int2(local.xy) + int2(x, y) + 8);
      matches += uint(all(origins[t.y * 24u + t.x] == uint2(center.origin_pixel, center.origin_frame)));
      ++valid;
    }
  duplication_buffer.Store(index * 4u, asuint(valid > 0u ? float(matches) / float(valid) : 0.0f));
}
