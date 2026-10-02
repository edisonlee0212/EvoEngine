import EvoEngine.CameraRayIntegrator;

[[vk::binding(15, 2)]] RWByteAddressBuffer previous_history_buffer;
[[vk::binding(18, 2)]] RWByteAddressBuffer duplication_buffer;

static const uint EE_RESTIR_PT_DUPLICATION_TILE_SIZE = 24u;
groupshared uint tile_seeds[EE_RESTIR_PT_DUPLICATION_TILE_SIZE * EE_RESTIR_PT_DUPLICATION_TILE_SIZE];

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID,
                                                    uint3 group_id : SV_GroupID,
                                                    uint3 group_thread_id : SV_GroupThreadID,
                                                    uint group_index : SV_GroupIndex) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const int2 tile_origin = int2(group_id.xy * 8u) - 8;
  for (uint tile_index = group_index; tile_index < EE_RESTIR_PT_DUPLICATION_TILE_SIZE * EE_RESTIR_PT_DUPLICATION_TILE_SIZE;
       tile_index += 64u) {
    const int2 tile_pixel = tile_origin +
                            int2(tile_index % EE_RESTIR_PT_DUPLICATION_TILE_SIZE,
                                 tile_index / EE_RESTIR_PT_DUPLICATION_TILE_SIZE);
    tile_seeds[tile_index] = all(tile_pixel >= int2(0)) && all(tile_pixel < int2(width, height))
                                 ? previous_history_buffer.Load(
                                       (uint(tile_pixel.y) * width + uint(tile_pixel.x)) * EE_RESTIR_PT_RESERVOIR_BYTES +
                                       8u)
                                 : 0u;
  }
  GroupMemoryBarrierWithGroupSync();
  const uint2 pixel = dispatch_thread_id.xy;
  if (pixel.x >= width || pixel.y >= height) return;
  const uint index = pixel.y * width + pixel.x;
  const RestirPtPathReservoir center = EE_RESTIR_PT_LOAD(previous_history_buffer, index);
  if (center.target_density <= 0.0f) {
    duplication_buffer.Store(index * 4u, 0u);
    return;
  }
  uint matches = 0u;
  uint valid = 0u;
  for (int dy = -8; dy <= 8; ++dy) {
    for (int dx = -8; dx <= 8; ++dx) {
      if (dx == 0 && dy == 0) continue;
      const int2 neighbor = int2(pixel) + int2(dx, dy);
      if (any(neighbor < int2(0)) || any(neighbor >= int2(width, height))) continue;
      const int2 tile_neighbor = int2(group_thread_id.xy) + int2(dx, dy) + 8;
      const uint other_seed = tile_seeds[uint(tile_neighbor.y) * EE_RESTIR_PT_DUPLICATION_TILE_SIZE +
                                         uint(tile_neighbor.x)];
      matches += uint(other_seed == center.sample_seed);
      ++valid;
    }
  }
  duplication_buffer.Store(index * 4u, valid > 0u ? uint(float(matches) / float(valid) * 255.0f) : 0u);
}
