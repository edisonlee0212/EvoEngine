// Core HDDAGI cache layout from Godot da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Geometry/placement changes invalidate hits AND misses before reuse.
import EvoEngine.HddagiTrace;

bool HddagiTraceCached(Texture3D<uint2> voxels, [vk::image_format("r8ui")] Texture3D<uint> regions,
                       [vk::image_format("r16ui")] Texture3D<uint> versions,
                       [vk::image_format("r32ui")] RWTexture2DArray<uint> cache,
                       [vk::image_format("r16ui")] RWTexture2DArray<uint> cache_versions,
                       int3 cache_pixel, HddagiCascadeBlock cascades, int3 grid, uint cascade_count,
                       float3 origin, float3 direction, uint first_cascade, bool use_cache,
                       out int3 physical_cell, out uint hit_cascade, out uint iterations, out bool reused) {
  physical_cell = int3(0);
  hit_cascade = iterations = 0;
  reused = false;
  uint entry = use_cache ? cache[cache_pixel] : 0;
  if ((entry & 0x80000000u) != 0) {
    bool hit = (entry & 0x40000000u) != 0;
    int3 stored = int3(entry & 255, (entry >> 8) & 255, (entry >> 16) & 255);
    uint cascade = (entry >> 24) & 7;
    // The upstream write shifts by REGION_SIZE (8), while its read divides by 8. Both must divide.
    if (!hit || (cascade >= first_cascade && cascade < cascade_count && all(stored < grid) &&
                 versions.Load(int4(stored / 8 + int3(0, grid.y / 8 * cascade, 0), 0)) == cache_versions[cache_pixel])) {
      reused = true;
      if (hit) { physical_cell = stored; hit_cascade = cascade; }
      return hit;
    }
  }
  bool hit = false;
  uint first = first_cascade;
  while (first < cascade_count) {
    int3 cell, side;
    uint cascade, steps;
    bool found = HddagiTrace(voxels, regions, cascades, grid, cascade_count, origin, direction, first, 8,
                             cell, side, cascade, steps);
    iterations += steps;
    if (!found) break;
    cell += side;
    if (all(cell >= 0) && all(cell < grid)) {
      physical_cell = HddagiWrap(cell + cascades.data[cascade].region_world_offset * 8, grid);
      hit_cascade = cascade;
      hit = true;
      break;
    }
    // An exterior neighbor has no lighting payload in this cascade; never wrap it to the opposite edge.
    first = cascade + 1;
  }
  if (use_cache) {
    uint value = 0x80000000u;
    uint version = 0;
    if (hit) {
      value |= 0x40000000u | uint(physical_cell.x) | (uint(physical_cell.y) << 8) |
               (uint(physical_cell.z) << 16) | (hit_cascade << 24);
      version = versions.Load(int4(physical_cell / 8 + int3(0, grid.y / 8 * hit_cascade, 0), 0));
    }
    cache[cache_pixel] = value;
    cache_versions[cache_pixel] = version;
  }
  return hit;
}
