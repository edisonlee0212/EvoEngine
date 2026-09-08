// Godot hddagi_integrate.glsl::trace_ray_hdda, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Rectangular wrapping and safe reciprocal conversion are host adaptations.
struct HddagiCascade {
  float3 offset;
  float to_cell;
  int3 region_world_offset;
  float padding;
};

struct HddagiCascadeBlock {
  HddagiCascade data[8];
};

int3 HddagiWrap(int3 value, int3 size) {
  return (value % size + size) % size;
}

bool HddagiTrace(Texture3D<uint2> voxels, [vk::image_format("r8ui")] Texture3D<uint> regions, HddagiCascadeBlock cascades,
                int3 grid, uint cascade_count, float3 ray_position, float3 ray_direction,
                uint first_cascade, uint fractional_bits, out int3 cell, out int3 side,
                out uint hit_cascade, out uint iterations) {
  cell = side = int3(0);
  hit_cascade = first_cascade;
  iterations = 0;
  bool3 ray_zero = abs(ray_direction) < 1.0 / 127.0;
  if (all(ray_zero) || !all(isfinite(ray_position)) || !all(isfinite(ray_direction)))
    return false;
  int fp = 1 << fractional_bits;
  int3 step = select(ray_direction > 0, int3(1), int3(0));
  int3 ray_sign = int3(sign(ray_direction));
  int3 direction_fp = int3(ray_direction * fp);
  int3 inverse_fp = int3(float(fp) / select(ray_zero, float3(1), ray_direction));
  int3 masks[3] = {int3(fp * 8 - 1), int3(fp * 4 - 1), int3(fp - 1)};
  int3 limits[3];
  limits[0] = (grid * fp - 1) * step;
  int level = -1;
  int cascade = int(first_cascade) - 1;
  int3 position = int3(0);
  int3 cascade_base = int3(0);
  int3 region_base = int3(0);
  uint2 block = uint2(0);
  bool hit = false;
  bool entered_cascade = false;
  while (true) {
    ++iterations;
    if (level == 2) {
      int3 local = (position & masks[1]) >> fractional_bits;
      uint bit = uint(local.z * 16 + local.y * 4 + local.x);
      if ((block[bit >> 5] & (1u << (bit & 31))) != 0) {
        hit = true;
        break;
      }
    } else if (level == 1) {
      int3 local = (position & masks[0]) >> (fractional_bits + 2);
      block = voxels.Load(int4(region_base + local, 0));
      if (any(block != 0)) {
        level = 2;
        limits[2] = position - (position & masks[1]) + step * (masks[1] + 1);
        continue;
      }
    } else if (level == 0) {
      int3 region = position >> (fractional_bits + 3);
      region = HddagiWrap(cascades.data[cascade].region_world_offset + region, grid / 8) + cascade_base;
      if (regions.Load(int4(region, 0)) != 0) {
        region_base = region * 2;
        level = 1;
        limits[1] = position - (position & masks[0]) + step * (masks[0] + 1);
        continue;
      }
    } else {
      if (entered_cascade)
        ray_position = float3(position) / (fp * cascades.data[cascade].to_cell) + cascades.data[cascade].offset;
      entered_cascade = false;
      if (++cascade >= int(cascade_count))
        break;
      float3 local = (ray_position - cascades.data[cascade].offset) * cascades.data[cascade].to_cell;
      if (any(local < 0) || any(local >= float3(grid)))
        continue;
      position = int3(local * fp);
      entered_cascade = true;
      cascade_base = int3(0, grid.y / 8 * cascade, 0);
      level = 0;
      continue;
    }
    int3 difference = masks[level] * step - (position & masks[level]);
    int3 times = select(ray_zero, int3(0x7fffffff), difference * inverse_fp) >> fractional_bits;
    int t = min(times.x, min(times.y, times.z));
    position += select(times == t, difference + ray_sign, (direction_fp * t) >> fractional_bits);
    while (true) {
      if (all((position < limits[level]) == (ray_direction > 0)))
        break;
      if (--level == -1)
        break;
    }
  }
  if (hit) {
    int3 difference = masks[2] * (step ^ 1) - (position & masks[2]);
    int3 times = select(ray_zero, int3(0x7fffffff), difference * -inverse_fp);
    uint axis = times.x < times.y ? 0 : 1;
    if (times.z < times[axis])
      axis = 2;
    side[axis] = -ray_sign[axis];
    cell = position >> fractional_bits;
    hit_cascade = uint(cascade);
  }
  return hit;
}
