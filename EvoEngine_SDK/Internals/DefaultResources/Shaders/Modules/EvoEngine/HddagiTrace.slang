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
  // Avoid negative signed remainder: rectangular GPU fields/probe histories must wrap bijectively.
  uint3 magnitude = uint3(value);
  magnitude = select(value < 0, uint3(0) - magnitude, magnitude);
  uint3 remainder = magnitude % uint3(size);
  return int3(select(value < 0 && remainder != 0, uint3(size) - remainder, remainder));
}

bool HddagiTrace(Texture3D<uint2> voxels, [vk::image_format("r8ui")] Texture3D<uint> regions, HddagiCascadeBlock cascades,
                int3 grid, uint cascade_count, float3 ray_position, float3 ray_direction,
                uint first_cascade, uint fractional_bits, out int3 cell, out int3 side,
                out uint hit_cascade, out uint iterations, float maximum_distance = -1) {
  cell = side = int3(0);
  hit_cascade = first_cascade;
  iterations = 0;
  bool3 ray_zero = abs(ray_direction) < 1.0 / 127.0;
  if (all(ray_zero) || !all(isfinite(ray_position)) || !all(isfinite(ray_direction)))
    return false;
  if (maximum_distance == 0 || isnan(maximum_distance))
    return false;
  bool bounded = maximum_distance > 0;
  float remaining_distance = maximum_distance;
  int3 distance_limit = int3(0);
  bool distance_limit_valid = false;
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
      if (entered_cascade && bounded && remaining_distance <= 0)
        break;
      if (entered_cascade)
        ray_position = float3(position) / (fp * cascades.data[cascade].to_cell) + cascades.data[cascade].offset;
      entered_cascade = false;
      if (++cascade >= int(cascade_count))
        break;
      float3 local = (ray_position - cascades.data[cascade].offset) * cascades.data[cascade].to_cell;
      if (any(local < 0) || any(local >= float3(grid)))
        continue;
      position = int3(local * fp);
      if (bounded) {
        float3 box = select(ray_zero, float3(3.402823466e38),
                            (float3(grid * step) - local) / select(ray_zero, float3(1), ray_direction));
        float advance_to_bounds = min(box.x, min(box.y, box.z));
        float distance_cells = remaining_distance * cascades.data[cascade].to_cell;
        distance_limit_valid = distance_cells < advance_to_bounds;
        if (distance_limit_valid) {
          // Use the segment endpoint; Godot's sign-only box can extend diagonal shadows past a light.
          distance_limit = position + int3(ray_direction * (distance_cells * fp));
        } else {
          remaining_distance -= advance_to_bounds / cascades.data[cascade].to_cell;
        }
      }
      entered_cascade = true;
      cascade_base = int3(0, grid.y / 8 * cascade, 0);
      level = 0;
      continue;
    }
    int3 difference = masks[level] * step - (position & masks[level]);
    int3 times = select(ray_zero, int3(0x7fffffff), difference * inverse_fp) >> fractional_bits;
    int t = min(times.x, min(times.y, times.z));
    position += select(times == t, difference + ray_sign, (direction_fp * t) >> fractional_bits);
    if (distance_limit_valid && any(!ray_zero && ((position < distance_limit) != (ray_direction > 0))))
      break;
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

// Direct-light visibility uses 10 fractional bits and a finite segment.
bool HddagiVisibility(Texture3D<uint2> voxels, [vk::image_format("r8ui")] Texture3D<uint> regions,
                      HddagiCascadeBlock cascades, int3 grid, uint cascade_count, float3 origin,
                      float3 direction, float distance, uint first_cascade, out uint iterations) {
  int3 cell, side;
  uint cascade;
  return HddagiTrace(voxels, regions, cascades, grid, cascade_count, origin, direction, first_cascade, 10,
                    cell, side, cascade, iterations, distance);
}
