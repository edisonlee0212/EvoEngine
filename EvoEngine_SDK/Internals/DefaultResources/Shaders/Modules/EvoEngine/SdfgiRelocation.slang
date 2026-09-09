// Optional unsigned-field surface clearance; coordinates are in anisotropic field cells.
public static const float SDFGI_RELOCATION_RADIUS = 0.45;
public uint SdfgiPlacementIndex(int3 cell, int3 size, uint cascade) {
  return cascade * uint(size.x * size.y * size.z) + uint(cell.x + cell.z * size.x + cell.y * size.x * size.z);
}

public float SdfgiClearance(Texture3D<float> field, SamplerState sampler_state, float3 position, float3 grid) {
  return max(0.0, field.SampleLevel(sampler_state, position / grid, 0) * 255.0 - 1.0);
}

public float4 SdfgiFindPlacement(Texture3D<float> field, SamplerState sampler_state,
                                float3 nominal, float3 grid, float spacing) {
  float radius = SDFGI_RELOCATION_RADIUS * spacing;
  float target = min(1.0, 0.25 * spacing);
  float3 candidate = clamp(nominal, float3(0), grid - 0.001);
  float3 best = candidate;
  float best_clearance = -1;
  // Gradient steps, then stable neighboring directions, then steps from the best fallback.
  for (uint iteration = 0; iteration < 49; ++iteration) {
    if (iteration >= 8 && iteration < 41) {
      uint direction_index = iteration - 8;
      float3 direction;
      if (direction_index < 6) {
        direction = float3(0);
        direction[direction_index / 2] = (direction_index & 1) == 0 ? 1.0 : -1.0;
      } else {
        uint k = direction_index - 6;
        direction = float3(k % 3, (k / 3) % 3, k / 9) - 1;
        if (all(direction == 0)) continue;
      }
      candidate = clamp(nominal + normalize(direction) * radius, float3(0), grid - 0.001);
    } else if (iteration == 41) {
      candidate = best;
    }
    float clearance = SdfgiClearance(field, sampler_state, candidate, grid);
    if (length(candidate - nominal) <= radius + 0.00001 && clearance >= target)
      return float4((candidate - nominal) / spacing, 1);
    if (clearance > best_clearance) { best = candidate; best_clearance = clearance; }
    if (iteration < 8 || iteration >= 41) {
      float3 gradient;
      for (uint axis = 0; axis < 3; ++axis) {
        float3 delta = float3(0); delta[axis] = 0.5;
        gradient[axis] = SdfgiClearance(field, sampler_state, clamp(candidate + delta, 0, grid - 0.001), grid) -
                         SdfgiClearance(field, sampler_state, clamp(candidate - delta, 0, grid - 0.001), grid);
      }
      float gradient_length = length(gradient);
      if (gradient_length > 0.00001) {
        float3 offset = candidate + gradient / gradient_length * max(target - clearance, 0.125) - nominal;
        candidate = clamp(nominal + offset * min(1.0, radius / max(length(offset), 0.00001)), 0, grid - 0.001);
      }
    }
  }
  return float4(0, 0, 0, -1);
}

public float SdfgiSegmentVisibility(Texture3D<float> field, SamplerState sampler_state,
                                    float3 origin, float3 target, float3 grid, float3 receiver_normal = float3(0)) {
  if (any(origin < 0) || any(origin >= grid) || any(target < 0) || any(target >= grid)) return 0;
  float3 delta = origin - target;
  float distance = length(delta);
  if (distance < 0.0001) return 1;
  float3 direction = delta / distance;
  if (dot(direction, receiver_normal) < 0) return 0;
  float travel = 0;
  // Skip only the receiver's contiguous voxel footprint (one cell plus filter support).
  // Once in free space, every obstruction blocks, including another nearby thin wall.
  for (uint step = 0; dot(receiver_normal, receiver_normal) > 0 && step < 24; ++step) {
    if (SdfgiClearance(field, sampler_state, target + direction * travel, grid) >= 0.05) break;
    travel += 0.125;
    if (travel >= distance || any(abs(direction * travel) > 1.5)) {
      // A grazing ray can stay in an adjacent surface's filter footprint indefinitely.
      // Escape locally along the field gradient, then trace the entire remaining segment.
      float3 receiver = target;
      for (uint escape = 0; escape < 16; ++escape) {
        if (SdfgiClearance(field, sampler_state, target, grid) >= 0.05) break;
        float3 gradient;
        for (uint axis = 0; axis < 3; ++axis) {
          float3 offset = float3(0); offset[axis] = 0.5;
          gradient[axis] = field.SampleLevel(sampler_state, (target + offset) / grid, 0) -
                           field.SampleLevel(sampler_state, (target - offset) / grid, 0);
        }
        if (length(gradient) < 0.00001) return 0;
        target += normalize(gradient) * 0.125;
        float3 offset = target - receiver;
        if (any(abs(offset) > 1.5) || dot(offset, receiver_normal) < -0.00001 ||
            dot(offset, delta) < 0 || any(target < 0) || any(target >= grid)) return 0;
      }
      delta = origin - target;
      distance = length(delta);
      if (distance < 0.0001) return 0;
      direction = delta / distance;
      travel = 0;
      break;
    }
  }
  for (uint step = 0; step < 256; ++step) {
    if (travel >= distance) return 1;
    float clearance = SdfgiClearance(field, sampler_state, target + direction * travel, grid);
    if (clearance < 0.05) return 0;
    travel += max(0.025, clearance * 0.8);
  }
  return 0;
}
