#ifndef EE_DDGI_GLSL
#define EE_DDGI_GLSL

const float EE_DDGI_PI = 3.14159265359f;
const float EE_DDGI_INACTIVE_RAY_ALPHA = -2.0f;
const uint EE_DDGI_FIXED_RAY_PAYLOAD_FLAG = 1u;
const uint EE_DDGI_RAY_MASK_GEOMETRY = 0x01u;
const uint EE_DDGI_RAY_MASK_SHADOW = 0x02u;
const int EE_DDGI_SCROLL_CLEAR_X_BIT = 1 << 0;
const int EE_DDGI_SCROLL_CLEAR_Y_BIT = 1 << 1;
const int EE_DDGI_SCROLL_CLEAR_Z_BIT = 1 << 2;
const int EE_DDGI_SCROLL_POSITIVE_X_BIT = 1 << 3;
const int EE_DDGI_SCROLL_POSITIVE_Y_BIT = 1 << 4;
const int EE_DDGI_SCROLL_POSITIVE_Z_BIT = 1 << 5;

vec3 EE_DDGI_SAFE_NORMALIZE(const vec3 value, const vec3 fallback) {
  const float length_squared = dot(value, value);
  return length_squared > 0.00000001f ? value * inversesqrt(length_squared) : fallback;
}

float EE_DDGI_AXIS_COORDINATE(const vec3 delta, const vec3 axis) {
  const float axis_length_squared = dot(axis, axis);
  return axis_length_squared > 1e-6f ? dot(delta, axis) / axis_length_squared : 0.0f;
}

float EE_DDGI_AXIS_INDEX(const vec3 delta, const vec3 axis) {
  return round(EE_DDGI_AXIS_COORDINATE(delta, axis));
}

float EE_DDGI_VOLUME_BLEND_WEIGHT(const vec3 probe_coordinate, const uvec3 probe_counts, const vec3 probe_step_x,
                                  const vec3 probe_step_y, const vec3 probe_step_z) {
  const vec3 max_probe_coordinate = vec3(probe_counts - uvec3(1u));
  const bool inside_volume =
      !any(lessThan(probe_coordinate, vec3(0.0f))) && !any(greaterThan(probe_coordinate, max_probe_coordinate));
  if (inside_volume) {
    return 1.0f;
  }

  const vec3 step_lengths =
      max(vec3(length(probe_step_x), length(probe_step_y), length(probe_step_z)), vec3(0.0001f));
  const vec3 lower_distance = max(-probe_coordinate * step_lengths, vec3(0.0f));
  const vec3 upper_distance = max((probe_coordinate - max_probe_coordinate) * step_lengths, vec3(0.0f));
  const vec3 outside_distance = max(lower_distance, upper_distance);
  const vec3 axis_weight = vec3(1.0f) - clamp(outside_distance / step_lengths, vec3(0.0f), vec3(1.0f));
  return axis_weight.x * axis_weight.y * axis_weight.z;
}

vec3 EE_DDGI_PROBE_COORDINATE(const vec3 position, const vec3 first_probe, const vec3 probe_step_x,
                              const vec3 probe_step_y, const vec3 probe_step_z) {
  const vec3 delta = position - first_probe;
  return vec3(EE_DDGI_AXIS_COORDINATE(delta, probe_step_x), EE_DDGI_AXIS_COORDINATE(delta, probe_step_y),
              EE_DDGI_AXIS_COORDINATE(delta, probe_step_z));
}

uint EE_DDGI_PROBE_INDEX(const uvec3 probe_grid, const uvec3 probe_counts) {
  return probe_grid.x + probe_grid.y * probe_counts.x + probe_grid.z * probe_counts.x * probe_counts.y;
}

uvec3 EE_DDGI_WRAP_PROBE_GRID(const ivec3 probe_grid, const uvec3 probe_counts) {
  const ivec3 counts = ivec3(max(probe_counts, uvec3(1u)));
  ivec3 wrapped = probe_grid % counts;
  wrapped = (wrapped + counts) % counts;
  return uvec3(wrapped);
}

uvec3 EE_DDGI_SCROLL_PROBE_GRID(const uvec3 probe_grid, const ivec3 scroll_offset, const uvec3 probe_counts) {
  return EE_DDGI_WRAP_PROBE_GRID(ivec3(probe_grid) + scroll_offset, probe_counts);
}

uint EE_DDGI_SCROLL_PROBE_INDEX(const uvec3 probe_grid, const ivec3 scroll_offset, const uvec3 probe_counts) {
  const uvec3 safe_counts = max(probe_counts, uvec3(1u));
  return EE_DDGI_PROBE_INDEX(EE_DDGI_SCROLL_PROBE_GRID(probe_grid, scroll_offset, safe_counts), safe_counts);
}

bool EE_DDGI_SCROLL_CLEAR_ENABLED(const ivec4 scroll_offset_and_flags, const int plane_index) {
  return (scroll_offset_and_flags.w & (1 << plane_index)) != 0;
}

bool EE_DDGI_SCROLL_DIRECTION_POSITIVE(const ivec4 scroll_offset_and_flags, const int plane_index) {
  return (scroll_offset_and_flags.w & (1 << (plane_index + 3))) != 0;
}

bool EE_DDGI_CLEAR_SCROLLED_PLANE(const uvec3 probe_grid, const int plane_index,
                                  const ivec4 scroll_offset_and_flags, const uvec3 probe_counts) {
  if (!EE_DDGI_SCROLL_CLEAR_ENABLED(scroll_offset_and_flags, plane_index)) {
    return false;
  }

  const int offset = scroll_offset_and_flags[plane_index];
  const int probe_count = int(max(probe_counts[plane_index], 1u));
  const bool positive_direction = EE_DDGI_SCROLL_DIRECTION_POSITIVE(scroll_offset_and_flags, plane_index);
  const int coord =
      positive_direction ? (probe_count + ((offset - 1) % probe_count)) % probe_count
                         : (probe_count + (offset % probe_count)) % probe_count;
  return int(probe_grid[plane_index]) == coord;
}

bool EE_DDGI_CLEAR_SCROLLED_PROBE(const uvec3 probe_grid, const ivec4 scroll_offset_and_flags,
                                  const uvec3 probe_counts) {
  return EE_DDGI_CLEAR_SCROLLED_PLANE(probe_grid, 0, scroll_offset_and_flags, probe_counts) ||
         EE_DDGI_CLEAR_SCROLLED_PLANE(probe_grid, 1, scroll_offset_and_flags, probe_counts) ||
         EE_DDGI_CLEAR_SCROLLED_PLANE(probe_grid, 2, scroll_offset_and_flags, probe_counts);
}

vec3 EE_DDGI_PROBE_WORLD_POSITION(const vec3 probe_grid, const vec3 first_probe, const vec3 probe_step_x,
                                  const vec3 probe_step_y, const vec3 probe_step_z) {
  return first_probe + probe_step_x * probe_grid.x + probe_step_y * probe_grid.y + probe_step_z * probe_grid.z;
}

vec3 EE_DDGI_SURFACE_BIASED_POSITION(const vec3 position, const vec3 normal, const vec3 view_direction,
                                     const float normal_bias, const float view_bias) {
  return position + normal * max(normal_bias, 0.0f) + view_direction * max(view_bias, 0.0f);
}

vec3 EE_DDGI_PROBE_RAY_DIRECTION(const uint ray_index, const uint ray_count, const uint fixed_ray_count) {
  const bool fixed_ray = ray_index < fixed_ray_count;
  const uint sample_index = fixed_ray ? ray_index : ray_index - fixed_ray_count;
  const uint sample_count = fixed_ray ? fixed_ray_count : max(ray_count - fixed_ray_count, 1u);
  const float sample_offset = float(sample_index) + 0.5f;
  const float sample_count_float = max(float(sample_count), 1.0f);
  const float golden_ratio_fraction = (sqrt(5.0f) * 0.5f + 0.5f) - 1.0f;
  const float phi = 2.0f * EE_DDGI_PI * fract(float(sample_index) * golden_ratio_fraction);
  const float cos_theta = 1.0f - 2.0f * sample_offset / sample_count_float;
  const float sin_theta = sqrt(max(0.0f, 1.0f - cos_theta * cos_theta));
  return normalize(vec3(cos(phi) * sin_theta, sin(phi) * sin_theta, cos_theta));
}

float EE_DDGI_SIGN_NOT_ZERO(const float value) {
  return value >= 0.0f ? 1.0f : -1.0f;
}

vec2 EE_DDGI_SIGN_NOT_ZERO(const vec2 value) {
  return vec2(EE_DDGI_SIGN_NOT_ZERO(value.x), EE_DDGI_SIGN_NOT_ZERO(value.y));
}

bool EE_DDGI_RELOCATION_OFFSET_INSIDE_VOXEL(const vec3 offset, const vec3 probe_step_x, const vec3 probe_step_y,
                                            const vec3 probe_step_z) {
  const vec3 normalized_offset = vec3(EE_DDGI_AXIS_COORDINATE(offset, probe_step_x),
                                      EE_DDGI_AXIS_COORDINATE(offset, probe_step_y),
                                      EE_DDGI_AXIS_COORDINATE(offset, probe_step_z));
  return dot(normalized_offset, normalized_offset) < 0.2025f;
}

vec2 EE_DDGI_OCTAHEDRAL_UV(vec3 direction) {
  direction = normalize(direction);
  direction /= max(abs(direction.x) + abs(direction.y) + abs(direction.z), 0.0001f);
  if (direction.z < 0.0f) {
    direction.xy = (vec2(1.0f) - abs(direction.yx)) * EE_DDGI_SIGN_NOT_ZERO(direction.xy);
  }
  return direction.xy * 0.5f + vec2(0.5f);
}

vec3 EE_DDGI_OCTAHEDRAL_DIRECTION(const uvec2 texel, const uint tile_size) {
  const float safe_tile_size = max(float(tile_size), 1.0f);
  const vec2 f = (vec2(texel) + vec2(0.5f)) / safe_tile_size * 2.0f - vec2(1.0f);
  vec3 direction = vec3(f.x, f.y, 1.0f - abs(f.x) - abs(f.y));
  if (direction.z < 0.0f) {
    const vec2 wrapped = (vec2(1.0f) - abs(direction.yx)) * EE_DDGI_SIGN_NOT_ZERO(direction.xy);
    direction = vec3(wrapped, direction.z);
  }
  return normalize(direction);
}

uint EE_DDGI_ATLAS_TILE_STRIDE(const uint tile_size) {
  return max(tile_size, 1u) + 2u;
}

vec2 EE_DDGI_ATLAS_UV(const uint probe_index, const uint atlas_columns, const uint tile_size, const vec3 direction,
                      const vec2 atlas_size) {
  const uvec2 atlas_tile = uvec2(probe_index % atlas_columns, probe_index / atlas_columns);
  const vec2 oct_uv = clamp(EE_DDGI_OCTAHEDRAL_UV(direction), vec2(0.0f), vec2(1.0f));
  const uint tile_stride = EE_DDGI_ATLAS_TILE_STRIDE(tile_size);
  const vec2 tile_uv = oct_uv * max(float(tile_size), 1.0f) + vec2(1.0f);
  return (vec2(atlas_tile * tile_stride) + tile_uv) / atlas_size;
}

float EE_DDGI_CHEBYSHEV_VISIBILITY(const vec2 half_moments, const float distance, const float visibility_bias) {
  const vec2 moments = max(2.0f * half_moments, vec2(0.0f));
  const float mean = moments.x;
  const float second_moment = moments.y;
  if (distance <= mean + visibility_bias) {
    return 1.0f;
  }
  const float variance = abs(mean * mean - second_moment);
  const float distance_delta = distance - mean;
  const float visibility = variance / (variance + distance_delta * distance_delta);
  return max(visibility * visibility * visibility, 0.0f);
}

float EE_DDGI_CRUSH_LOW_WEIGHT(float weight) {
  const float crush_threshold = 0.2f;
  if (weight < crush_threshold) {
    weight *= weight * weight / (crush_threshold * crush_threshold);
  }
  return max(weight, 0.0f);
}

uint EE_DDGI_FIXED_RAY_COUNT(const uint ray_count, const bool fixed_rays_enabled) {
  return fixed_rays_enabled && ray_count > 1u ? min(32u, ray_count - 1u) : 0u;
}

bool EE_DDGI_IS_INACTIVE_PROBE_RAY(const uint hit_count, const float ray_alpha) {
  return hit_count == 0u && ray_alpha <= -1.5f;
}

float EE_DDGI_AXIS_PLANE_DISTANCE(const vec3 ray_direction, const vec3 axis) {
  const float axis_length = length(axis);
  if (axis_length <= 0.0001f) {
    return 1e27f;
  }
  const vec3 direction = EE_DDGI_SAFE_NORMALIZE(ray_direction, vec3(0.0f, 1.0f, 0.0f));
  const float axis_projection = abs(dot(direction, axis / axis_length));
  return axis_length / max(axis_projection, 0.0001f);
}

float EE_DDGI_VOXEL_PLANE_DISTANCE(const vec3 ray_direction, const vec3 probe_step_x, const vec3 probe_step_y,
                                   const vec3 probe_step_z) {
  return min(EE_DDGI_AXIS_PLANE_DISTANCE(ray_direction, probe_step_x),
             min(EE_DDGI_AXIS_PLANE_DISTANCE(ray_direction, probe_step_y),
                 EE_DDGI_AXIS_PLANE_DISTANCE(ray_direction, probe_step_z)));
}

#endif
