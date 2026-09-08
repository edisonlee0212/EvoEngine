// Godot gi.glsl probe reconstruction, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Rectangular wrapping and zero-weight guards are host adaptations.
__exported import EvoEngine.HddagiTrace;
import EvoEngine.HddagiVoxel;

struct HddagiGatherData {
  HddagiCascadeBlock cascades;
  int3 grid;
  uint cascade_count;
  int3 probe_size;
  float energy;
  float3 anchor_origin;
  float y_mult;
  float normal_bias;
  float reflection_bias;
  float occlusion_bias;
  uint blend_ambient;
};

float2 HddagiOctEncode(float3 n) {
  n /= dot(abs(n), float3(1));
  float2 oct = n.xy;
  if (n.z < 0)
    oct = (1 - abs(oct.yx)) * select(oct >= 0, float2(1), float2(-1));
  return oct * 0.5 + 0.5;
}

void HddagiGatherProbes(HddagiGatherData data, Texture2DArray<float3> diffuse_atlas,
                        Texture2DArray<float3> specular_atlas, Texture3D<float4> occlusion0,
                        Texture3D<float4> occlusion1, SamplerState linear_sampler, uint cascade, float3 cell_position,
                        float3 normal, float3 reflection, float roughness, bool dynamic_receiver, out float3 diffuse,
                        out float3 specular) {
  float3 position = cell_position + (dynamic_receiver ? float3(0) : normal * data.normal_bias);
  // Bias may leave the chosen cascade near an edge. Clamp to its represented probe interval.
  position = clamp(position, float3(0), float3(data.grid) - 0.0001);
  int3 integer_position = int3(position);
  int3 base_probe = integer_position / 8;
  int3 occlusion_cell = HddagiWrap(integer_position + data.cascades.data[cascade].region_world_offset * 8, data.grid);
  occlusion_cell += int3(1, 1 + (data.grid.y + 2) * cascade, 1);
  int3 occlusion_size = (data.grid + 2) * int3(1, data.cascade_count, 1);
  float3 occlusion_uv = (float3(occlusion_cell) + frac(position)) / float3(occlusion_size);
  float4 occ0 = occlusion0.SampleLevel(linear_sampler, occlusion_uv, 0);
  float4 occ1 = occlusion1.SampleLevel(linear_sampler, occlusion_uv, 0);
  float2 inv_size = 1.0 / float2(7 * data.probe_size.x, 7 * data.probe_size.y * data.probe_size.z);
  float2 diffuse_oct = HddagiOctEncode(normal) * 5;
  float2 specular_oct = HddagiOctEncode(reflection) * 5;
  diffuse = specular = float3(0);
  float total = 0;
  for (uint i = 0; i < 8; ++i) {
    int3 probe = base_probe + int3(uint3(i, i >> 1, i >> 2) & 1);
    float3 delta = position - float3(probe * 8);
    float distance = length(delta);
    float weight = dynamic_receiver ? 1 : max(0.005, distance > 0 ? dot(-delta / distance, normal) : 1);
    int3 parity = (data.cascades.data[cascade].region_world_offset + probe) & 1;
    uint index = parity.x | (parity.y << 1) | (parity.z << 2);
    weight *= max(data.occlusion_bias, index < 4 ? occ0[index] : occ1[index - 4]);
    float3 trilinear = max(float3(0), 1 - abs(delta / 8));
    weight *= trilinear.x * trilinear.y * trilinear.z;
    int3 physical = HddagiWrap(probe + data.cascades.data[cascade].region_world_offset, data.probe_size);
    float2 base_uv = float2((physical.xy + int2(0, physical.z * data.probe_size.y)) * 7 + 1);
    diffuse +=
        diffuse_atlas.SampleLevel(linear_sampler, float3((base_uv + diffuse_oct) * inv_size, cascade), 0) * weight;
    float3 sharp =
        roughness > 0 && roughness < 0.99
            ? specular_atlas.SampleLevel(linear_sampler, float3((base_uv + specular_oct) * inv_size, cascade), 0)
            : float3(0);
    float3 broad = roughness > 0.25 ? diffuse_atlas.SampleLevel(linear_sampler,
                                                                float3((base_uv + specular_oct) * inv_size, cascade), 0)
                                    : float3(0);
    specular += lerp(sharp, broad, smoothstep(0.25, 1, roughness)) * weight;
    total += weight;
  }
  diffuse = total > 0 ? diffuse / total : float3(0);
  specular = total > 0 ? specular / total : float3(0);
}

float4 HddagiSharpReflection(HddagiGatherData data, Texture3D<uint2> voxels,
                             [vk::image_format("r8ui")] Texture3D<uint> regions, Texture3D<float3> light,
                             [vk::image_format("r8ui")] Texture3D<uint> disocclusion, Texture3D<uint> neighbors,
                             float3 position, float3 normal, float3 reflection, float roughness, uint cascade,
                             uint2 pixel) {
  float3 start = (position - data.cascades.data[cascade].offset) * data.cascades.data[cascade].to_cell;
  start += normal / max(max(abs(normal.x), abs(normal.y)), abs(normal.z)) * data.reflection_bias;
  float3 ray_position = start / data.cascades.data[cascade].to_cell + data.cascades.data[cascade].offset;
  uint sample_index = (pixel.x & 1) + (pixel.y & 1) * 2;
  float theta = 2.399963229728653 * sample_index;
  float z = 1 - 0.5 * sample_index;
  float radius = sqrt(max(0, 1 - z * z));
  float3 sample =
      normalize(lerp(float3(0, 0, -1), float3(radius * cos(theta), radius * sin(theta), z), roughness * roughness));
  sample.z = -sample.z;
  float3 tangent = normalize(cross(abs(reflection.z) < 0.999 ? float3(0, 0, 1) : float3(0, 1, 0), reflection));
  float3 bitangent = normalize(cross(tangent, reflection));
  float3 ray_direction = tangent * sample.x + bitangent * sample.y + reflection * sample.z;
  for (uint first = cascade; first < data.cascade_count; ++first) {
    int3 cell, side;
    uint hit_cascade, iterations;
    if (!HddagiTrace(voxels, regions, data.cascades, data.grid, data.cascade_count, ray_position, ray_direction, first,
                     10, cell, side, hit_cascade, iterations))
      return float4(0);
    bool self_hit = hit_cascade == cascade && all(cell == int3(start));
    int3 world_offset = data.cascades.data[hit_cascade].region_world_offset * 8;
    int3 cascade_base = int3(0, data.grid.y * hit_cascade, 0);
    if (self_hit) {
      uint mask = disocclusion.Load(int4(HddagiWrap(cell + world_offset, data.grid) + cascade_base, 0));
      if (mask == 0) {
        float3 abs_normal = abs(normal);
        uint axis = abs_normal.y > abs_normal.x ? 1 : 0;
        if (abs_normal.z > abs_normal[axis])
          axis = 2;
        float3 local = frac(start) - 0.5;
        uint a = (axis + 1) % 3, b = (axis + 2) % 3;
        uint advance_axis = abs(local[a]) > abs(local[b]) ? a : b;
        int3 advance = int3(0);
        advance[advance_axis] = int(sign(local[advance_axis]));
        start += float3(advance);
        cell += advance;
        if (any(cell < 0) || any(cell >= data.grid))
          continue;
        mask = disocclusion.Load(int4(HddagiWrap(cell + world_offset, data.grid) + cascade_base, 0));
      }
      uint best = 0;
      float best_distance = -20;
      for (uint i = 0; i < 6; ++i) {
        float distance = dot(frac(start) - 0.5, HDDAGI_FACING_DIRECTIONS[i]);
        if ((mask & (1u << i)) != 0 && distance > best_distance) {
          best = i;
          best_distance = distance;
        }
      }
      side = int3(HDDAGI_FACING_DIRECTIONS[best]);
    }
    cell += side;
    // A face outside this cascade has no adjacent payload; retry the ray in the next cascade.
    if (any(cell < 0) || any(cell >= data.grid)) {
      first = hit_cascade;
      continue;
    }
    int3 physical = HddagiWrap(cell + world_offset, data.grid) + cascade_base;
    float3 radiance = light.Load(int4(physical, 0));
    if (!self_hit) {
      float to_cell = data.cascades.data[hit_cascade].to_cell;
      float3 center = (float3(cell) + 0.5) / to_cell + data.cascades.data[hit_cascade].offset;
      float3 delta = center - ray_position;
      float weight = max(0, 1 - length(delta - dot(delta, ray_direction) * ray_direction) * to_cell);
      float4 accumulated = float4(radiance, 1) * weight;
      uint mask = neighbors.Load(int4(physical, 0)) & 0x3ffffffu;
      while (mask != 0) {
        uint i = firstbitlow(mask);
        int3 offset = int3(sign(HDDAGI_FACING_DIRECTIONS[i]));
        int3 neighbor = cell + offset;
        if (all(neighbor >= 0) && all(neighbor < data.grid)) {
          delta = center + float3(offset) / to_cell - ray_position;
          weight = max(0, 1 - length(delta - dot(delta, ray_direction) * ray_direction) * to_cell);
          accumulated +=
              float4(light.Load(int4(HddagiWrap(neighbor + world_offset, data.grid) + cascade_base, 0)), 1) * weight;
        }
        mask &= ~(1u << i);
      }
      if (accumulated.w > 0)
        radiance = accumulated.rgb / accumulated.w;
    }
    return float4(radiance, 1);
  }
  return float4(0);
}

void HddagiGatherLighting(HddagiGatherData data, Texture2DArray<float3> diffuse_atlas,
                          Texture2DArray<float3> specular_atlas, Texture3D<float4> occlusion0,
                          Texture3D<float4> occlusion1, SamplerState linear_sampler, Texture3D<uint2> voxels,
                          [vk::image_format("r8ui")] Texture3D<uint> regions, Texture3D<float3> light,
                          [vk::image_format("r8ui")] Texture3D<uint> disocclusion, Texture3D<uint> neighbors,
                          float3 world_position, float3 normal, float3 reflection, float roughness,
                          bool dynamic_receiver, bool trace_reflections, uint2 pixel, out float4 diffuse,
                          out float4 specular) {
  diffuse = specular = float4(0);
  float3 scale = float3(1, data.y_mult, 1);
  float3 position = world_position * scale - data.anchor_origin;
  normal = normalize(normal * scale);
  reflection = normalize(reflection * scale);
  uint cascade = data.cascade_count;
  float3 cell = float3(0);
  for (uint c = 0; c < data.cascade_count; ++c) {
    cell = (position - data.cascades.data[c].offset) * data.cascades.data[c].to_cell;
    if (all(cell >= 0) && all(cell < float3(data.grid))) {
      cascade = c;
      break;
    }
  }
  if (cascade == data.cascade_count)
    return;
  float3 inner_distance = float3(data.probe_size - 1) * 0.5 - abs(position * (data.cascades.data[cascade].to_cell / 8));
  float blend = 1 - smoothstep(0.5, 2.5, min(inner_distance.x, min(inner_distance.y, inner_distance.z)));
  static const float dither[4] = {0, 0.5, 0.75, 0.25};
  bool select_outer = blend > dither[(pixel.x & 1) + (pixel.y & 1) * 2];
  if (data.blend_ambient == 0 && cascade + 1 < data.cascade_count) {
    if (select_outer)
      ++cascade;
    cell = (position - data.cascades.data[cascade].offset) * data.cascades.data[cascade].to_cell;
    blend = 0;
  }
  float3 ambient, reflected;
  HddagiGatherProbes(data, diffuse_atlas, specular_atlas, occlusion0, occlusion1, linear_sampler, cascade, cell, normal,
                     reflection, roughness, dynamic_receiver, ambient, reflected);
  if (data.blend_ambient != 0 && blend > 0 && cascade + 1 < data.cascade_count) {
    float3 outer_diffuse, outer_specular;
    float3 outer_cell = (position - data.cascades.data[cascade + 1].offset) * data.cascades.data[cascade + 1].to_cell;
    HddagiGatherProbes(data, diffuse_atlas, specular_atlas, occlusion0, occlusion1, linear_sampler, cascade + 1,
                       outer_cell, normal, reflection, roughness, dynamic_receiver, outer_diffuse, outer_specular);
    ambient = lerp(ambient, outer_diffuse, blend);
    reflected = lerp(reflected, outer_specular, blend);
    if (select_outer)
      ++cascade;
    blend = 0;
  }
  diffuse = float4(ambient, 1 - blend);
  specular = float4(reflected, 1 - blend);
  if (trace_reflections && roughness < 0.25) {
    float4 sharp = HddagiSharpReflection(data, voxels, regions, light, disocclusion, neighbors, position, normal,
                                         reflection, roughness, cascade, pixel);
    specular = lerp(sharp, specular, smoothstep(0, 0.25, roughness));
  }
  diffuse.rgb *= data.energy;
  specular.rgb *= data.energy;
}
