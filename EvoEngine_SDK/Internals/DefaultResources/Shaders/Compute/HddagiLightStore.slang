// Godot hddagi_preprocess.glsl::MODE_LIGHT_STORE, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Coordinates use 8 bits per axis; dispatch is bounded on overflow.
import EvoEngine.HddagiTrace;
import EvoEngine.HddagiVoxel;
#define PROBE_CELLS 8
#define PROCESS_STATIC_PENDING_BIT 0x80000000u
#define PROCESS_DYNAMIC_PENDING_BIT 0x40000000u
struct HddagiLightStoreParams {
  int3 grid_size;
  uint maximum_light_cells;
  int3 offset;
  uint cascade;
  int3 limit;
  uint region_index;
  int3 region_world_pos;
  uint padding2;
};

[[vk::push_constant]] ConstantBuffer<HddagiLightStoreParams> params;
[[vk::binding(0, 0)]][vk::image_format("r16ui")] Texture3D<uint> src_albedo;
[[vk::binding(1, 0)]] Texture3D<uint> src_emission;
[[vk::binding(2, 0)]] Texture3D<uint> src_emission_aniso;
[[vk::binding(3, 0)]] Texture3D<uint> src_normal_bits;
[[vk::binding(4, 0)]] RWStructuredBuffer<uint> dispatch_data;
[[vk::binding(5, 0)]] RWStructuredBuffer<HddagiProcessVoxel> dst_process_voxels;
[[vk::binding(6, 0)]] Texture3D<float4> occlusion0;
[[vk::binding(7, 0)]] Texture3D<float4> occlusion1;
[[vk::binding(8, 0)]][vk::image_format("r8ui")] RWTexture3D<uint> dst_disocclusion;
[[vk::binding(9, 0)]][vk::image_format("r32ui")] RWTexture3D<uint> voxel_neighbours;
[[vk::binding(10, 0)]][vk::image_format("r32ui")] RWTexture3D<uint> light_tex;
[[vk::binding(11, 0)]] StructuredBuffer<HddagiUpdateBounds> update_bounds;
groupshared uint normal_facings[216];
groupshared uint store_position_count;
groupshared uint store_from_index;
uint get_normal_facing(int3 pos) {
  pos += 1;
  return normal_facings[pos.z * 36 + pos.y * 6 + pos.x];
}
void set_normal_facing(int3 pos, uint facing) {
  pos += 1;
  normal_facings[pos.z * 36 + pos.y * 6 + pos.x] = facing;
}
float2 octahedron_encode(float3 n) {
  n /= abs(n.x) + abs(n.y) + abs(n.z);
  n.xy = n.z >= 0 ? n.xy : (1 - abs(n.yx)) * select(n.xy >= 0, float2(1), float2(-1));
  return n.xy * 0.5 + 0.5;
}
[numthreads(4, 4, 4)] void main(uint3 id : SV_DispatchThreadID, uint3 local_id : SV_GroupThreadID,
                                uint3 group_id : SV_GroupID) {
  int3 local = int3(local_id);
  int3 pos = int3(id) + params.offset;
  {
    int3 load_from = local * 6 / 4;
    int3 load_to = (local + int3(1)) * 6 / 4;
    int3 base_pos = params.offset + int3(group_id) * 4;
    for (int i = load_from.x; i < load_to.x; i++) {
      for (int j = load_from.y; j < load_to.y; j++) {
        for (int k = load_from.z; k < load_to.z; k++) {
          int3 i_ofs = int3(i, j, k) - int3(1);
          int3 load_pos = base_pos + i_ofs;
          uint solid = 0;
          if (all(load_pos >= int3(0)) && all(load_pos < params.grid_size)) {
            solid = src_normal_bits.Load(int4(load_pos, 0));
          }
          set_normal_facing(i_ofs, solid);
        }
      }
    }
  }
  if (all(local == int3(0))) store_position_count = 0;
  if (all(pos == params.offset)) dispatch_data[1] = dispatch_data[2] = 1;
  bool thread_active = true;
  if (any(pos >= params.limit)) thread_active = false;
  for (uint region = 0; region < params.region_index; ++region)
    if (HddagiContains(update_bounds[region], pos, params.cascade)) thread_active = false;
  GroupMemoryBarrierWithGroupSync();
  float4 albedo_accum = float4(0.0);
  float4 emission_accum = float4(0.0);
  float3 normal_accum = float3(0.0);
  float3 first_normal = float3(0);
  uint occlusionu = 0;
  bool voxels_found = false;
  int3 base_dst_pos = int3(0);
  if (thread_active) {
    uint solid = get_normal_facing(local);
    const int3 offsets[6] = {int3(1, 0, 0),  int3(-1, 0, 0), int3(0, 1, 0),
                             int3(0, -1, 0), int3(0, 0, 1),  int3(0, 0, -1)};
    const float3 aniso_dir[6] = {float3(-1, 0, 0), float3(1, 0, 0),  float3(0, -1, 0),
                                 float3(0, 1, 0),  float3(0, 0, -1), float3(0, 0, 1)};
    const uint aniso_mask[6] = {(1 << 0), (1 << 1), (1 << 2), (1 << 3), (1 << 4), (1 << 5)};
    const uint aniso_offset_mask[6] = {(1 << 1), (1 << 0), (1 << 3), (1 << 2), (1 << 5), (1 << 4)};
    uint disocclusion = 0;
    const int facing_direction_count = 26;
    bool use_for_filter = false;
    for (int i = 0; i < 6; i++) {
      uint n = get_normal_facing(local + offsets[i]);
      if (n == 0) {
        disocclusion |= aniso_offset_mask[i];
      } else if (solid == 0) {
        use_for_filter = true;
      }
      if (solid != 0 || !bool(n & aniso_mask[i])) {
        continue;
      }
      voxels_found = true;
      for (int j = 0; j < facing_direction_count; j++) {
        if (bool(n & uint((1 << (j + 6))))) {
          if (all(first_normal == 0))
            first_normal = HDDAGI_FACING_DIRECTIONS[j];
          normal_accum += HDDAGI_FACING_DIRECTIONS[j];
        }
      }
      int3 ofs = pos + offsets[i];
      int3 albedo_ofs = ofs >> 1;
      albedo_ofs.z *= 6;
      albedo_ofs.z += i;
      uint a = src_albedo.Load(int4(albedo_ofs, 0));
      albedo_accum +=
          float4(float3((int3(a) >> int3(0, 5, 11)) & int3(0x1f, 0x3f, 0x1f)) / float3(31.0, 63.0, 31.0), 1.0);
      uint rgbe = src_emission.Load(int4(ofs >> 1, 0));
      float3 emission = HddagiDecodeRadiance(rgbe);
      uint rgbe_aniso = src_emission_aniso.Load(int4(ofs >> 1, 0));
      float strength = ((rgbe_aniso >> (i * 5)) & 0x1F) / float(0x1F);
      emission_accum += float4(emission * strength, 1.0);
    }
    base_dst_pos = HddagiWrap(pos + params.region_world_pos * 8, params.grid_size);
    int3 dst_pos = base_dst_pos;
    dst_pos.y += params.grid_size.y * params.cascade;
    dst_disocclusion[dst_pos] = disocclusion;
    voxel_neighbours[dst_pos] = 0;
    if (!voxels_found)
      light_tex[dst_pos] = 0;
    if (solid != 0) {
      thread_active = false;
    } else if (use_for_filter) {
      uint neighbour_voxels = 0;
      for (int i = 0; i < facing_direction_count; i++) {
        int3 neighbor = int3(sign(HDDAGI_FACING_DIRECTIONS[i]));
        int3 neighbour_pos = local + neighbor;
        uint n = get_normal_facing(neighbour_pos);
        if (n == 0) {
          continue;
        }
        for (int j = 0; j < 6; j++) {
          int3 neighbour_neighbour = neighbour_pos + int3(aniso_dir[j]);
          int3 nn_rel = neighbour_neighbour - local;
          if (any(nn_rel < -int3(1)) || any(nn_rel > +int3(1))) {
            continue;
          }
          if (all(nn_rel == int3(0))) {
            continue;
          }
          uint q = get_normal_facing(local + nn_rel);
          if (q != 0) {
            continue;
          }
          int3 nn_rel_abs = abs(nn_rel);
          int nn_steps = nn_rel_abs.x + nn_rel_abs.y + nn_rel_abs.z;
          if (nn_steps == 3) {
            continue;
          }
          if (nn_steps > 1) {
            int3 test_dirs[3] = int3[](int3(nn_rel.x, 0, 0), int3(0, nn_rel.y, 0), int3(0, 0, nn_rel.z));
            int occlusions = 0;
            for (int k = 0; k < 3; k++) {
              if (all(test_dirs[k] == int3(0))) {
                continue;
              }
              q = get_normal_facing(local + test_dirs[k]);
              if (q != 0) {
                occlusions++;
              }
            }
            if (occlusions >= 2) {
              continue;
            }
          }
          const uint reverse_map[27] = {6, 14, 18, 9,  4, 21, 11, 16, 23, 7,  2,  19, 0, 0,
                                        1, 12, 3,  24, 8, 15, 20, 10, 5,  22, 13, 17, 25};
          int3 abs_pos = nn_rel + int3(1);
          neighbour_voxels |= 1 << reverse_map[abs_pos.z * 3 * 3 + abs_pos.y * 3 + abs_pos.x];
        }
      }
      int3 store_pos = HddagiWrap(pos + params.region_world_pos * 8, params.grid_size);
      store_pos.y += params.grid_size.y * params.cascade;
      voxel_neighbours[store_pos] = neighbour_voxels;
    }
  }
  GroupMemoryBarrierWithGroupSync();
  uint index = 0;
  if (thread_active && voxels_found) {
    InterlockedAdd(store_position_count, 1, index);
  }
  GroupMemoryBarrierWithGroupSync();
  if (thread_active) {
    if (!voxels_found || store_position_count == 0) {
      thread_active = false;
    } else {
      if (thread_active && index == 0) {
        InterlockedAdd(dispatch_data[3], store_position_count, store_from_index);
        uint group_count = (min(store_from_index + store_position_count, params.maximum_light_cells) + 63) / 64;
        InterlockedMax(dispatch_data[0], group_count);
        if (store_from_index + store_position_count > params.maximum_light_cells)
          InterlockedOr(dispatch_data[4], 1u);
      }
    }
  }
  GroupMemoryBarrierWithGroupSync();
  if (thread_active) {
    int3 base_probe = params.region_world_pos + pos / PROBE_CELLS;
    int3 occ_pos = base_dst_pos + int3(1);
    occ_pos.y += (params.grid_size.y + 2) * params.cascade;
    float4 occ_0 = occlusion0.Load(int4(occ_pos, 0));
    float4 occ_1 = occlusion1.Load(int4(occ_pos, 0));
    if (bool(base_probe.x & 1)) {
      occ_0.xyzw = occ_0.yxwz;
      occ_1.xyzw = occ_1.yxwz;
    }
    if (bool(base_probe.y & 1)) {
      occ_0.xyzw = occ_0.zwxy;
      occ_1.xyzw = occ_1.zwxy;
    }
    if (bool(base_probe.z & 1)) {
      float4 tmp = occ_0;
      occ_0 = occ_1;
      occ_1 = tmp;
    }
    float total_weight = dot(occ_0, float4(1)) + dot(occ_1, float4(1));
    if (total_weight > 0.0) {
      occ_0 /= total_weight;
      occ_1 /= total_weight;
    }
    float weights[8] = float[](occ_0.x, occ_0.y, occ_0.z, occ_0.w, occ_1.x, occ_1.y, occ_1.z, occ_1.w);
    for (int i = 0; i < 8; i++) {
      float w;
      if (total_weight > 0.0) {
        w = weights[i];
        w *= 15.0;
      } else {
        w = 0;
      }
      occlusionu |= uint(clamp(w, 0.0, 15.0)) << (i * 4);
    }
    index += store_from_index;
    if (index < params.maximum_light_cells) {
      normal_accum = dot(normal_accum, normal_accum) > 1e-8 ? normalize(normal_accum) : first_normal;
      albedo_accum.rgb /= albedo_accum.a;
      emission_accum.rgb /= emission_accum.a;
      dst_process_voxels[index].position =
          uint(pos.x | (pos.y << 8) | (pos.z << 16)) | PROCESS_STATIC_PENDING_BIT | PROCESS_DYNAMIC_PENDING_BIT;
      uint albedo_norm = 0;
      albedo_norm |= clamp(uint(albedo_accum.r * 31.0), 0, 31) << 0;
      albedo_norm |= clamp(uint(albedo_accum.g * 63.0), 0, 63) << 5;
      albedo_norm |= clamp(uint(albedo_accum.b * 31.0), 0, 31) << 11;
      float2 octa_normal = octahedron_encode(normal_accum);
      uint2 octa_unormal = clamp(uint2(octa_normal * 255), uint2(0), uint2(255));
      albedo_norm |= (octa_unormal.x << 16) | (octa_unormal.y << 24);
      dst_process_voxels[index].albedo_normal = albedo_norm;
      dst_process_voxels[index].emission = HddagiEncodeRadiance(emission_accum.rgb);
      dst_process_voxels[index].occlusion = occlusionu;
    }
  }
}
