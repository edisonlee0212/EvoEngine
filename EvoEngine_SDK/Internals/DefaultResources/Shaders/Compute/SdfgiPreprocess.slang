// Preprocessing adapted from Godot shaders/environment/sdfgi_preprocess.glsl,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
import EvoEngine.SdfgiTypes;

[[vk::push_constant]] ConstantBuffer<SdfgiPreprocessPushConstant> params;

#if defined(MODE_INITIALIZE_JUMP_FLOOD) || defined(MODE_INITIALIZE_JUMP_FLOOD_HALF)
[[vk::binding(1, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> src_color;
[[vk::binding(2, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> dst_positions;

#elif defined(MODE_JUMPFLOOD) || defined(MODE_JUMPFLOOD_OPTIMIZED)
[[vk::binding(1, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> src_positions;
[[vk::binding(2, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> dst_positions;

#elif defined(MODE_UPSCALE_JUMP_FLOOD)
[[vk::binding(1, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> src_color;
[[vk::binding(2, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> src_positions_half;
[[vk::binding(3, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> dst_positions;

#elif defined(MODE_OCCLUSION)
[[vk::binding(1, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> src_color;
[[vk::binding(2, 0)]] [vk::image_format("r8")] RWTexture3D<float> dst_occlusion[8];
[[vk::binding(3, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> src_facing;

#elif defined(MODE_STORE)
[[vk::binding(1, 0)]] [vk::image_format("rgba8ui")] RWTexture3D<uint4> src_positions;
[[vk::binding(2, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> src_color;
[[vk::binding(3, 0)]] [vk::image_format("r8")] RWTexture3D<float> src_occlusion[8];
[[vk::binding(4, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> src_emission;
[[vk::binding(5, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> src_emission_aniso;
[[vk::binding(6, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> src_facing;
[[vk::binding(7, 0)]] [vk::image_format("r8")] RWTexture3D<float> dst_sdf;
[[vk::binding(8, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> dst_occlusion;

[[vk::binding(10, 0)]] RWStructuredBuffer<SdfgiDispatchData> dispatch_data;
[[vk::binding(11, 0)]] RWStructuredBuffer<SdfgiSolidCell> dst_process_voxels;
[[vk::binding(12, 0)]] RWStructuredBuffer<SdfgiFieldStatus> field_status;
#elif defined(MODE_SCROLL)
[[vk::binding(1, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> dst_albedo;
[[vk::binding(2, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> dst_facing;
[[vk::binding(3, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> dst_light;
[[vk::binding(4, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> dst_light_aniso;

[[vk::binding(5, 0)]] StructuredBuffer<SdfgiDispatchData> dispatch_data;
[[vk::binding(6, 0)]] StructuredBuffer<SdfgiSolidCell> src_process_voxels;
[[vk::binding(7, 0)]] StructuredBuffer<SdfgiFieldStatus> field_status;
#elif defined(MODE_SCROLL_OCCLUSION)
[[vk::binding(1, 0)]] [vk::image_format("r8")] RWTexture3D<float> dst_occlusion[8];
[[vk::binding(2, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> src_occlusion;

#endif


int3 grid_size() {
  return int3(params.grid_size, params.grid_size_y, params.grid_size);
}

#define GROUP_SIZE 8
#ifndef OCCLUSION_SIZE
#define OCCLUSION_SIZE 8
#endif
#ifdef MODE_JUMPFLOOD_OPTIMIZED

groupshared uint4 group_positions[(GROUP_SIZE + 2) * (GROUP_SIZE + 2) * (GROUP_SIZE + 2)]; //4x4x4 with margins

void group_store(int3 p_pos, uint4 p_value) {
  uint offset = uint(p_pos.z * (GROUP_SIZE + 2) * (GROUP_SIZE + 2) + p_pos.y * (GROUP_SIZE + 2) + p_pos.x);
  group_positions[offset] = p_value;
}

uint4 group_load(int3 p_pos) {
  uint offset = uint(p_pos.z * (GROUP_SIZE + 2) * (GROUP_SIZE + 2) + p_pos.y * (GROUP_SIZE + 2) + p_pos.x);
  return group_positions[offset];
}

#endif

#ifdef MODE_OCCLUSION
static const uint2 group_size_offset[11] = {uint2(1, 0), uint2(3, 1), uint2(6, 4), uint2(10, 10), uint2(15, 20), uint2(21, 35), uint2(28, 56), uint2(36, 84), uint2(42, 120), uint2(46, 162), uint2(48, 208)};
static const uint group_pos[256] = {0,
    65536, 256, 1,
    131072, 65792, 512, 65537, 257, 2,
    196608, 131328, 66048, 768, 131073, 65793, 513, 65538, 258, 3,
    262144, 196864, 131584, 66304, 1024, 196609, 131329, 66049, 769, 131074, 65794, 514, 65539, 259, 4,
    327680, 262400, 197120, 131840, 66560, 1280, 262145, 196865, 131585, 66305, 1025, 196610, 131330, 66050, 770, 131075, 65795, 515, 65540, 260, 5,
    393216, 327936, 262656, 197376, 132096, 66816, 1536, 327681, 262401, 197121, 131841, 66561, 1281, 262146, 196866, 131586, 66306, 1026, 196611, 131331, 66051, 771, 131076, 65796, 516, 65541, 261, 6,
    458752, 393472, 328192, 262912, 197632, 132352, 67072, 1792, 393217, 327937, 262657, 197377, 132097, 66817, 1537, 327682, 262402, 197122, 131842, 66562, 1282, 262147, 196867, 131587, 66307, 1027, 196612, 131332, 66052, 772, 131077, 65797, 517, 65542, 262, 7,
    459008, 393728, 328448, 263168, 197888, 132608, 67328, 458753, 393473, 328193, 262913, 197633, 132353, 67073, 1793, 393218, 327938, 262658, 197378, 132098, 66818, 1538, 327683, 262403, 197123, 131843, 66563, 1283, 262148, 196868, 131588, 66308, 1028, 196613, 131333, 66053, 773, 131078, 65798, 518, 65543, 263,
    459264, 393984, 328704, 263424, 198144, 132864, 459009, 393729, 328449, 263169, 197889, 132609, 67329, 458754, 393474, 328194, 262914, 197634, 132354, 67074, 1794, 393219, 327939, 262659, 197379, 132099, 66819, 1539, 327684, 262404, 197124, 131844, 66564, 1284, 262149, 196869, 131589, 66309, 1029, 196614, 131334, 66054, 774, 131079, 65799, 519,
    459520, 394240, 328960, 263680, 198400, 459265, 393985, 328705, 263425, 198145, 132865, 459010, 393730, 328450, 263170, 197890, 132610, 67330, 458755, 393475, 328195, 262915, 197635, 132355, 67075, 1795, 393220, 327940, 262660, 197380, 132100, 66820, 1540, 327685, 262405, 197125, 131845, 66565, 1285, 262150, 196870, 131590, 66310, 1030, 196615, 131335, 66055, 775};

groupshared uint occlusion_facing[((OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2)) / 4];

uint get_facing(int3 p_pos) {
  uint ofs = uint(p_pos.z * OCCLUSION_SIZE * 2 * OCCLUSION_SIZE * 2 + p_pos.y * OCCLUSION_SIZE * 2 + p_pos.x);
  uint v = occlusion_facing[ofs / 4];
  return (v >> ((ofs % 4) * 8)) & 0xFF;
}

#endif

#ifdef MODE_STORE
groupshared SdfgiSolidCell store_positions[4 * 4 * 4];
groupshared uint store_position_count;
groupshared uint store_from_index;
#endif


#ifdef MODE_JUMPFLOOD_OPTIMIZED
[numthreads(8, 8, 8)]
#elif defined(MODE_OCCLUSION) || defined(MODE_SCROLL)
[numthreads(64, 1, 1)]
#else
[numthreads(4, 4, 4)]
#endif
void main(uint3 id : SV_DispatchThreadID, uint3 local_id : SV_GroupThreadID, uint3 group_id : SV_GroupID) {

#ifdef MODE_SCROLL
  uint index = id.x;
  if (field_status[0].failure_flags != 0 || index >= min(dispatch_data[0].total_count, field_status[0].solid_cell_capacity)) return;
  SdfgiSolidCell voxel = src_process_voxels[index];
  int3 read_pos = SdfgiUnpackPosition(voxel);
  int3 write_pos = read_pos + params.scroll;
  if (any(write_pos < 0) || any(write_pos >= grid_size())) return;
  dst_albedo[write_pos] = ((voxel.albedo & 0x7fff) << 1) | 1;
  dst_facing[write_pos] = (voxel.albedo >> 15) & 0x3f;
  dst_light[write_pos] = voxel.light & 0x3fffffff;
  dst_light_aniso[write_pos] = voxel.light_aniso & 0x3fffffff;
#endif

#ifdef MODE_SCROLL_OCCLUSION
  int3 pos = int3(id);
  if (any(pos >= grid_size() - abs(params.scroll))) return;
  int3 read_pos = pos + max(int3(0), -params.scroll);
  int3 write_pos = pos + max(int3(0), params.scroll);
  read_pos.z += int(params.cascade * params.grid_size);
  uint packed = src_occlusion[read_pos] | (src_occlusion[read_pos + int3(params.grid_size, 0, 0)] << 16);
  static const uint shifts[8] = {12, 8, 4, 0, 28, 24, 20, 16};
  for (uint i = 0; i < 8; ++i)
    dst_occlusion[i][write_pos] = float((packed >> shifts[i]) & 15) / 15.0;
#endif

#ifdef MODE_INITIALIZE_JUMP_FLOOD

  int3 pos = int3(id.xyz);

  uint c = uint4(src_color[pos]).r;
  uint4 v;
  if (bool(c & 0x1)) {
    //bit set means this is solid
    v.xyz = uint3(pos);
    v.w = 255; //not zero means used
  } else {
    v.xyz = uint3(0);
    v.w = 0; // zero means unused
  }

  dst_positions[pos] = v;
#endif

#ifdef MODE_INITIALIZE_JUMP_FLOOD_HALF

  int3 pos = int3(id.xyz);
  int3 base_pos = pos * 2;

  //since we store in half size, lets kind of randomize what we store, so
  //the half size jump flood has a bit better chance to find something
  uint4 closest[8];
  int closest_count = 0;

  for (uint i = 0; i < 8; i++) {
    int3 src_pos = base_pos + ((int3(i) >> int3(0, 1, 2)) & int3(1, 1, 1));
    uint c = uint4(src_color[src_pos]).r;
    if (bool(c & 1)) {
      uint4 v = uint4(uint3(src_pos), 255);
      closest[closest_count] = v;
      closest_count++;
    }
  }

  if (closest_count == 0) {
    dst_positions[pos] = uint4(0);
  } else {
    int3 indexv = (pos & int3(1, 1, 1)) * int3(1, 2, 4);
    int index = (indexv.x | indexv.y | indexv.z) % closest_count;
    dst_positions[pos] = closest[index];
  }

#endif

#ifdef MODE_JUMPFLOOD

  //regular jumpflood, efficient for large steps, inefficient for small steps
  int3 pos = int3(id.xyz);

  float3 posf = float3(pos);

  if (params.half_size) {
    posf = posf * 2.0 + 0.5;
  }

  uint4 p = uint4(src_positions[pos]);

  if (!params.half_size && all(p == uint4(uint3(pos), 255))) {
    dst_positions[pos] = p;
    return; //points to itself and valid, nothing better can be done, just pass
  }

  float p_dist;

  if (p.w != 0) {
    p_dist = distance(posf, float3(p.xyz));
  } else {
    p_dist = 0.0; //should not matter
  }

  const uint offset_count = 26;
  static const int3 offsets[offset_count] = {
      int3(-1, -1, -1),
      int3(-1, -1, 0),
      int3(-1, -1, 1),
      int3(-1, 0, -1),
      int3(-1, 0, 0),
      int3(-1, 0, 1),
      int3(-1, 1, -1),
      int3(-1, 1, 0),
      int3(-1, 1, 1),
      int3(0, -1, -1),
      int3(0, -1, 0),
      int3(0, -1, 1),
      int3(0, 0, -1),
      int3(0, 0, 1),
      int3(0, 1, -1),
      int3(0, 1, 0),
      int3(0, 1, 1),
      int3(1, -1, -1),
      int3(1, -1, 0),
      int3(1, -1, 1),
      int3(1, 0, -1),
      int3(1, 0, 0),
      int3(1, 0, 1),
      int3(1, 1, -1),
      int3(1, 1, 0),
      int3(1, 1, 1)};

  for (uint i = 0; i < offset_count; i++) {
    int3 ofs = pos + offsets[i] * params.step_size;
    if (any((ofs < int3(0))) || any((ofs >= grid_size()))) {
      continue;
    }
    uint4 q = uint4(src_positions[ofs]);

    if (q.w == 0) {
      continue; //was not initialized yet, ignore
    }

    float q_dist = distance(posf, float3(q.xyz));
    if (p.w == 0 || q_dist < p_dist) {
      p = q; //just replace because current is unused
      p_dist = q_dist;
    }
  }

  dst_positions[pos] = p;
#endif

#ifdef MODE_JUMPFLOOD_OPTIMIZED
  //optimized version using groupshared compute memory

  int3 group_offset = int3(group_id.xyz) % params.step_size;
  int3 group_pos = group_offset + (int3(group_id.xyz) / params.step_size) * int3(GROUP_SIZE * params.step_size);

  //load data into local group memory

  if (all((int3(local_id.xyz) < int3((GROUP_SIZE + 2) / 2)))) {
    //use this thread for loading, this method uses less threads for this but its simpler and less divergent
    int3 base_pos = int3(local_id.xyz) * 2;
    for (uint i = 0; i < 8; i++) {
      int3 load_pos = base_pos + ((int3(i) >> int3(0, 1, 2)) & int3(1, 1, 1));
      int3 load_global_pos = group_pos + (load_pos - int3(1)) * params.step_size;
      uint4 q;
      if (all((load_global_pos >= int3(0))) && all((load_global_pos < grid_size()))) {
        q = uint4(src_positions[load_global_pos]);
      } else {
        q = uint4(0); //unused
      }

      group_store(load_pos, q);
    }
  }

  //sync
  AllMemoryBarrierWithGroupSync();

  int3 global_pos = group_pos + int3(local_id.xyz) * params.step_size;

  if (any((global_pos < int3(0))) || any((global_pos >= grid_size()))) {
    return; //do nothing else, end here because outside range
  }

  int3 local_pos = int3(local_id.xyz) + int3(1);

  const uint offset_count = 27;
  static const int3 offsets[offset_count] = {
      int3(-1, -1, -1),
      int3(-1, -1, 0),
      int3(-1, -1, 1),
      int3(-1, 0, -1),
      int3(-1, 0, 0),
      int3(-1, 0, 1),
      int3(-1, 1, -1),
      int3(-1, 1, 0),
      int3(-1, 1, 1),
      int3(0, -1, -1),
      int3(0, -1, 0),
      int3(0, -1, 1),
      int3(0, 0, -1),
      int3(0, 0, 0),
      int3(0, 0, 1),
      int3(0, 1, -1),
      int3(0, 1, 0),
      int3(0, 1, 1),
      int3(1, -1, -1),
      int3(1, -1, 0),
      int3(1, -1, 1),
      int3(1, 0, -1),
      int3(1, 0, 0),
      int3(1, 0, 1),
      int3(1, 1, -1),
      int3(1, 1, 0),
      int3(1, 1, 1)};

  //only makes sense if point is inside screen
  uint4 closest = uint4(0);
  float closest_dist = 0.0;

  float3 posf = float3(global_pos);

  if (params.half_size) {
    posf = posf * 2.0 + 0.5;
  }

  for (uint i = 0; i < offset_count; i++) {
    uint4 point = group_load(local_pos + offsets[i]);

    if (point.w == 0) {
      continue; //was not initialized yet, ignore
    }

    float dist = distance(posf, float3(point.xyz));
    if (closest.w == 0 || dist < closest_dist) {
      closest = point;
      closest_dist = dist;
    }
  }

  dst_positions[global_pos] = closest;

#endif

#ifdef MODE_UPSCALE_JUMP_FLOOD

  int3 pos = int3(id.xyz);

  uint c = uint4(src_color[pos]).r;
  uint4 v;
  if (bool(c & 1)) {
    //bit set means this is solid
    v.xyz = uint3(pos);
    v.w = 255; //not zero means used
  } else {
    v = uint4(src_positions_half[pos >> 1]);
    float d = length(float3(int3(v.xyz) - pos));

    int3 vbase = int3(v.xyz - (v.xyz & uint3(1)));

    //search around if there is a better candidate from the same block
    for (int i = 0; i < 8; i++) {
      int3 bits = ((int3(i) >> int3(0, 1, 2)) & int3(1, 1, 1));
      int3 p = vbase + bits;

      float d2 = length(float3(p - pos));
      if (d2 < d) { //check valid distance before test so we avoid a read
        uint c2 = uint4(src_color[p]).r;
        if (bool(c2 & 1)) {
          v.xyz = uint3(p);
          d = d2;
        }
      }
    }

    //could validate better position..
  }

  dst_positions[pos] = v;

#endif

#ifdef MODE_OCCLUSION

  uint invocation_idx = uint(local_id.x);
  int3 region = int3(group_id);

  int3 region_offset = -int3(OCCLUSION_SIZE);
  region_offset += region * OCCLUSION_SIZE * 2;
  region_offset += params.probe_offset * OCCLUSION_SIZE;

  bool region_out_of_bounds = false;

  if (any(params.scroll != int3(0))) {
    //validate scroll region
    int3 region_offset_to = region_offset + int3(OCCLUSION_SIZE * 2);
    uint3 scroll_mask = uint3(params.scroll != int3(0));
    int3 scroll_from = select((params.scroll < int3(0)), grid_size() + params.scroll, int3(0));
    int3 scroll_to = select((params.scroll > int3(0)), params.scroll, grid_size());

    if (all(((uint3(region_offset_to <= scroll_from) | uint3(region_offset >= scroll_to)) * scroll_mask) == scroll_mask)) {
      region_out_of_bounds = true; //region outside scroll bounds, quit
    }
  }

  if (!region_out_of_bounds) {
    for (uint packed_index = invocation_idx; packed_index < (OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2) / 4; packed_index += 64) {
      uint facing_pack = 0;
      for (int j = 0; j < 4; j++) {
        uint index = packed_index * 4 + j;
        int3 offset = int3(index % (OCCLUSION_SIZE * 2), (index / (OCCLUSION_SIZE * 2)) % (OCCLUSION_SIZE * 2), index / ((OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2)));
        int3 foffset = region_offset + offset;
        if (all((foffset >= int3(0))) && all((foffset < grid_size()))) {
          uint f = uint4(src_facing[foffset]).r;
          facing_pack |= f << (j * 8);
        }
      }

      occlusion_facing[packed_index] = facing_pack;
    }
  }

  //sync occlusion saved
  AllMemoryBarrierWithGroupSync();

  //process occlusion

#define OCC_STEPS (OCCLUSION_SIZE * 3 - 2)
#define OCC_HALF_STEPS (OCC_STEPS / 2)

  for (int step = 0; step < OCC_STEPS; step++) {
    if (!region_out_of_bounds) {
#if OCCLUSION_SIZE == 8
      bool shrink = step >= OCC_HALF_STEPS;
      int occ_step = shrink ? OCC_HALF_STEPS - (step - OCC_HALF_STEPS) - 1 : step;

      if (invocation_idx < group_size_offset[occ_step].x) {
        uint pv = group_pos[group_size_offset[occ_step].y + invocation_idx];
        int3 proc_abs = (int3(int(pv)) >> int3(0, 8, 16)) & int3(0xFF);

        if (shrink) {
          proc_abs = int3(OCCLUSION_SIZE) - proc_abs - int3(1);
        }
#else
      int3 proc_abs = int3(invocation_idx % OCCLUSION_SIZE, invocation_idx / OCCLUSION_SIZE, 0);
      proc_abs.z = step - proc_abs.x - proc_abs.y;
      if (all(proc_abs >= 0) && all(proc_abs < OCCLUSION_SIZE)) {
#endif

        for (int i = 0; i < 8; i++) {
          int3 bits = ((int3(i) >> int3(0, 1, 2)) & int3(1, 1, 1));
          int3 proc_sign = bits * 2 - 1;
          int3 local_offset = int3(OCCLUSION_SIZE) + proc_abs * proc_sign - (int3(1) - bits);
          int3 offset = local_offset + region_offset;
          if (all((offset >= int3(0))) && all((offset < grid_size()))) {
            float occ;

            uint facing = get_facing(local_offset);

            if (facing != 0) { //solid
              occ = 0.0;
            } else if (step == 0) {
              occ = 1.0;

            } else {
              int3 read_dir = -proc_sign;

              int3 major_axis;
              if (proc_abs.x < proc_abs.y) {
                if (proc_abs.z < proc_abs.y) {
                  major_axis = int3(0, 1, 0);
                } else {
                  major_axis = int3(0, 0, 1);
                }
              } else {
                if (proc_abs.z < proc_abs.x) {
                  major_axis = int3(1, 0, 0);
                } else {
                  major_axis = int3(0, 0, 1);
                }
              }

              float avg = 0.0;
              occ = 0.0;

              int3 read_x = offset + int3(read_dir.x, 0, 0) + (proc_abs.x == 0 ? major_axis * read_dir : int3(0));
              int3 read_y = offset + int3(0, read_dir.y, 0) + (proc_abs.y == 0 ? major_axis * read_dir : int3(0));
              int3 read_z = offset + int3(0, 0, read_dir.z) + (proc_abs.z == 0 ? major_axis * read_dir : int3(0));

              uint facing_x = get_facing(read_x - region_offset);
              if (facing_x == 0) {
                if (all((read_x >= int3(0))) && all((read_x < grid_size()))) {
                  occ += float4(dst_occlusion[params.occlusion_index][read_x]).r;
                  avg += 1.0;
                }
              } else {
                if (proc_abs.x != 0) { //do not occlude from voxels in the opposite octant
                  avg += 1.0;
                }
              }

              uint facing_y = get_facing(read_y - region_offset);
              if (facing_y == 0) {
                if (all((read_y >= int3(0))) && all((read_y < grid_size()))) {
                  occ += float4(dst_occlusion[params.occlusion_index][read_y]).r;
                  avg += 1.0;
                }
              } else {
                if (proc_abs.y != 0) {
                  avg += 1.0;
                }
              }

              uint facing_z = get_facing(read_z - region_offset);
              if (facing_z == 0) {
                if (all((read_z >= int3(0))) && all((read_z < grid_size()))) {
                  occ += float4(dst_occlusion[params.occlusion_index][read_z]).r;
                  avg += 1.0;
                }
              } else {
                if (proc_abs.z != 0) {
                  avg += 1.0;
                }
              }

              if (avg > 0.0) {
                occ /= avg;
              }
            }

            dst_occlusion[params.occlusion_index][offset] = (float4(occ)).x;
          }
        }
      }
    }

    AllMemoryBarrierWithGroupSync();
  }
  //bias solid voxels away

  if (!region_out_of_bounds) {
    for (uint i = invocation_idx; i < (OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2); i += 64) {
      int3 local_offset = int3(i % (OCCLUSION_SIZE * 2), (i / (OCCLUSION_SIZE * 2)) % (OCCLUSION_SIZE * 2), i / ((OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2)));
      int3 offset = region_offset + local_offset;

      if (all((offset >= int3(0))) && all((offset < grid_size()))) {
        uint facing = get_facing(local_offset);

        if (facing != 0) {
          //only work on solids

          int3 proc_pos = local_offset - int3(OCCLUSION_SIZE);
          proc_pos += select((proc_pos >= int3(0)), int3(1), int3(0));

          float avg = 0.0;
          float occ = 0.0;

          int3 read_dir = -sign(proc_pos);
          int3 read_dir_x = int3(read_dir.x, 0, 0);
          int3 read_dir_y = int3(0, read_dir.y, 0);
          int3 read_dir_z = int3(0, 0, read_dir.z);
          //solid
          uint3 facing_pos = (uint3(facing) >> uint3(0, 1, 2)) & uint3(1, 1, 1);
          uint3 facing_neg = (uint3(facing) >> uint3(3, 4, 5)) & uint3(1, 1, 1);
          bool3 read_valid = bool3(select((read_dir > int3(0)), facing_pos, facing_neg));

          //sides
          if (read_valid.x) {
            int3 read_offset = local_offset + read_dir_x;
            uint f = get_facing(read_offset);
            if (f == 0) {
              read_offset += region_offset;
              if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
                occ += float4(dst_occlusion[params.occlusion_index][read_offset]).r;
                avg += 1.0;
              }
            }
          }

          if (read_valid.y) {
            int3 read_offset = local_offset + read_dir_y;
            uint f = get_facing(read_offset);
            if (f == 0) {
              read_offset += region_offset;
              if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
                occ += float4(dst_occlusion[params.occlusion_index][read_offset]).r;
                avg += 1.0;
              }
            }
          }

          if (read_valid.z) {
            int3 read_offset = local_offset + read_dir_z;
            uint f = get_facing(read_offset);
            if (f == 0) {
              read_offset += region_offset;
              if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
                occ += float4(dst_occlusion[params.occlusion_index][read_offset]).r;
                avg += 1.0;
              }
            }
          }

          //adjacents

          if (all(read_valid.yz)) {
            int3 read_offset = local_offset + read_dir_y + read_dir_z;
            uint f = get_facing(read_offset);
            if (f == 0) {
              read_offset += region_offset;
              if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
                occ += float4(dst_occlusion[params.occlusion_index][read_offset]).r;
                avg += 1.0;
              }
            }
          }

          if (all(read_valid.xz)) {
            int3 read_offset = local_offset + read_dir_x + read_dir_z;
            uint f = get_facing(read_offset);
            if (f == 0) {
              read_offset += region_offset;
              if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
                occ += float4(dst_occlusion[params.occlusion_index][read_offset]).r;
                avg += 1.0;
              }
            }
          }

          if (all(read_valid.xy)) {
            int3 read_offset = local_offset + read_dir_x + read_dir_y;
            uint f = get_facing(read_offset);
            if (f == 0) {
              read_offset += region_offset;
              if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
                occ += float4(dst_occlusion[params.occlusion_index][read_offset]).r;
                avg += 1.0;
              }
            }
          }

          //diagonal

          if (all(read_valid)) {
            int3 read_offset = local_offset + read_dir;
            uint f = get_facing(read_offset);
            if (f == 0) {
              read_offset += region_offset;
              if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
                occ += float4(dst_occlusion[params.occlusion_index][read_offset]).r;
                avg += 1.0;
              }
            }
          }

          if (avg > 0.0) {
            occ /= avg;
          }

          dst_occlusion[params.occlusion_index][offset] = (float4(occ)).x;
        }
      }
    }
  }

  AllMemoryBarrierWithGroupSync();

  // There are no more barriers after this, so we can early return.
  if (region_out_of_bounds) {
    return;
  }

  for (uint i = invocation_idx; i < (OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2); i += 64) {
    int3 local_offset = int3(i % (OCCLUSION_SIZE * 2), (i / (OCCLUSION_SIZE * 2)) % (OCCLUSION_SIZE * 2), i / ((OCCLUSION_SIZE * 2) * (OCCLUSION_SIZE * 2)));
    int3 offset = region_offset + local_offset;

    if (all((offset >= int3(0))) && all((offset < grid_size()))) {
      uint facing = get_facing(local_offset);

      if (facing == 0) {
        int3 proc_pos = local_offset - int3(OCCLUSION_SIZE);
        proc_pos += select((proc_pos >= int3(0)), int3(1), int3(0));

        int3 proc_abs = abs(proc_pos);

        int3 read_dir = sign(proc_pos); //opposite direction
        int3 read_dir_x = int3(read_dir.x, 0, 0);
        int3 read_dir_y = int3(0, read_dir.y, 0);
        int3 read_dir_z = int3(0, 0, read_dir.z);
        //solid
        uint3 read_mask = select((read_dir > int3(0)), uint3(8, 16, 32), uint3(1, 2, 4)); //match positive with negative normals
        uint3 block_mask = select((read_dir < int3(0)), uint3(8, 16, 32), uint3(1, 2, 4)); //match positive with negative normals

        block_mask = uint3(0);

        float visible = 0.0;
        float occlude_total = 0.0;

        if (proc_abs.x < OCCLUSION_SIZE) {
          int3 read_offset = local_offset + read_dir_x;
          uint x_mask = get_facing(read_offset);
          if (x_mask != 0) {
            read_offset += region_offset;
            if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
              occlude_total += 1.0;
              if (bool(x_mask & read_mask.x) && !bool(x_mask & block_mask.x)) {
                visible += 1.0;
              }
            }
          }
        }

        if (proc_abs.y < OCCLUSION_SIZE) {
          int3 read_offset = local_offset + read_dir_y;
          uint y_mask = get_facing(read_offset);
          if (y_mask != 0) {
            read_offset += region_offset;
            if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
              occlude_total += 1.0;
              if (bool(y_mask & read_mask.y) && !bool(y_mask & block_mask.y)) {
                visible += 1.0;
              }
            }
          }
        }

        if (proc_abs.z < OCCLUSION_SIZE) {
          int3 read_offset = local_offset + read_dir_z;
          uint z_mask = get_facing(read_offset);
          if (z_mask != 0) {
            read_offset += region_offset;
            if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
              occlude_total += 1.0;
              if (bool(z_mask & read_mask.z) && !bool(z_mask & block_mask.z)) {
                visible += 1.0;
              }
            }
          }
        }

        //if near the cartesian plane, test in opposite direction too

        read_mask = select((read_dir < int3(0)), uint3(8, 16, 32), uint3(1, 2, 4)); //match negative with positive normals
        block_mask = select((read_dir > int3(0)), uint3(8, 16, 32), uint3(1, 2, 4)); //match negative with positive normals
        block_mask = uint3(0);

        if (proc_abs.x == 1) {
          int3 read_offset = local_offset - read_dir_x;
          uint x_mask = get_facing(read_offset);
          if (x_mask != 0) {
            read_offset += region_offset;
            if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
              occlude_total += 1.0;
              if (bool(x_mask & read_mask.x) && !bool(x_mask & block_mask.x)) {
                visible += 1.0;
              }
            }
          }
        }

        if (proc_abs.y == 1) {
          int3 read_offset = local_offset - read_dir_y;
          uint y_mask = get_facing(read_offset);
          if (y_mask != 0) {
            read_offset += region_offset;
            if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
              occlude_total += 1.0;
              if (bool(y_mask & read_mask.y) && !bool(y_mask & block_mask.y)) {
                visible += 1.0;
              }
            }
          }
        }

        if (proc_abs.z == 1) {
          int3 read_offset = local_offset - read_dir_z;
          uint z_mask = get_facing(read_offset);
          if (z_mask != 0) {
            read_offset += region_offset;
            if (all((read_offset >= int3(0))) && all((read_offset < grid_size()))) {
              occlude_total += 1.0;
              if (bool(z_mask & read_mask.z) && !bool(z_mask & block_mask.z)) {
                visible += 1.0;
              }
            }
          }
        }

        if (occlude_total > 0.0) {
          float occ = float4(dst_occlusion[params.occlusion_index][offset]).r;
          occ *= visible / occlude_total;
          dst_occlusion[params.occlusion_index][offset] = (float4(occ)).x;
        }
      }
    }
  }

#endif

#ifdef MODE_STORE

  int3 local = int3(local_id.xyz);
  int3 pos = int3(id.xyz);
  // store SDF
  uint4 p = uint4(src_positions[pos]);

  bool solid = false;
  float d;
  if (all(int3(p.xyz) == pos)) {
    //solid block
    d = 0;
    solid = true;
  } else {
    //distance block
    d = 1.0 + length(float3(p.xyz) - float3(pos));
  }

  d /= 255.0;

  dst_sdf[pos] = (float4(d)).x;

  // STORE OCCLUSION

  uint occlusion = 0;
  static const uint occlusion_shift[8] = {12, 8, 4, 0, 28, 24, 20, 16};
  for (int i = 0; i < 8; i++) {
    float occ = float4(src_occlusion[i][pos]).r;
    occlusion |= uint(clamp(occ * 15.0, 0.0, 15.0)) << occlusion_shift[i];
  }
  {
    int3 occ_pos = pos;
    occ_pos.z += params.cascade * params.grid_size;
    dst_occlusion[occ_pos] = (uint4(occlusion & 0xFFFF)).x;
    occ_pos.x += params.grid_size;
    dst_occlusion[occ_pos] = (uint4(occlusion >> 16)).x;
  }

  // STORE POSITIONS

  if (all(local == int3(0))) {
    store_position_count = 0; //base one stores as zero, the others wait
    store_from_index = 0;
  }

  AllMemoryBarrierWithGroupSync();

  if (solid) {
    uint index;
    InterlockedAdd(store_position_count, 1, index);
    // At least do the conversion work in parallel
    store_positions[index].position = uint((pos.x & 127) | ((pos.y & 127) << 7) | ((pos.z & 127) << 14));
    store_positions[index].position_high = uint((pos.x >> 7) | ((pos.z >> 7) << 1) | ((pos.y >> 7) << 2));

    //see around which voxels point to this one, add them to the list
    uint bit_index = 0;
    uint neighbour_bits = 0;
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          if (i == 0 && j == 0 && k == 0) {
            continue;
          }
          int3 npos = pos + int3(i, j, k);
          if (all((npos >= int3(0))) && all((npos < grid_size()))) {
            p = uint4(src_positions[npos]);
            if (all(int3(p.xyz) == pos)) {
              neighbour_bits |= (1 << bit_index);
            }
          }
          bit_index++;
        }
      }
    }

    uint rgb = uint4(src_color[pos]).r;
    uint facing = uint4(src_facing[pos]).r;

    store_positions[index].albedo = rgb >> 1; //store as it comes (555) to avoid precision loss (and move away the alpha bit)
    store_positions[index].albedo |= (facing & 0x3F) << 15; // store facing in bits 15-21

    store_positions[index].albedo |= neighbour_bits << 21; //store lower 11 bits of neighbors with remaining albedo
    store_positions[index].position |= (neighbour_bits >> 11) << 21; //store 11 bits more of neighbors with position

    store_positions[index].light = uint4(src_emission[pos]).r;
    store_positions[index].light_aniso = uint4(src_emission_aniso[pos]).r;
    //add neighbors
    store_positions[index].light |= (neighbour_bits >> 22) << 30; //store 2 bits more of neighbors with light
    store_positions[index].light_aniso |= (neighbour_bits >> 24) << 30; //store 2 bits more of neighbors with aniso
  }

  AllMemoryBarrierWithGroupSync();

  // global increment only once per group, to reduce pressure

  if (all(local == int3(0)) && store_position_count > 0) {
    InterlockedAdd(dispatch_data[0].total_count, store_position_count, store_from_index);
    uint end = store_from_index + store_position_count;
    uint capacity = field_status[0].solid_cell_capacity;
    if (end > capacity)
      InterlockedOr(field_status[0].failure_flags, 1u);
    uint group_count = (min(end, capacity) + 63) / 64;
    InterlockedMax(dispatch_data[0].x, group_count);
  }

  AllMemoryBarrierWithGroupSync();

  uint read_index = uint(local.z * 4 * 4 + local.y * 4 + local.x);
  uint write_index = store_from_index + read_index;

  if (read_index < store_position_count && write_index < field_status[0].solid_cell_capacity) {
    dst_process_voxels[write_index] = store_positions[read_index];
  }

  if (all(pos == int3(0))) {
    //this thread clears y and z
    dispatch_data[0].y = 1;
    dispatch_data[0].z = 1;
  }
#endif

}
