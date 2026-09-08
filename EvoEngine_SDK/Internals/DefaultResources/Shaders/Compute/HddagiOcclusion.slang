// Godot hddagi_preprocess.glsl::MODE_OCCLUSION, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Rectangular wrapping; retain reference RGBA4 packing and host Z-layer offset.
import EvoEngine.HddagiTrace;
#define REGION_SIZE 8
#define PROBE_CELLS 8
struct Params {
  int3 grid_size;
  uint cascade;
  int3 offset;
  int occlusion_offset;
  int3 region_world_pos;
  uint padding;
};
[[vk::push_constant]] ConstantBuffer<Params> params;
[[vk::binding(0, 0)]] Texture3D<uint> src_normal_bits;
[[vk::binding(1, 0)]][vk::image_format("r16ui")] RWTexture3D<uint> dst_occlusion;
groupshared float occlusion[REGION_SIZE * REGION_SIZE * REGION_SIZE * 4];
groupshared uint bit_normals[REGION_SIZE * REGION_SIZE * REGION_SIZE];
groupshared uint solid_cell_list[REGION_SIZE * REGION_SIZE * REGION_SIZE];
groupshared uint solid_cell_count;
void set_occlusion(int3 p_pos, int p_layer, float p_occlusion) {
  int occ_pos = p_pos.z * (REGION_SIZE * REGION_SIZE) + p_pos.y * REGION_SIZE + p_pos.x;
  occlusion[occ_pos * 4 + p_layer] = p_occlusion;
}
float get_occlusion(int3 p_pos, int p_layer) {
  int occ_pos = p_pos.z * (REGION_SIZE * REGION_SIZE) + p_pos.y * REGION_SIZE + p_pos.x;
  return occlusion[occ_pos * 4 + p_layer];
}
void set_bit_normal(int3 p_pos, uint p_normal) {
  int occ_pos = p_pos.z * (REGION_SIZE * REGION_SIZE) + p_pos.y * REGION_SIZE + p_pos.x;
  bit_normals[occ_pos] = p_normal;
}
uint get_bit_normal(int3 p_pos) {
  int occ_pos = p_pos.z * (REGION_SIZE * REGION_SIZE) + p_pos.y * REGION_SIZE + p_pos.x;
  return bit_normals[occ_pos];
}
int3 offset_to_pos(int p_offset) {
  return int3(p_offset % REGION_SIZE, (p_offset / REGION_SIZE) % REGION_SIZE, p_offset / (REGION_SIZE * REGION_SIZE));
}
static const uint2 group_size_offset[11] = {uint2(1, 0),    uint2(3, 1),    uint2(6, 4),   uint2(10, 10),
                                            uint2(15, 20),  uint2(21, 35),  uint2(28, 56), uint2(36, 84),
                                            uint2(42, 120), uint2(46, 162), uint2(48, 208)};
static const uint group_pos[256] = {
    0,      65536,  256,    1,      131072, 65792,  512,    65537,  257,    2,      196608, 131328, 66048,  768,
    131073, 65793,  513,    65538,  258,    3,      262144, 196864, 131584, 66304,  1024,   196609, 131329, 66049,
    769,    131074, 65794,  514,    65539,  259,    4,      327680, 262400, 197120, 131840, 66560,  1280,   262145,
    196865, 131585, 66305,  1025,   196610, 131330, 66050,  770,    131075, 65795,  515,    65540,  260,    5,
    393216, 327936, 262656, 197376, 132096, 66816,  1536,   327681, 262401, 197121, 131841, 66561,  1281,   262146,
    196866, 131586, 66306,  1026,   196611, 131331, 66051,  771,    131076, 65796,  516,    65541,  261,    6,
    458752, 393472, 328192, 262912, 197632, 132352, 67072,  1792,   393217, 327937, 262657, 197377, 132097, 66817,
    1537,   327682, 262402, 197122, 131842, 66562,  1282,   262147, 196867, 131587, 66307,  1027,   196612, 131332,
    66052,  772,    131077, 65797,  517,    65542,  262,    7,      459008, 393728, 328448, 263168, 197888, 132608,
    67328,  458753, 393473, 328193, 262913, 197633, 132353, 67073,  1793,   393218, 327938, 262658, 197378, 132098,
    66818,  1538,   327683, 262403, 197123, 131843, 66563,  1283,   262148, 196868, 131588, 66308,  1028,   196613,
    131333, 66053,  773,    131078, 65798,  518,    65543,  263,    459264, 393984, 328704, 263424, 198144, 132864,
    459009, 393729, 328449, 263169, 197889, 132609, 67329,  458754, 393474, 328194, 262914, 197634, 132354, 67074,
    1794,   393219, 327939, 262659, 197379, 132099, 66819,  1539,   327684, 262404, 197124, 131844, 66564,  1284,
    262149, 196869, 131589, 66309,  1029,   196614, 131334, 66054,  774,    131079, 65799,  519,    459520, 394240,
    328960, 263680, 198400, 459265, 393985, 328705, 263425, 198145, 132865, 459010, 393730, 328450, 263170, 197890,
    132610, 67330,  458755, 393475, 328195, 262915, 197635, 132355, 67075,  1795,   393220, 327940, 262660, 197380,
    132100, 66820,  1540,   327685, 262405, 197125, 131845, 66565,  1285,   262150, 196870, 131590, 66310,  1030,
    196615, 131335, 66055,  775};
[numthreads(64, 1, 1)] void main(uint3 gl_LocalInvocationID : SV_GroupThreadID, uint3 gl_WorkGroupID : SV_GroupID) {
  int invocation_idx = int(gl_LocalInvocationID.x);
  int3 workgroup = int3(gl_WorkGroupID.xyz);
  if (invocation_idx == 0) {
    solid_cell_count = 0;
  }
  GroupMemoryBarrierWithGroupSync();
  int cells_load_total = PROBE_CELLS * PROBE_CELLS * PROBE_CELLS;
  int group_total = 64;
  {
    int load_from = invocation_idx * cells_load_total / group_total;
    int load_to = (invocation_idx + 1) * cells_load_total / group_total;
    for (int i = load_from; i < load_to; i++) {
      int3 load_pos;
      load_pos.z = i / (PROBE_CELLS * PROBE_CELLS);
      load_pos.y = (i / PROBE_CELLS) % PROBE_CELLS;
      load_pos.x = i % PROBE_CELLS;
      int3 tex_load_pos = load_pos + int3(workgroup.xyz * PROBE_CELLS) + params.offset;
      uint n = src_normal_bits.Load(int4(tex_load_pos, 0));
      set_bit_normal(load_pos, n);
      if (n != 0) {
        uint index;
        InterlockedAdd(solid_cell_count, 1, index);
        solid_cell_list[index] = uint(load_pos.x | (load_pos.y << 7) | (load_pos.z << 14));
      }
    }
  }
  GroupMemoryBarrierWithGroupSync();
#define OCC_STEPS (PROBE_CELLS * 3 - 2)
#define OCC_HALF_STEPS (OCC_STEPS / 2)
  int probe_offset = params.occlusion_offset;
  int3 base_world_region = params.region_world_pos + (params.offset / REGION_SIZE) + workgroup;
  for (int step = 0; step < OCC_STEPS; step++) {
    bool shrink = step >= OCC_HALF_STEPS;
    int occ_step = shrink ? OCC_HALF_STEPS - (step - OCC_HALF_STEPS) - 1 : step;
    if (invocation_idx < group_size_offset[occ_step].x) {
      uint pv = group_pos[group_size_offset[occ_step].y + invocation_idx];
      int3 proc_abs = (int3(int(pv)) >> int3(0, 8, 16)) & int3(0xFF);
      if (shrink) {
        proc_abs = int3(PROBE_CELLS) - proc_abs - int3(1);
      }
      for (int i = 0; i < 4; i++) {
        int3 bits = (((int3(i + probe_offset) >> int3(0, 1, 2)) + base_world_region) & int3(1, 1, 1));
        int3 proc_sign = bits * 2 - 1;
        int3 offset = int3(PROBE_CELLS - 1) * (int3(1) - bits) + proc_abs * proc_sign;
        float occ;
        uint facing = get_bit_normal(offset);
        if (facing != 0) {
          occ = 0.0;
        } else if (step == 0) {
          occ = 1.0;
        } else {
          int3 read_dir = -proc_sign;
          float avg = 0.0;
          occ = 0.0;
          int3 read_x = offset + int3(read_dir.x, 0, 0);
          int3 read_y = offset + int3(0, read_dir.y, 0);
          int3 read_z = offset + int3(0, 0, read_dir.z);
          if (all((read_x >= int3(0))) && all((read_x < int3(PROBE_CELLS)))) {
            uint facing_x = get_bit_normal(read_x);
            if (facing_x == 0) {
              occ += get_occlusion(read_x, i);
            }
            avg += 1.0;
          }
          if (all((read_y >= int3(0))) && all((read_y < int3(PROBE_CELLS)))) {
            uint facing_y = get_bit_normal(read_y);
            if (facing_y == 0) {
              occ += get_occlusion(read_y, i);
            }
            avg += 1.0;
          }
          if (all((read_z >= int3(0))) && all((read_z < int3(PROBE_CELLS)))) {
            uint facing_z = get_bit_normal(read_z);
            if (facing_z == 0) {
              occ += get_occlusion(read_z, i);
            }
            avg += 1.0;
          }
          if (avg > 0.0) {
            occ /= avg;
          }
        }
        set_occlusion(offset, i, occ);
      }
    }
    GroupMemoryBarrierWithGroupSync();
  }
  int cell_from = invocation_idx * int(solid_cell_count) / group_total;
  int cell_to = (invocation_idx + 1) * int(solid_cell_count) / group_total;
  for (int cell_i = cell_from; cell_i < cell_to; cell_i++) {
    int3 offset = (int3(solid_cell_list[cell_i]) >> int3(0, 7, 14)) & (0x7F);
    uint facing = get_bit_normal(offset);
    for (int i = 0; i < 4; i++) {
      int3 bits = (((int3(i + probe_offset) >> int3(0, 1, 2)) + base_world_region) & int3(1, 1, 1));
      int3 proc_sign = bits * 2 - 1;
      int3 read_dir = -proc_sign;
      float avg = 0.0;
      float occ = 0.0;
      int3 read_dir_x = int3(read_dir.x, 0, 0);
      int3 read_dir_y = int3(0, read_dir.y, 0);
      int3 read_dir_z = int3(0, 0, read_dir.z);
      uint3 facing_neg = (uint3(facing) >> uint3(0, 2, 4)) & uint3(1, 1, 1);
      uint3 facing_pos = (uint3(facing) >> uint3(1, 3, 5)) & uint3(1, 1, 1);
      bool3 read_valid = bool3(select((read_dir > int3(0)), facing_pos, facing_neg));
      read_valid = select((offset + read_dir < int3(0)), bool3(false), read_valid);
      read_valid = select((offset + read_dir >= int3(PROBE_CELLS)), bool3(false), read_valid);
      if (read_valid.x) {
        int3 read_offset = offset + read_dir_x;
        uint f = get_bit_normal(read_offset);
        if (f == 0) {
          occ += get_occlusion(read_offset, i);
          avg += 1.0;
        }
      }
      if (read_valid.y) {
        int3 read_offset = offset + read_dir_y;
        uint f = get_bit_normal(read_offset);
        if (f == 0) {
          occ += get_occlusion(read_offset, i);
          avg += 1.0;
        }
      }
      if (read_valid.z) {
        int3 read_offset = offset + read_dir_z;
        uint f = get_bit_normal(read_offset);
        if (f == 0) {
          occ += get_occlusion(read_offset, i);
          avg += 1.0;
        }
      }
      if (all(read_valid.yz)) {
        int3 read_offset = offset + read_dir_y + read_dir_z;
        uint f = get_bit_normal(read_offset);
        if (f == 0) {
          occ += get_occlusion(read_offset, i);
          avg += 1.0;
        }
      }
      if (all(read_valid.xz)) {
        int3 read_offset = offset + read_dir_x + read_dir_z;
        uint f = get_bit_normal(read_offset);
        if (f == 0) {
          occ += get_occlusion(read_offset, i);
          avg += 1.0;
        }
      }
      if (all(read_valid.xy)) {
        int3 read_offset = offset + read_dir_x + read_dir_y;
        uint f = get_bit_normal(read_offset);
        if (f == 0) {
          occ += get_occlusion(read_offset, i);
          avg += 1.0;
        }
      }
      if (all(read_valid)) {
        int3 read_offset = offset + read_dir;
        uint f = get_bit_normal(read_offset);
        if (f == 0) {
          occ += get_occlusion(read_offset, i);
          avg += 1.0;
        }
      }
      if (avg > 0.0) {
        occ /= avg;
      }
      set_occlusion(offset, i, occ);
    }
  }
  GroupMemoryBarrierWithGroupSync();
  cell_from = invocation_idx * cells_load_total / group_total;
  cell_to = (invocation_idx + 1) * cells_load_total / group_total;
  for (int cell_i = cell_from; cell_i < cell_to; cell_i++) {
    int3 offset;
    offset.z = cell_i / (PROBE_CELLS * PROBE_CELLS);
    offset.y = (cell_i / PROBE_CELLS) % PROBE_CELLS;
    offset.x = cell_i % PROBE_CELLS;
    uint facing = get_bit_normal(offset);
    if (facing != 0) {
      continue;
    }
    for (int i = 0; i < 4; i++) {
      int3 bits = (((int3(i + probe_offset) >> int3(0, 1, 2)) + base_world_region) & int3(1, 1, 1));
      int3 proc_sign = bits * 2 - 1;
      int3 read_dir = -proc_sign;
      int3 read_dir_x = int3(read_dir.x, 0, 0);
      int3 read_dir_y = int3(0, read_dir.y, 0);
      int3 read_dir_z = int3(0, 0, read_dir.z);
      uint3 read_mask = select((read_dir > int3(0)), uint3(1, 4, 16), uint3(2, 8, 32));
      bool3 read_valid = bool3(true);
      read_valid = select((offset + read_dir < int3(0)), bool3(false), read_valid);
      read_valid = select((offset + read_dir >= int3(PROBE_CELLS)), bool3(false), read_valid);
      float visible = 0.0;
      float occlude_total = 0.0;
      if (read_valid.x) {
        uint x_mask = get_bit_normal(offset + read_dir_x);
        if (x_mask != 0) {
          occlude_total += 1.0;
          if (bool(x_mask & read_mask.x)) {
            visible += 1.0;
          }
        }
      }
      if (read_valid.y) {
        uint y_mask = get_bit_normal(offset + read_dir_y);
        if (y_mask != 0) {
          occlude_total += 1.0;
          if (bool(y_mask & read_mask.y)) {
            visible += 1.0;
          }
        }
      }
      if (read_valid.z) {
        uint z_mask = get_bit_normal(offset + read_dir_z);
        if (z_mask != 0) {
          occlude_total += 1.0;
          if (bool(z_mask & read_mask.z)) {
            visible += 1.0;
          }
        }
      }
      if (occlude_total > 0.0) {
        float occ = get_occlusion(offset, i);
        occ *= visible / occlude_total;
        set_occlusion(offset, i, occ);
      }
    }
  }
  GroupMemoryBarrierWithGroupSync();
  cell_from = invocation_idx * cells_load_total / group_total;
  cell_to = (invocation_idx + 1) * cells_load_total / group_total;
  for (int cell_i = cell_from; cell_i < cell_to; cell_i++) {
    int3 cell_pos;
    cell_pos.z = cell_i / (PROBE_CELLS * PROBE_CELLS);
    cell_pos.y = (cell_i / PROBE_CELLS) % PROBE_CELLS;
    cell_pos.x = cell_i % PROBE_CELLS;
    int3 cascade_pos = cell_pos + params.offset + workgroup * REGION_SIZE;
    cascade_pos = HddagiWrap(cascade_pos + params.region_world_pos * REGION_SIZE, params.grid_size);
    cascade_pos += int3(1);
    int3 store_pos = cascade_pos;
    store_pos.y += (params.grid_size.y + 2) * params.cascade;
    uint enc_occlusion = 0;
    for (int i = 0; i < 4; i++) {
      float occ = get_occlusion(cell_pos, i);
      enc_occlusion |= uint(clamp(occ * 0xF, 0, 0xF)) << (i * 4);
    }
    dst_occlusion[store_pos] = enc_occlusion;
    for (uint mask = 1; mask < 8; ++mask) {
      int3 selected = int3((uint3(mask) >> uint3(0, 1, 2)) & 1);
      if (any((selected != 0) && (cascade_pos != int3(1)) && (cascade_pos != params.grid_size)))
        continue;
      int3 delta = selected * select(cascade_pos == int3(1), params.grid_size, -params.grid_size);
      dst_occlusion[store_pos + delta] = enc_occlusion;
    }
  }
}
