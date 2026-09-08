// EvoEngine occupancy-preserving edit extension. Packing matches Godot sdfgi_preprocess.glsl::MODE_STORE,
// 34d06658a85845111a50db9e485ec4a0701d4298. See docs/licenses/Godot-MIT.txt.
import EvoEngine.SdfgiTypes;

[[vk::binding(1, 0)]] [vk::image_format("r16ui")] RWTexture3D<uint> albedo;
[[vk::binding(2, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> facing;
[[vk::binding(3, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> emission;
[[vk::binding(4, 0)]] [vk::image_format("r32ui")] RWTexture3D<uint> emission_aniso;
[[vk::binding(5, 0)]] StructuredBuffer<SdfgiDispatchData> dispatch_data;
[[vk::binding(6, 0)]] RWStructuredBuffer<SdfgiSolidCell> cells;
[[vk::binding(7, 0)]] RWStructuredBuffer<SdfgiFieldStatus> field_status;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (field_status[0].failure_flags != 0 || id.x >= min(dispatch_data[0].total_count, field_status[0].solid_cell_capacity)) return;
  SdfgiSolidCell cell = cells[id.x];
  int3 p = SdfgiUnpackPosition(cell);
  uint color = albedo[p];
  // Godot MODE_STORE retains a zero-payload origin cell for an entirely empty cascade.
  if (dispatch_data[0].total_count == 1 && all(p == 0) && color == 0 && facing[p] == 0 &&
      (cell.albedo & 0x1fffffu) == 0 && (cell.light & 0x3fffffffu) == 0 &&
      (cell.light_aniso & 0x3fffffffu) == 0) return;
  if ((color & 1) == 0 || (facing[p] & 0x3f) != ((cell.albedo >> 15) & 0x3f)) {
    InterlockedOr(field_status[0].failure_flags, 4u);
    return;
  }
  cell.albedo = (cell.albedo & 0xffff8000u) | (color >> 1);
  cell.light = (cell.light & 0xc0000000u) | (emission[p] & 0x3fffffffu);
  cell.light_aniso = (cell.light_aniso & 0xc0000000u) | (emission_aniso[p] & 0x3fffffffu);
  cells[id.x] = cell;
}
