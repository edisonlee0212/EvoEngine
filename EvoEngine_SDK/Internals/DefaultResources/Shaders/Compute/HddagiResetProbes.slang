// HDDAGI circular probe storage reset; layouts follow Godot da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Retained world probes keep history on camera scrolling.
import EvoEngine.HddagiTrace;
struct HddagiResetParams {
  int3 probe_size; uint cascade;
  int3 region_offset; uint history_size;
  int3 scroll; uint reset_all;
};
[[vk::push_constant]] ConstantBuffer<HddagiResetParams> params;
[[vk::binding(0, 0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> history;
[[vk::binding(1, 0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> history_sum;
[[vk::binding(2, 0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> diffuse;
[[vk::binding(3, 0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> specular;
[[vk::binding(4, 0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> filtered;
[[vk::binding(5, 0)]] [vk::image_format("rgba16f")] RWTexture2DArray<float4> ambient;
[[vk::binding(6, 0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> neighbors;
[[vk::binding(7, 0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> process_frame;
[[vk::binding(8, 0)]] [vk::image_format("r8")] RWTexture2DArray<float> proximity;
[[vk::binding(9, 0)]] [vk::image_format("r8")] RWTexture2DArray<float> camera_visibility;
[numthreads(4, 4, 4)]
void main(uint3 id : SV_DispatchThreadID) {
  if (any(id >= uint3(params.probe_size))) return;
  int3 previous = int3(id) + params.scroll / 8;
  if (params.reset_all == 0 && all(previous >= 0) && all(previous < params.probe_size)) return;
  int3 physical = HddagiWrap(int3(id) + params.region_offset, params.probe_size);
  int2 probe = physical.xy + int2(0, physical.z * params.probe_size.y);
  int3 destination = int3(probe, params.cascade);
  ambient[destination] = float4(0);
  neighbors[destination] = 0;
  process_frame[destination] = 0;
  proximity[destination] = 0;
  camera_visibility[destination] = 0;
  for (int y = 0; y < 7; ++y)
    for (int x = 0; x < 7; ++x) {
      int3 pixel = int3(probe * 7 + int2(x, y), params.cascade);
      diffuse[pixel] = 0;
      specular[pixel] = 0;
      filtered[pixel] = 0;
    }
  for (int y = 0; y < 5; ++y)
    for (int x = 0; x < 5; ++x) {
      int2 pixel = probe * 5 + int2(x, y);
      for (uint phase = 0; phase < params.history_size; ++phase)
        history[int3(pixel, params.cascade * params.history_size + phase)] = 0;
      for (int channel = 0; channel < 3; ++channel)
        history_sum[int3(pixel.x * 3 + channel, pixel.y, params.cascade)] = 0;
    }
}
