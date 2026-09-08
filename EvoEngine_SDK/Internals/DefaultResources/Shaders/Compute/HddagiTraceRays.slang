// Explicit HDDAGI traversal diagnostic; the walk is shared with probe transport.
import EvoEngine.HddagiTrace;

struct HddagiRay {
  float3 origin;
  uint cascade;
  float3 direction;
  float distance;
};
struct HddagiTraceParams {
  int3 grid;
  uint ray_count;
  uint cascade_count;
  uint fractional_bits;
  uint2 padding;
};
[[vk::binding(0, 0)]] Texture3D<uint2> voxels;
[[vk::binding(1, 0)]] [vk::image_format("r8ui")] Texture3D<uint> regions;
[[vk::binding(2, 0)]] ConstantBuffer<HddagiCascadeBlock> cascades;
[[vk::binding(3, 0)]] StructuredBuffer<HddagiRay> rays;
[[vk::binding(4, 0)]] RWStructuredBuffer<uint4> results;
[[vk::push_constant]] ConstantBuffer<HddagiTraceParams> params;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x >= params.ray_count)
    return;
  HddagiRay ray = rays[id.x];
  int3 cell, side;
  uint cascade, iterations;
  bool hit = HddagiTrace(voxels, regions, cascades, params.grid, params.cascade_count,
                        ray.origin, ray.direction, ray.cascade, params.fractional_bits,
                        cell, side, cascade, iterations, params.padding.x != 0 ? ray.distance : -1);
  results[id.x * 3] = uint4(uint3(cell), cascade);
  results[id.x * 3 + 1] = uint4(uint3(side), hit ? 1u : 0u);
  results[id.x * 3 + 2] = uint4(iterations, 0, 0, 0);
}
