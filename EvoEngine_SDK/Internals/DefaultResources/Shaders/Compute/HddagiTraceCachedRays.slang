import EvoEngine.HddagiCache;
import EvoEngine.HddagiTrace;
struct Ray { float3 origin; uint cascade; float3 direction; float distance; };
struct Params { int3 grid; uint ray_count; uint cascade_count; uint use_cache; uint2 padding; };
[[vk::push_constant]] ConstantBuffer<Params> params;
[[vk::binding(0, 0)]] Texture3D<uint2> voxels;
[[vk::binding(1, 0)]] [vk::image_format("r8ui")] Texture3D<uint> regions;
[[vk::binding(2, 0)]] ConstantBuffer<HddagiCascadeBlock> cascades;
[[vk::binding(3, 0)]] StructuredBuffer<Ray> rays;
[[vk::binding(4, 0)]] RWStructuredBuffer<uint4> results;
[[vk::binding(5, 0)]] [vk::image_format("r16ui")] Texture3D<uint> versions;
[[vk::binding(6, 0)]] [vk::image_format("r32ui")] RWTexture2DArray<uint> cache;
[[vk::binding(7, 0)]] [vk::image_format("r16ui")] RWTexture2DArray<uint> cache_versions;
[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x >= params.ray_count) return;
  uint width, height, layers;
  cache.GetDimensions(width, height, layers);
  Ray ray = rays[id.x];
  int3 cell; uint cascade, iterations; bool reused;
  bool hit = HddagiTraceCached(voxels, regions, versions, cache, cache_versions,
                               int3(id.x % width, id.x / width, 0), cascades, params.grid, params.cascade_count,
                               ray.origin, ray.direction, ray.cascade, params.use_cache != 0,
                               cell, cascade, iterations, reused);
  results[id.x * 2] = uint4(uint3(cell), cascade);
  results[id.x * 2 + 1] = uint4(hit ? 1u : 0u, reused ? 1u : 0u, iterations, 0);
}
