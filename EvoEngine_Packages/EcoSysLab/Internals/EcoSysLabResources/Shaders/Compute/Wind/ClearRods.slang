struct Parameters {
  uint particle_count;
  uint rod_count;
  uint bucket_count;
  float cell_size;
};

[[vk::binding(1, 0)]] RWStructuredBuffer<uint> rod_heads;
[[vk::binding(3, 0)]] RWStructuredBuffer<uint> max_rod_extent;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x < parameters.bucket_count) rod_heads[id.x] = 0xffffffffu;
  if (id.x == 0) max_rod_extent[0] = 0u;
}
