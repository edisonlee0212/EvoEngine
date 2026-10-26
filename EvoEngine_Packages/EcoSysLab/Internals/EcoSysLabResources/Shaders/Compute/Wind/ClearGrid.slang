struct Parameters {
  uint count;
  uint bucket_count;
  float cell_size;
  float padding;
};

[[vk::binding(1, 0)]] RWStructuredBuffer<uint> heads;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  if (id.x < parameters.bucket_count)
    heads[id.x] = 0xffffffffu;
}
