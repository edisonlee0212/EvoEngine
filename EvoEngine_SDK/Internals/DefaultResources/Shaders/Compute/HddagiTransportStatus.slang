struct Params {
  uint cascade;
  uint capacity;
  uint generation;
  uint padding;
};
[[vk::push_constant]] ConstantBuffer<Params> params;
[[vk::binding(0, 0)]] StructuredBuffer<uint> dispatch_data;
[[vk::binding(1, 0)]] RWStructuredBuffer<uint> status;
[numthreads(1, 1, 1)] void main() {
#if defined(MODE_BEGIN)
  uint failure = params.cascade == 0 ? 0 : status[1];
  if (params.cascade == 0) {
    status[3] = status[2];
    status[0] = params.generation;
    status[1] = 0;
    status[2] = 0;
  }
  status[1] = failure | dispatch_data[4] | (dispatch_data[3] > params.capacity ? 1u : 0u);
#else
  status[2] = status[1] == 0 ? 1 : 0;
#endif
}
