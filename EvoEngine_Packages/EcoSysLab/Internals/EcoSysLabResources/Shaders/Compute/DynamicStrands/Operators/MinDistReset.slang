import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;

#define INFINITY asfloat(0x7F800000)

[[vk::binding(0, 0)]] RWStructuredBuffer<int4, Std430DataLayout> min_distance_state;

[shader("compute")]
[numthreads(1, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  min_distance_state[0].x = asint(INFINITY);
  min_distance_state[0].y = 0xFFFFFFFF;
}