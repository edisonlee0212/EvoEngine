import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PC_APPLY {
  uint segment_size;
  float dt;
};

void clamp_and_apply(inout float3 x, float3 last_x, inout float3 v, float ub, float dt, float scale) {
  float extra_cap = 0.0;
  float b = (extra_cap > 0.0) ? min(ub, extra_cap) : ub;
  float3 dx = x - last_x;
  float l = length(dx);
  if (l > b && l > 0.0)
    dx *= (b / l);
  dx *= scale;
  x = last_x + dx;
  v = dx / dt;
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint it = 0; it < ITERATIONS_PER_TASK; ++it) {
    uint local_index = laneID + it * WORKGROUP_SIZE;
    uint si = baseID + local_index;
    if (si >= segment_size)
      break;

    Segment s = segments[si];

    float scale = (s.quasi_stable == 1) ? s.ground_damping : 1.0;

    clamp_and_apply(s.particle0.x, s.particle0.last_x, s.particle0.v, s.particle0.user_bound, dt, scale);

    clamp_and_apply(s.particle1.x, s.particle1.last_x, s.particle1.v, s.particle1.user_bound, dt, scale);

    segments[si] = s;
  }
}
