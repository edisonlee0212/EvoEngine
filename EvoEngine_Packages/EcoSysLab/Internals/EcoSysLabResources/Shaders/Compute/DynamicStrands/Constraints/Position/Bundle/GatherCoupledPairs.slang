import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

struct PairState { float4 positional_lambda; float4 angular_lambda; };
struct PairCorrection {
  float4 segment0_position;
  float4 segment0_angular;
  float4 segment1_position;
  float4 segment1_angular;
};
[[vk::binding(0, 1)]] RWStructuredBuffer<PairState, Std430DataLayout> pair_states;
[[vk::binding(1, 1)]] RWStructuredBuffer<PairCorrection, Std430DataLayout> pair_corrections;
[[vk::push_constant]] cbuffer PushConstants {
  uint pair_begin; uint pair_count; uint segment_count; uint reset_lambdas;
  float inverse_time_step_squared; float position_compliance_scale; float bending_compliance_scale; float torsion_compliance_scale;
};
static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK = (COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  uint base_index = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  [[unroll]] for (uint task = 0; task < ITERATIONS_PER_TASK; ++task) {
    uint segment_index = base_index + lane_id + task * WORKGROUP_SIZE;
    if (segment_index >= segment_count)
      break;
    float3 position_delta = 0;
    float3 angular_delta = 0;
    [[unroll]] for (uint connection = 0; connection < BUNDLE_MAX_CONNECTION; ++connection) {
      int pair_index = segment_connection_handles[segment_index].handles[connection];
      if (pair_index < int(pair_begin))
        continue;
      SegmentPair pair = segment_pairs[pair_index];
      bool is_segment0 = pair.segment0_handle == int(segment_index);
      position_delta += is_segment0 ? pair_corrections[pair_index].segment0_position.xyz
                                    : pair_corrections[pair_index].segment1_position.xyz;
      angular_delta += is_segment0 ? pair_corrections[pair_index].segment0_angular.xyz
                                   : pair_corrections[pair_index].segment1_angular.xyz;
    }
    segment_particle0s[segment_index].x += position_delta;
    segment_particle1s[segment_index].x += position_delta;
    float angle = length(angular_delta);
    if (angle > 1e-8f)
      segments[segment_index].q = normalize(quat_mul(angle_axis(angle, angular_delta / angle), segments[segment_index].q));
    update_inertia_w(segments[segment_index].q, segments[segment_index].inertia_tensor,
                     segments[segment_index].inv_inertia_tensor, segments[segment_index].inertia_w,
                     segments[segment_index].inv_inertia_w);
  }
}
