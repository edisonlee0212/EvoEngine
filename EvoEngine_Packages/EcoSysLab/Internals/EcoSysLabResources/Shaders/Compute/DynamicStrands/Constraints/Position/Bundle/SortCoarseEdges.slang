struct CoarseEdgeCandidate {
  uint slice0;
  uint slice1;
  uint pair;
  uint padding;
};

[[vk::binding(9, 1)]] RWStructuredBuffer<CoarseEdgeCandidate, Std430DataLayout> candidates;

[[vk::push_constant]] cbuffer PushConstants {
  uint direct_pair_count;
  uint padded_count;
  uint sort_stage;
  uint sort_pass;
  float inverse_time_step_squared;
  float position_compliance_scale;
  float bending_compliance_scale;
  float torsion_compliance_scale;
};

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

bool less_than(CoarseEdgeCandidate lhs, CoarseEdgeCandidate rhs) {
  return lhs.slice0 < rhs.slice0 || (lhs.slice0 == rhs.slice0 && lhs.slice1 < rhs.slice1);
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint index : SV_DispatchThreadID) {
  if (index >= padded_count)
    return;
  uint partner = index ^ sort_pass;
  if (partner <= index || partner >= padded_count)
    return;
  bool ascending = (index & sort_stage) == 0;
  CoarseEdgeCandidate first = candidates[index];
  CoarseEdgeCandidate second = candidates[partner];
  if (less_than(second, first) == ascending) {
    candidates[index] = second;
    candidates[partner] = first;
  }
}
