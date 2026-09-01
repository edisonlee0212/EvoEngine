import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;

struct CoarseEdgeCandidate {
  uint slice0;
  uint slice1;
  uint pair;
  uint padding;
};

[[vk::binding(5, 1)]] StructuredBuffer<uint, Std430DataLayout> segment_slices;
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

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint index : SV_DispatchThreadID) {
  if (index >= padded_count)
    return;
  CoarseEdgeCandidate candidate;
  candidate.slice0 = uint(-1);
  candidate.slice1 = uint(-1);
  candidate.pair = uint(-1);
  candidate.padding = 0;
  if (index < direct_pair_count) {
    SegmentPair pair = segment_pairs[index];
    Segment segment0 = segments[pair.segment0_handle];
    Segment segment1 = segments[pair.segment1_handle];
    uint slice0 = segment_slices[pair.segment0_handle];
    uint slice1 = segment_slices[pair.segment1_handle];
    if (slice0 != slice1 && segment0.group_index == segment1.group_index && pair.connectivity_integrity > 1e-6f &&
        pair.bend_twist_bundle_integrity > 1e-6f) {
      candidate.slice0 = min(slice0, slice1);
      candidate.slice1 = max(slice0, slice1);
      candidate.pair = index;
    }
  }
  candidates[index] = candidate;
}
