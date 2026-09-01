import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;

struct CoarseEdgeCandidate {
  uint slice0;
  uint slice1;
  uint pair;
  uint padding;
};

struct CoarseEdge {
  uint slice0;
  uint slice1;
  uint count;
  uint padding;
  float4 compliance_cohesion;
  float4 positional_lambda;
  float4 angular_lambda;
};

[[vk::binding(9, 1)]] StructuredBuffer<CoarseEdgeCandidate, Std430DataLayout> candidates;
[[vk::binding(10, 1)]] RWStructuredBuffer<CoarseEdge, Std430DataLayout> edges;
[[vk::binding(11, 1)]] RWStructuredBuffer<uint, Std430DataLayout> edge_count;

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

[shader("compute")]
[numthreads(1, 1, 1)]
void main() {
  uint output_count = 0;
  uint begin = 0;
  while (begin < direct_pair_count && candidates[begin].pair != uint(-1)) {
    CoarseEdgeCandidate first = candidates[begin];
    CoarseEdge edge = (CoarseEdge)0;
    edge.slice0 = first.slice0;
    edge.slice1 = first.slice1;
    uint end = begin;
    while (end < direct_pair_count && candidates[end].slice0 == first.slice0 &&
           candidates[end].slice1 == first.slice1) {
      SegmentPair pair = segment_pairs[candidates[end].pair];
      Segment segment0 = segments[pair.segment0_handle];
      Segment segment1 = segments[pair.segment1_handle];
      float cohesion = min(pair.connectivity_integrity, pair.bend_twist_bundle_integrity);
      cohesion *= saturate(min(min(segment0.HC, segment1.HC), min(segment0.HL, segment1.HL)));
      cohesion *= saturate(1.f - max(max(segment0.RW, segment1.RW), max(segment0.RB, segment1.RB)));
      cohesion *= saturate(min(segment0.moisture, segment1.moisture));
      edge.compliance_cohesion +=
          float4(.5f * (segment0.shear_stretch_alpha + segment1.shear_stretch_alpha), pair.bending_alpha,
                 pair.twisting_alpha, cohesion);
      ++edge.count;
      ++end;
    }
    edge.compliance_cohesion /= max(float(edge.count), 1.f);
    edges[output_count++] = edge;
    begin = end;
  }
  edge_count[0] = output_count;
}
