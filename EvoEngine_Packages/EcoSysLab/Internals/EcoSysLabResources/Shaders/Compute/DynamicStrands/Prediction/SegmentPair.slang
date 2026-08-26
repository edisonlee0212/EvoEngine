import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

static const float MAX_FUNGUS_DENSITY = 8.0;
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint pair_size;
  float time_step;
  float inv_time_step;
  float fungus_growth_rate;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint pair_index = baseID + local_index;
    if (pair_index >= pair_size)
      break;

    int segment0_handle = segment_pairs[pair_index].segment0_handle;
    int segment1_handle = segment_pairs[pair_index].segment1_handle;

    // carbon health will affect the strain limits for connectivity and bending twist
    float carbon_health0 = segments[segment0_handle].HC;
    float carbon_health1 = segments[segment1_handle].HC;
    float connection_health = min(carbon_health0, carbon_health1);

    // we need to amplify decay somehow or no effect is visible
    connection_health = 2.0 * connection_health - 1.0;

    // Modify limit here.
    // float3 bending_twist_bundle_strain_limit =
    // segment_pairs[pair_index].max_bending_twist_bundle_strain * connection_health;
    // float connectivity_strain_limit = segment_pairs[pair_index].max_connectivity_strain * connection_health;
    float3 bending_twist_bundle_strain_limit = segment_pairs[pair_index].max_bending_twist_bundle_strain;
    float connectivity_strain_limit = segment_pairs[pair_index].max_connectivity_strain;

    float strength = (segments[segment0_handle].strength + segments[segment1_handle].strength) * .5f;

    float3 s0p0 = segment_particle0s[segment0_handle].x;
    float3 s0p1 = segment_particle1s[segment0_handle].x;
    float3 s1p0 = segment_particle0s[segment1_handle].x;
    float3 s1p1 = segment_particle1s[segment1_handle].x;
    float l0 = segments[segment0_handle].rest_length;
    float l1 = segments[segment1_handle].rest_length;

    float3 segment0_center_position = (s0p0 + s0p1) * .5f;
    float3 segment1_center_position = (s1p0 + s1p1) * .5f;
    float current_distance = distance(segment0_center_position, segment1_center_position);
    float3 orginal_segment0_center_position =
        (segment_particle0s[segment0_handle].x0 + segment_particle1s[segment0_handle].x0) * .5f;
    float3 orginal_segment1_center_position =
        (segment_particle0s[segment1_handle].x0 + segment_particle1s[segment1_handle].x0) * .5f;
    float orginal_distance = distance(orginal_segment0_center_position, orginal_segment1_center_position);
    float bundle_strain = (current_distance - orginal_distance) / (l0 + l1) * 2.0f;

    float2 bend_twist_strain = bend_twist_strain(segments[segment0_handle].q, segments[segment1_handle].q,
                                               segment_pairs[pair_index].rest_darboux_vector);

    // bool is_connection = segment_pairs[pair_index].connectivity_integrity != 0.f &&
    // (segments[segment0_handle].next_handle == segment1_handle || segments[segment1_handle].next_handle ==
    // segment0_handle);

    float segment0_stretch_strain = (distance(s0p0, s0p1) - l0) / l0;
    float segment1_stretch_strain = (distance(s1p0, s1p1) - l1) / l1;
    float connectivity_strain = max(segment0_stretch_strain, segment1_stretch_strain);

    segment_pairs[pair_index].bending_twist_bundle_strain =
        segment_pairs[pair_index].bend_twist_bundle_integrity != 0.f
            ? float3(bend_twist_strain.x, bend_twist_strain.y, bundle_strain) * inv_time_step
            : float3(0.f);
    segment_pairs[pair_index].connectivity_strain = connectivity_strain * inv_time_step;
    segment_pairs[pair_index].bending_twist_bundle_strain_limit = bending_twist_bundle_strain_limit * strength;
    segment_pairs[pair_index].connectivity_strain_limit = connectivity_strain_limit * strength;
  }
}
