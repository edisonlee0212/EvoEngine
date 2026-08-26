import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint pair_size;
  uint allow_disconnection;
  uint allow_breaking;
  float compression_strength_factor;

  uint tensile_disconnection;
  uint compression_disconnection;
  uint positional_breaking;
  uint rotational_breaking;

  float rod_strength_factor;
  float bundle_strength_factor;
  float boundary_strength_decay_factor;
  uint moisture_breaking_rod;

  uint pull_cubical;
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
    SegmentPair segment_pair = segment_pairs[pair_index];
    float connectivity_integrity = segment_pairs[pair_index].connectivity_integrity;
    float bend_twist_bundle_integrity = segment_pairs[pair_index].bend_twist_bundle_integrity;
    float3 bending_twist_bundle_strain_limit = segment_pair.bending_twist_bundle_strain_limit;
    float3 bending_twist_bundle_strain = segment_pairs[pair_index].bending_twist_bundle_strain;

    float connectivity_strain_limit = segment_pair.connectivity_strain_limit;
    float connectivity_strain = segment_pairs[pair_index].connectivity_strain;

    int segment0_handle = segment_pairs[pair_index].segment0_handle;
    int segment1_handle = segment_pairs[pair_index].segment1_handle;

    float shear_stretch_strain_limit_0 = segments[segment0_handle].shear_stretch_strain_limit;
    float shear_stretch_strain_limit_1 = segments[segment1_handle].shear_stretch_strain_limit;
    float bd = (segments[segment0_handle].boundary_distance + segments[segment1_handle].boundary_distance) / 2.0f;
    float3 pair_position = (segment_particle0s[segment0_handle].x +
                             segment_particle1s[segment0_handle].x +
                             segment_particle0s[segment1_handle].x +
                             segment_particle1s[segment1_handle].x) *
                         0.25f;

    float scaling_factor = 1.0f;
    float bundle_scaling_factor = 1.0f;
    float moisture = (segments[segment0_handle].moisture + segments[segment1_handle].moisture) / 2.0;
    scaling_factor = 1.0f;
    if (moisture_breaking_rod == 1) {
      scaling_factor = moisture * moisture * moisture * moisture * moisture * moisture;
    }
    //scaling_factor = 0.0004f + bd * 0.001;
    scaling_factor = max(0.0006f, scaling_factor); //currently disabled, for tree: 0.0003f to 0.001f elm: 0.0006f
    //scaling_factor = 0.0006f;
    //if (pair_position.y <= 0.21f && pair_position.z <= 0.06f)
    //{
    //   if (pull_cubical != 1)
    //  {
    //    scaling_factor *=
    //        rod_strength_factor + max(pair_position.z - 0.05, 0) / (0.06 - 0.05) * (1.0 - rod_strength_factor);
    //     bundle_scaling_factor *=
    //         bundle_strength_factor + max(pair_position.z - 0.05, 0) / (0.06 - 0.05) * (1.0 - bundle_strength_factor);
    //  } else if (any(segments[segment0_handle].color != segments[segment1_handle].color))
    //  {
    //    scaling_factor *= rod_strength_factor;
    //    bundle_scaling_factor *= bundle_strength_factor;
    //  }
    // } //ZY: Hack for trunk infection case
    if (pull_cubical != 1)
    {
      scaling_factor *= rod_strength_factor;
      bundle_scaling_factor *= bundle_strength_factor;
    } else if (any(segments[segment0_handle].color != segments[segment1_handle].color))
    {
      scaling_factor *= rod_strength_factor;
      bundle_scaling_factor *= bundle_strength_factor;
    }

    if (segments[segment0_handle].reach_ground == 0) {
      shear_stretch_strain_limit_0 *= 1.f * scaling_factor;
    }
    if (segments[segment1_handle].reach_ground == 0) {
      shear_stretch_strain_limit_1 *= 1.f * scaling_factor;
    }
    if (segments[segment0_handle].reach_ground == 0 || segments[segment1_handle].reach_ground == 0) {
      // connectivity_strain_limit *= 0.4f * scaling_factor * 1.25;
      // connectivity_strain_limit *= 0.0005f;
      connectivity_strain_limit *= max(5.f * scaling_factor / 6.f, 0.0004f);
      bending_twist_bundle_strain_limit.x *= 0.4f * scaling_factor;
      // if (segment_particle0s[segment0_handle].x[1] > 0.1)
      //{
      //   bending_twist_bundle_strain_limit.x *= 0.45f;
      // } //Disabled hacking for spruce case (0.25) and oak case (0.4 - 0.48)
      bending_twist_bundle_strain_limit.y *= 0.4f * scaling_factor;
      bending_twist_bundle_strain_limit.z *=
          (1.0f - bd * boundary_strength_decay_factor) * bundle_scaling_factor;  // for log
      // bending_twist_bundle_strain_limit.z *= bundle_strength_factor;
      // bending_twist_bundle_strain_limit.z *= max((1.0f - bd * 6.0f), 0.1f) * bundle_strength_factor; //for trunk
    }

    bool is_connection = segment_pairs[pair_index].connectivity_integrity != 0.f &&
                         (segments[segment0_handle].next_handle == segment1_handle ||
                          segments[segment1_handle].next_handle == segment0_handle);

    if (allow_disconnection == 1) {
      if (((tensile_disconnection == 1 && segment_pairs[pair_index].tensile_lock == 0) &&
           connectivity_strain > connectivity_strain_limit) ||
          ((compression_disconnection == 1 && segment_pairs[pair_index].compression_lock == 0) &&
           connectivity_strain < -connectivity_strain_limit * compression_strength_factor)) {
        connectivity_integrity = 0.0f;
      }
    }
    if (allow_breaking == 1) {
      if (is_connection) {
        if (((positional_breaking == 1 && segment_pairs[pair_index].positional_lock == 0) &&
             (abs(segments[segment0_handle].shear_stretch_strain) > shear_stretch_strain_limit_0 ||
              abs(segments[segment1_handle].shear_stretch_strain) > shear_stretch_strain_limit_1))

            || ((rotational_breaking == 1 && segment_pairs[pair_index].rotational_lock == 0) &&
                (abs(bending_twist_bundle_strain.x) > bending_twist_bundle_strain_limit.x ||
                 abs(bending_twist_bundle_strain.y) > bending_twist_bundle_strain_limit.y))) {
          bend_twist_bundle_integrity = 0.0f;
          // connectivity_integrity = 0.0f;
        }
      } else {
        bool bundle_breaking = false;
        bundle_breaking =
            bundle_breaking || ((tensile_disconnection == 1 && segment_pairs[pair_index].tensile_lock == 0) &&
                                bending_twist_bundle_strain.z > bending_twist_bundle_strain_limit.z);
        bundle_breaking =
            bundle_breaking ||
            ((compression_disconnection == 1 && segment_pairs[pair_index].compression_lock == 0) &&
             bending_twist_bundle_strain.z < -bending_twist_bundle_strain_limit.z * compression_strength_factor);

        if (((rotational_breaking == 1 && segment_pairs[pair_index].rotational_lock == 0) &&
             (abs(bending_twist_bundle_strain.x) > bending_twist_bundle_strain_limit.x ||
              abs(bending_twist_bundle_strain.y) > bending_twist_bundle_strain_limit.y)) ||
            ((positional_breaking == 1 && segment_pairs[pair_index].positional_lock == 0) && bundle_breaking)) {
          bend_twist_bundle_integrity = 0.0f;
        }
      }
    }

    segment_pairs[pair_index].bend_twist_bundle_integrity = bend_twist_bundle_integrity;
    segment_pairs[pair_index].connectivity_integrity = connectivity_integrity;
  }
}
