import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint segment_size;
  float dt;
  float aw;
  float ab;

  float bw;
  float bb;
  float ycw;
  float ycb;

  float ylw;
  float pc;
  float pl;
  float k;

  float delta;
  float ll;
  float lc;
  float bo;

  float kc;
  float brw;
  float brb;
  float msr;  // moisture spread rate

  float bd_offset;
  float cpb;
  float cpw;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint segment_index = baseID + local_index;
    float heart_w_multiplier = 1.0;

    if (segment_index >= segment_size)
      break;

    // Calculate K based on boundary distance
    //segment_biology[segment_index].K = (0.2 + bo) / (16.0 * bo * segments[segment_index].boundary_distance + 1.0);d
    //segment_biology[segment_index].K = 5.0;
    segment_biology[segment_index].K = k;
    //if (segments[segment_index].boundary_distance > 0.13) {
    //  segment_biology[segment_index].K = 0.0;
    //}  // Inner fast decay
    if (segments[segment_index].boundary_distance >
             bd_offset + segment_particle0s[segment_index].root_distance / 40.0 ) {
      segment_biology[segment_index].K = 0.0;
    }  // Inner faster decay considering root distance for TREE ELM (0.02) SPRUCE (0.015) OAK (0.02)
    //segment_biology[segment_index].K = 1.0; // for board case
    //segment_biology[segment_index].K = k; // ZY: for competition(0.4) & log(0.2)

    // Get diffusion values from the segment
    float diffusion_c = segment_biology[segment_index].diffusion_c;
    float diffusion_w = segment_biology[segment_index].diffusion_w;
    float diffusion_b = segment_biology[segment_index].diffusion_b;
    float diffusion_m = segment_biology[segment_index].diffusion_m;

    // Update chemical protection (C)
    segment_biology[segment_index].C = segment_biology[segment_index].C_pre +
                                dt * (0.1 * diffusion_c +
                                      segment_biology[segment_index].K * (1.0 - segment_biology[segment_index].C_pre) *
                                      (cpw * segment_biology[segment_index].RW_pre + cpb * segment_biology[segment_index].RB_pre) *
                                      segment_biology[segment_index].HC_pre -
                                      delta * segment_biology[segment_index].C_pre);

    // Clamp to valid range
    segment_biology[segment_index].C = clamp(segment_biology[segment_index].C, 0.0, 1.0);

    // Update health of carbon (HC)
    segment_biology[segment_index].HC = segment_biology[segment_index].HC_pre +
                                 dt * ((-ycw * segment_biology[segment_index].RW_pre - ycb * segment_biology[segment_index].RB_pre) *
                                           segment_biology[segment_index].HC_pre +
                                       pc * segment_biology[segment_index].C_pre * (1.0 - segment_biology[segment_index].HC_pre));

    segment_biology[segment_index].HC = clamp(segment_biology[segment_index].HC, 0.0, 1.0);

    // Update health of lignin (HL)
    segment_biology[segment_index].HL = segment_biology[segment_index].HL_pre +
                                 dt * ((-ylw * segment_biology[segment_index].RW_pre) * segment_biology[segment_index].HL_pre +
                                       pl * segment_biology[segment_index].C_pre * (1.0 - segment_biology[segment_index].HL_pre));

    segment_biology[segment_index].HL = clamp(segment_biology[segment_index].HL, 0.0, 1.0);

    // Update white rot (RW)
    segment_biology[segment_index].RW =
        segment_biology[segment_index].RW_pre +
        dt * (aw * (brw + (1.0 - brw) * segment_biology[segment_index].moisture_pre) * segment_biology[segment_index].RW_pre *
                  (1.0 - segment_biology[segment_index].RW_pre) *
                  (ll * segment_biology[segment_index].HL_pre + lc * segment_biology[segment_index].HC_pre) *
                  segment_biology[segment_index].HC_pre / (kc + segment_biology[segment_index].HC_pre) -
              bw * segment_biology[segment_index].C_pre * segment_biology[segment_index].RW_pre + 0.1 * diffusion_w);

    segment_biology[segment_index].RW = clamp(segment_biology[segment_index].RW, 0.0, 1.0);

    // Update brown rot (RB)
    segment_biology[segment_index].RB =
        segment_biology[segment_index].RB_pre +
        dt * (ab * (brb + (1.0 - brb) * segment_biology[segment_index].moisture_pre) * segment_biology[segment_index].RB_pre *
                  (1.0 - segment_biology[segment_index].RB_pre) * segment_biology[segment_index].HC_pre -
              bb * segment_biology[segment_index].C_pre * segment_biology[segment_index].RB_pre + 0.1 * diffusion_b);

    segment_biology[segment_index].RB = clamp(segment_biology[segment_index].RB, 0.0, 1.0);

    segment_biology[segment_index].moisture = segment_biology[segment_index].moisture_pre +
                                       dt * (msr * (1.0f + (1.0f - segment_biology[segment_index].HL_pre)) * diffusion_m);

    if (segments[segment_index].boundary_distance < 0.01) {
      segment_biology[segment_index].moisture -= dt * segment_biology[segment_index].moisture_pre * 0.5f * 0.6f;
    }

    // for log
    // segment_biology[segment_index].moisture =
    //     segment_biology[segment_index].moisture_pre +
    //     dt * (0.1 * diffusion_m +
    //           25.0 * (1.0 - segment_biology[segment_index].moisture_pre) * max(0.25 -
    //           segments[segment_index].boundary_distance, 0.0) *
    //               (segment_biology[segment_index].HL_pre));
    //
    // if (segments[segment_index].boundary_distance < 0.03) {
    //   segment_biology[segment_index].moisture -= dt * segment_biology[segment_index].moisture_pre * 0.1 *
    //                                       (1.0 - segment_biology[segment_index].HL_pre + 0.1);
    // }
    // if (segment_particle0s[segment_index].x[0] < 0.03 || segment_particle0s[segment_index].x[0] > 0.47) {
    //   segment_biology[segment_index].moisture -= 50.0 * dt * (segment_biology[segment_index].moisture_pre + 0.2) *
    //                                       (1.0 - segment_biology[segment_index].HL_pre + 0.05) *
    //                                       (segments[segment_index].boundary_distance + 0.03);
    // }
    segment_biology[segment_index].moisture = clamp(segment_biology[segment_index].moisture, 0.0, 1.0);

    // Update previous values for next iteration
    segment_biology[segment_index].C_pre = segment_biology[segment_index].C;
    segment_biology[segment_index].HC_pre = segment_biology[segment_index].HC;
    segment_biology[segment_index].HL_pre = segment_biology[segment_index].HL;
    segment_biology[segment_index].RW_pre = segment_biology[segment_index].RW;
    segment_biology[segment_index].RB_pre = segment_biology[segment_index].RB;
    segment_biology[segment_index].moisture_pre = segment_biology[segment_index].moisture;
    segment_biology[segment_index].diffusion_c = 0.0;
    segment_biology[segment_index].diffusion_w = 0.0;
    segment_biology[segment_index].diffusion_b = 0.0;
    segment_biology[segment_index].diffusion_m = 0.0;
    segment_biology[segment_index].pairs_count = 0;
    // segments[segment_index].shear_stretch_alpha =
    //     1.f / (segments[segment_index].max_young_modulus * 3.1415926535f * segments[segment_index].radius *
    //            segments[segment_index].radius / segments[segment_index].rest_length) *
    //     (1.0f - 0.5 * segment_biology[segment_index].HC);
    //
    // segment_biology[segment_index].moisture = 0.2 + 0.4 * float(segments[segment_index].reach_ground);
    //
    // if (segments[segment_index].quasi_stable == 1)
    //{
    //   segment_particle0s[segment_index].v = float3(0.0);
    //   segment_particle1s[segment_index].v = float3(0.0);
    // }
  }
}
