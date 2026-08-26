import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

static const float FUNGUS_GROWTH_RATIO = 0.1;
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint pair_size;
  float be;
  float lignin_threshold;
  uint global_parameter;
  float HC_threshold;
  float HL_threshold;
  uint treespace;
  float padding1;
  float4 matrixAw[3];
  float4 matrixAb[3];
  float4 matrixAc[3];
  float4 matrixAm[3];
};

float3x3 unpackMat3(float4 c0, float4 c1, float4 c2) {
  return float3x3(c0.xyz, c1.xyz, c2.xyz);
}

// void computeDiffusionWeights(float3 source_pos, float3 target_pos, float source_distance, float target_distance, float
// max_b, float max_w,
//                              out float dc_weight, out float dw_weight, out float db_weight, out float dm_weight) {
//   float3 edge_dir = source_pos - target_pos;  // TODO: General space for the tree
//   float d = length(edge_dir);
//
//   if (d < 0.000001) {
//     dc_weight = 0.0;
//     dw_weight = 0.0;
//     db_weight = 0.0;
//     dm_weight = 0.0;
//     return;
//   }
//
//   edge_dir /= d;
//
//   float3x3 matrixAw = unpackMat3(matrixAw[0], matrixAw[1], matrixAw[2]);
//   float3x3 matrixAb = unpackMat3(matrixAb[0], matrixAb[1], matrixAb[2]);
//   float3x3 matrixAc = unpackMat3(matrixAc[0], matrixAc[1], matrixAc[2]);
//   //float3x3 matrixAm = float3x3(2.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1);
//   float3x3 matrixAm = float3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
//
//   // Calculate quadratic forms using matrix multiplication
//   float kw = max(dot(mul(edge_dir, matrixAw), edge_dir), 0.0);
//   float kb = max(dot(mul(edge_dir, matrixAb), edge_dir), 0.0);
//   float kc = max(dot(mul(edge_dir, matrixAc), edge_dir), 0.0);
//   float km = max(dot(mul(edge_dir, matrixAm), edge_dir), 0.0);
//
//   float coef = min(source_distance, target_distance);
//   if (be < 0.001) {
//     coef = 1.0;
//   } else {
//     //coef = pow(4.0 * (coef + 0.01), be);
//     coef = pow(4.0 * (coef + 0.01), 0.0);//moisture test
//     // coef = 1.0;
//   }
//
//   dw_weight = kw / d * coef;
//   db_weight = kb / d * coef;
//   dc_weight = kc / d;
//   dm_weight = km / d * coef;
// }

void computeDiffusionWeights(float3 source_pos, float3 target_pos, float source_distance, float target_distance, float3 ob_w,
                             float3 ob_b, float3 ob_c, float3 ob_m, float max_b, float max_w, out float dc_weight,
                             out float dw_weight, out float db_weight, out float dm_weight) {
  float3 dvec = target_pos - source_pos;
  float d2 = dot(dvec, dvec);

  if (d2 < 1e-12) {
    dc_weight = 0.0;
    dw_weight = 0.0;
    db_weight = 0.0;
    dm_weight = 0.0;
    return;
  }

  float3x3 M_w = float3x3(float3(ob_w.x, 0.0, 0.0), float3(0.0, ob_w.y, 0.0), float3(0.0, 0.0, ob_w.z));
  float3x3 M_b = float3x3(float3(ob_b.x, 0.0, 0.0), float3(0.0, ob_b.y, 0.0), float3(0.0, 0.0, ob_b.z));
  float3x3 M_c = float3x3(float3(ob_c.x, 0.0, 0.0), float3(0.0, ob_c.y, 0.0), float3(0.0, 0.0, ob_c.z));
  float3x3 M_m = float3x3(float3(ob_m.x, 0.0, 0.0), float3(0.0, ob_m.y, 0.0), float3(0.0, 0.0, ob_m.z));

  const float epsK = 1e-4;
  // float3x3 M_w = float3x3(100.0, 0.0, 0.0, 0.0, 0.20, 0.0, 0.0, 0.0, 0.20);
  // float3x3 M_b = float3x3(100.0, 0.0, 0.0, 0.0, 0.20, 0.0, 0.0, 0.0, 0.20); //log
  if (global_parameter == 1) {
    M_w = unpackMat3(matrixAw[0], matrixAw[1], matrixAw[2]);
    M_b = unpackMat3(matrixAb[0], matrixAb[1], matrixAb[2]);
    M_c = unpackMat3(matrixAc[0], matrixAc[1], matrixAc[2]);
    M_m = unpackMat3(matrixAm[0], matrixAm[1], matrixAm[2]);
  }

  float rho2_w = max(dot(mul(dvec, M_w), dvec), 0.0);
  float rho2_b = max(dot(mul(dvec, M_b), dvec), 0.0);
  float rho2_c = max(dot(mul(dvec, M_c), dvec), 0.0);
  float rho2_m = max(dot(mul(dvec, M_m), dvec), 0.0);

  float h_w = 0.05;  // e.g. uniform float
  float h_b = 0.05;
  float h_c = 0.05;
  float h_m = 0.05;

  float kw = exp(-rho2_w / (h_w * h_w));
  float kb = exp(-rho2_b / (h_b * h_b));
  float kc = exp(-rho2_c / (h_c * h_c));
  float km_loc = exp(-rho2_m / (h_m * h_m));

  float coef;
  float dist_var = max(source_distance, target_distance);
  if (be < 0.001) {
    coef = 1.0;
  } else {
    // coef = pow(4.0 * (dist_var + 0.01), be);
    coef = pow(dist_var * 10.0 + 0.01, be);
  }
  coef = 1.0f; //ZY: for board

  float multiplier = 20.0f;
  dw_weight = kw * coef * multiplier;
  db_weight = kb * coef * multiplier;
  dc_weight = kc;
  dm_weight = km_loc * multiplier;

  if (max_b < 0.01) {
    db_weight = 0.0;
  }
  if (max_w < 0.01) {
    dw_weight = 0.0;
  }
}

void computeDiffusionWeights_Tspace(float3 source_pos, float3 target_pos, float source_distance, float target_distance,
                                    float3 l_dir, float2 source_profile, float2 target_profile, float2 source_polar,
                                    float2 target_polar, float3 ob_w, float3 ob_b, float3 ob_c, float3 ob_m, float max_b,
                                    float max_w, out float dc_weight, out float dw_weight, out float db_weight,
                                    out float dm_weight) {
  float3 dvec = target_pos - source_pos;
  float d2 = dot(dvec, dvec);
  float3 L_normal_world = normalize(l_dir);
  float2 R_normal_profile = normalize(source_profile);
  float2 T_normal_profile = float2(-R_normal_profile.y, R_normal_profile.x);
  float R_diff = abs(dot(target_profile - source_profile, R_normal_profile));
  float T_diff = abs(dot(target_profile - source_profile, T_normal_profile));
  float L_diff = abs(dot(dvec, L_normal_world));

  float3 Tspace_diff = float3(R_diff * 0.005, T_diff * 0.005, L_diff);
  float d2_Tspace = dot(Tspace_diff, Tspace_diff);

  if (d2_Tspace < 1e-12) {
    dc_weight = 0.0;
    dw_weight = 0.0;
    db_weight = 0.0;
    dm_weight = 0.0;
    return;
  }

  float3x3 M_w = float3x3(float3(ob_w.x, 0.0, 0.0), float3(0.0, ob_w.y, 0.0), float3(0.0, 0.0, ob_w.z));
  float3x3 M_b = float3x3(float3(ob_b.x, 0.0, 0.0), float3(0.0, ob_b.y, 0.0), float3(0.0, 0.0, ob_b.z));
  float3x3 M_c = float3x3(float3(ob_c.x, 0.0, 0.0), float3(0.0, ob_c.y, 0.0), float3(0.0, 0.0, ob_c.z));
  float3x3 M_m = float3x3(float3(ob_m.x, 0.0, 0.0), float3(0.0, ob_m.y, 0.0), float3(0.0, 0.0, ob_m.z));

  const float epsK = 1e-4;
  // float3x3 M_w = float3x3(100.0, 0.0, 0.0, 0.0, 0.20, 0.0, 0.0, 0.0, 0.20);
  // float3x3 M_b = float3x3(100.0, 0.0, 0.0, 0.0, 0.20, 0.0, 0.0, 0.0, 0.20); //log
  if (global_parameter == 1) {
    M_w = unpackMat3(matrixAw[0], matrixAw[1], matrixAw[2]);
    M_b = unpackMat3(matrixAb[0], matrixAb[1], matrixAb[2]);
    M_c = unpackMat3(matrixAc[0], matrixAc[1], matrixAc[2]);
    M_m = unpackMat3(matrixAm[0], matrixAm[1], matrixAm[2]);

     ////M_b = float3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 20.0); //For elm
     //M_b = float3x3(1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0);
     //M_w = float3x3(500.0, 0.0, 0.0, 0.0, 500.0, 0.0, 0.0, 0.0, 200.0);
     //M_c = float3x3(100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 100.0);
     //M_m = float3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
     //if ((source_distance + target_distance) < 2.0 * 0.06 && (source_distance + target_distance) > 2.0 * 0.03) {
     //  M_w = float3x3(800.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 200.0);
     //}
     //if ((source_distance + target_distance) < 2.0 * 0.1 && (source_distance + target_distance) > 2.0 * 0.08) {
     //  M_w = float3x3(800.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 200.0);
     //}
     //if (source_polar.y + target_polar.y > -21.0 * 3.1415926 / 24.0 &&
     //    source_polar.y + target_polar.y < -5.0 * 3.1415926 / 6.0 && (source_distance + target_distance) > 2.0 * 0.035)
     //    {
     //  M_w = float3x3(0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 200.0);
     //}
     //if ((source_distance + target_distance) > 2.0 * 0.13) {
     //  M_w = float3x3(0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0);
     //} //Anisotropic Case
  }

  float rho2_w = max(dot(mul(Tspace_diff, M_w), Tspace_diff), 0.0);
  float rho2_b = max(dot(mul(Tspace_diff, M_b), Tspace_diff), 0.0);
  float rho2_c = max(dot(mul(Tspace_diff, M_c), Tspace_diff), 0.0);
  float rho2_m = max(dot(mul(Tspace_diff, M_m), Tspace_diff), 0.0);

  float h_w = 0.05;  // e.g. uniform float
  float h_b = 0.05;
  float h_c = 0.05;
  float h_m = 0.05;

  float kw = exp(-rho2_w / (h_w * h_w));
  float kb = exp(-rho2_b / (h_b * h_b));
  float kc = exp(-rho2_c / (h_c * h_c));
  float km_loc = exp(-rho2_m / (h_m * h_m));

  float coef;
  float dist_var = max(source_distance, target_distance);
  if (be < 0.001) {
    coef = 1.0;
  } else {
    // coef = pow(4.0 * (dist_var + 0.01), be);
    coef = pow(dist_var * 10.0 + 0.01, be);
  }

  float multiplier = 20.0f;
  dw_weight = kw * coef * multiplier;
  db_weight = kb * coef * multiplier;
  dc_weight = kc;
  dm_weight = km_loc * multiplier;

  if (max_b < 0.01) {
    db_weight = 0.0;
  }
  if (max_w < 0.01) {
    dw_weight = 0.0;
  }
}

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint pair_index = baseID + local_index;
    if (pair_index >= pair_size)
      break;
    if (segment_pairs[pair_index].connectivity_integrity <= 0.0 &&
        segment_pairs[pair_index].bend_twist_bundle_integrity <= 0.0) {
      continue;
    }

    int segment0_handle = segment_pairs[pair_index].segment0_handle;
    int segment1_handle = segment_pairs[pair_index].segment1_handle;

    InterlockedAdd(segments[segment0_handle].pairs_count, 1);
    InterlockedAdd(segments[segment1_handle].pairs_count, 1);

    // Get segment positions and boundary distances
    float3 segment0_pos = (segment_particle0s[segment0_handle].x + segment_particle1s[segment0_handle].x) / 2.0;
    float3 segment1_pos = (segment_particle0s[segment1_handle].x + segment_particle1s[segment1_handle].x) / 2.0;
    float segment0_distance = segments[segment0_handle].boundary_distance;
    float segment1_distance = segments[segment1_handle].boundary_distance;
    float max_b = max(segments[segment0_handle].RB_pre, segments[segment1_handle].RB_pre);
    float max_w = max(segments[segment0_handle].RW_pre, segments[segment1_handle].RW_pre);
    float3 l_dir = (segment_particle1s[segment0_handle].x - segment_particle0s[segment0_handle].x +
                  segment_particle1s[segment1_handle].x - segment_particle0s[segment1_handle].x) /
                 2.0;
    float2 source_profile = segments[segment0_handle].profile_position;
    float2 target_profile = segments[segment1_handle].profile_position;
    float2 source_polar = segments[segment0_handle].profile_polar_coordinate;
    float2 target_polar = segments[segment1_handle].profile_polar_coordinate;
    float3 ob_w = (segments[segment0_handle].Obstruction_w + segments[segment1_handle].Obstruction_w) / 2.0;
    float3 ob_b = (segments[segment0_handle].Obstruction_b + segments[segment1_handle].Obstruction_b) / 2.0;
    float3 ob_c = (segments[segment0_handle].Obstruction_c + segments[segment1_handle].Obstruction_c) / 2.0;
    float3 ob_m = (segments[segment0_handle].Obstruction_m + segments[segment1_handle].Obstruction_m) / 2.0;

    // Calculate diffusion weights
    float dc_weight, dw_weight, db_weight, dm_weight;
    if (treespace == 1) {
      computeDiffusionWeights_Tspace(segment0_pos, segment1_pos, segment0_distance, segment1_distance, l_dir,
                                     source_profile, target_profile, source_polar, target_polar, ob_w, ob_b, ob_c, ob_m,
                                     max_b, max_w, dc_weight, dw_weight, db_weight, dm_weight);
    } else {
      computeDiffusionWeights(segment0_pos, segment1_pos, segment0_distance, segment1_distance, ob_w, ob_b, ob_c, ob_m,
                              max_b, max_w, dc_weight, dw_weight, db_weight, dm_weight);
    }

    // Accumulate diffusion contribution from neighbors
    InterlockedAdd(segments[segment0_handle].diffusion_c, dc_weight * segments[segment1_handle].C_pre);
    InterlockedAdd(segments[segment0_handle].diffusion_w, dw_weight * segments[segment1_handle].RW_pre);
    InterlockedAdd(segments[segment0_handle].diffusion_b, db_weight * segments[segment1_handle].RB_pre);
    InterlockedAdd(segments[segment0_handle].diffusion_m, dm_weight * segments[segment1_handle].moisture_pre);

    // Add reverse direction contributions (undirected graph)
    InterlockedAdd(segments[segment1_handle].diffusion_c, dc_weight * segments[segment0_handle].C_pre);
    InterlockedAdd(segments[segment1_handle].diffusion_w, dw_weight * segments[segment0_handle].RW_pre);
    InterlockedAdd(segments[segment1_handle].diffusion_b, db_weight * segments[segment0_handle].RB_pre);
    InterlockedAdd(segments[segment1_handle].diffusion_m, dm_weight * segments[segment0_handle].moisture_pre);

    // Add diagonal terms (self-interactions with negative values for conservation)
    InterlockedAdd(segments[segment0_handle].diffusion_c, -dc_weight * segments[segment0_handle].C_pre);
    InterlockedAdd(segments[segment0_handle].diffusion_w, -dw_weight * segments[segment0_handle].RW_pre);
    InterlockedAdd(segments[segment0_handle].diffusion_b, -db_weight * segments[segment0_handle].RB_pre);
    InterlockedAdd(segments[segment0_handle].diffusion_m, -dm_weight * segments[segment0_handle].moisture_pre);

    InterlockedAdd(segments[segment1_handle].diffusion_c, -dc_weight * segments[segment1_handle].C_pre);
    InterlockedAdd(segments[segment1_handle].diffusion_w, -dw_weight * segments[segment1_handle].RW_pre);
    InterlockedAdd(segments[segment1_handle].diffusion_b, -db_weight * segments[segment1_handle].RB_pre);
    InterlockedAdd(segments[segment1_handle].diffusion_m, -dm_weight * segments[segment1_handle].moisture_pre);

    // if (segments[segment1_handle].HL < lignin_threshold || segments[segment0_handle].HL < lignin_threshold) {
    //   segment_pairs[pair_index].bend_twist_bundle_integrity = 0.0;
    //   segment_pairs[pair_index].connectivity_integrity = 0.0;
    // }

    // if (segments[segment0_handle].internal_pattern * segments[segment1_handle].internal_pattern == 0) {
    //  if (segments[segment0_handle].cube_pattern * segments[segment1_handle].cube_pattern == 1) {
    //    if (uint4(clamp(segments[segment1_handle].color * 255.0, 0.0, 255.0)) !=
    //        uint4(clamp(segments[segment0_handle].color * 255.0, 0.0, 255.0))) {
    //      float temp_boundary_limit = 0.3;
    //      float temp_health_limit = 0.1;
    //      if (segments[segment1_handle].HC <
    //              temp_health_limit + max(0.3 - segments[segment1_handle].boundary_distance, 0.0) ||
    //          segments[segment0_handle].HC <
    //              temp_health_limit + max(0.3 - segments[segment1_handle].boundary_distance, 0.0))  // Surface case
    //      {
    //        segment_pairs[pair_index].bend_twist_bundle_integrity = 0.0;
    //        segment_pairs[pair_index].connectivity_integrity = 0.0;
    //        segment_pairs[pair_index].connectivity_integrity = 0.0;
    //      }
    //    }
    //  } else {
    //    float temp_boundary_limit = 0.3;
    //    float temp_health_limit = 0.1;
    //    if (segments[segment1_handle].HC <
    //            temp_health_limit + max(0.3 - segments[segment1_handle].boundary_distance, 0.0) ||
    //        segments[segment0_handle].HC <
    //            temp_health_limit + max(0.3 - segments[segment1_handle].boundary_distance, 0.0)) {
    //      segment_pairs[pair_index].bend_twist_bundle_integrity = 0.0;
    //      segment_pairs[pair_index].connectivity_integrity = 0.0;
    //    }
    //  }
    //}

     if (true) {
         if (segments[segment1_handle].HL < HL_threshold || segments[segment0_handle].HL < HL_threshold) {
           segment_pairs[pair_index].bend_twist_bundle_integrity = 0.0;
           segment_pairs[pair_index].connectivity_integrity = 0.0;
         }
         if (any(uint4(clamp(segments[segment1_handle].color * 255.0, 0.0, 255.0)) !=
                 uint4(clamp(segments[segment0_handle].color * 255.0, 0.0, 255.0)))) {
           if (segments[segment1_handle].HC < HC_threshold &&
               segments[segment0_handle].HC < HC_threshold)  //ZY: Switch to || for non-tree cases
           {
             segment_pairs[pair_index].bend_twist_bundle_integrity = 0.0;
             segment_pairs[pair_index].connectivity_integrity = 0.0;
           }
         }
     } // For TREE ELM

    // if (segments[segment1_handle].color != segments[segment0_handle].color &&
    //     (segment_pairs[pair_index].connectivity_integrity > 0.0 ||
    //      segment_pairs[pair_index].bend_twist_bundle_integrity > 0.0))
    //{
    //   segments[segment1_handle].prev_inside = 0;
    //   segments[segment0_handle].prev_inside = 0;
    // }
    // if ((segments[segment1_handle].boundary_distance < 1.0 || segments[segment0_handle].boundary_distance < 1.0) &&
    //   uint4(clamp(segments[segment1_handle].color * 255.0, 0.0, 255.0)) !=
    //       uint4(clamp(segments[segment0_handle].color * 255.0, 0.0, 255.0))) {
    // segment_pairs[pair_index].bend_twist_bundle_integrity = 0.0;
    // segment_pairs[pair_index].connectivity_integrity = 0.0;
    // }

    /*if ((segments[segment1_handle].boundary_distance >
             0.01 + segment_particle0s[segment1_handle].root_distance / 40.0 ||
         segments[segment0_handle].boundary_distance >
             0.01 + segment_particle0s[segment0_handle].root_distance / 40.0)) {
      segment_pairs[pair_index].bend_twist_bundle_integrity = 0.0;
      segment_pairs[pair_index].connectivity_integrity = 0.0;

    }*/  // Test: Force break if different color near surface(Spruce)

    //  if ((segments[segment1_handle].boundary_distance >
    //         0.02 + segment_particle0s[segment1_handle].root_distance / 40.0 ||
    //     segments[segment0_handle].boundary_distance >
    //         0.02 + segment_particle0s[segment0_handle].root_distance / 40.0)) {
    //  segment_pairs[pair_index].bend_twist_bundle_integrity = 0.0;
    //  segment_pairs[pair_index].connectivity_integrity = 0.0;

    //}  // Test: Force break if different color near surface(Elm)

        //if ((segments[segment1_handle].boundary_distance >
        //      0.06 + segment_particle0s[segment1_handle].root_distance / 40.0 ||
        //     segments[segment0_handle].boundary_distance >
        //      0.06 + segment_particle0s[segment0_handle].root_distance / 40.0)) {
        //  segment_pairs[pair_index].bend_twist_bundle_integrity = 0.0;
        //  segment_pairs[pair_index].connectivity_integrity = 0.0;
        //}  // Test: Force break if different color near surface(Oak Trunk)

    if (all(uint4(clamp(segments[segment1_handle].color * 255.0, 0.0, 255.0)) ==
            uint4(clamp(segments[segment0_handle].color * 255.0, 0.0, 255.0)))) {
      if (segments[segment0_handle].reach_ground == 1 && (segment_pairs[pair_index].connectivity_integrity > 0.0 ||
                                                          segment_pairs[pair_index].bend_twist_bundle_integrity > 0.0))
        segments[segment1_handle].reach_ground = 1;
      if (segments[segment1_handle].reach_ground == 1 && (segment_pairs[pair_index].connectivity_integrity > 0.0 ||
                                                          segment_pairs[pair_index].bend_twist_bundle_integrity > 0.0))
        segments[segment0_handle].reach_ground = 1;
      if (segments[segment0_handle].quasi_stable == 1 && (segment_pairs[pair_index].connectivity_integrity > 0.0 ||
                                                          segment_pairs[pair_index].bend_twist_bundle_integrity > 0.0))
        segments[segment1_handle].quasi_stable = 1;
      if (segments[segment1_handle].quasi_stable == 1 && (segment_pairs[pair_index].connectivity_integrity > 0.0 ||
                                                          segment_pairs[pair_index].bend_twist_bundle_integrity > 0.0))
        segments[segment0_handle].quasi_stable = 1;
    }

    /*if (segments[segment0_handle].reach_ground == 1 && (segment_pairs[pair_index].connectivity_integrity > 0.0 ||
                                                        segment_pairs[pair_index].bend_twist_bundle_integrity > 0.0))
      segments[segment1_handle].reach_ground = 1;
    if (segments[segment1_handle].reach_ground == 1 && (segment_pairs[pair_index].connectivity_integrity > 0.0 ||
                                                        segment_pairs[pair_index].bend_twist_bundle_integrity > 0.0))
      segments[segment0_handle].reach_ground = 1;
    if (segments[segment0_handle].quasi_stable == 1 && (segment_pairs[pair_index].connectivity_integrity > 0.0 ||
                                                        segment_pairs[pair_index].bend_twist_bundle_integrity > 0.0))
      segments[segment1_handle].quasi_stable = 1;
    if (segments[segment1_handle].quasi_stable == 1 && (segment_pairs[pair_index].connectivity_integrity > 0.0 ||
                                                        segment_pairs[pair_index].bend_twist_bundle_integrity > 0.0))
      segments[segment0_handle].quasi_stable = 1;*/
  }
}
