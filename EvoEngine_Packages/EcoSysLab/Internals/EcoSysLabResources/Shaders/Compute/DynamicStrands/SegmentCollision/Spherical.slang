import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrandsPhysics;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  uint segment_size;
  float cell_size;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint segment_handle = baseID + local_index;

    if (segment_handle >= segment_size)
      break;
    SegmentData segment_data = segment_data_list[segment_handle];

    Segment segment = segments[segment_handle];
    float ground_damping = segment.ground_damping;
    float quasi_damping = segment.quasi_damping;
    if (segment.reach_ground == 1) {
      segments[segment_handle].ground_damping *= 0.99;
    }
    if (segment.quasi_stable == 1) {
      segments[segment_handle].quasi_damping *= 0.995;
    }
    Particle particle0 = segment.particle0;
    Particle particle1 = segment.particle1;

    int exert_fricton_0 = 0;
    int exert_fricton_1 = 0;

    float3 p0 = particle0.x;
    float3 p1 = particle1.x;
    float4 color = segment.color;

    float3 segment_center_position = (p0 + p1) * 0.5f;

    float3 segment_direction = normalize(p1 - p0);

    int3 coordinate;
    coordinate.x = int(segment_center_position.x / cell_size);
    coordinate.y = int(segment_center_position.y / cell_size);
    coordinate.z = int(segment_center_position.z / cell_size);
    int sum = 0;
    uint begin = segment_size;
    // begin = 0;
    uint end = 0;
    float3 position_offset = float3(0.0f, 0.0f, 0.0f);
    float position_offset_length = 0.0f;
    int group_index = segment.group_index;
    for (int x = -1; x <= 1; x++) {
      for (int y = -1; y <= 1; y++) {
        for (int z = -1; z <= 1; z++) {
          begin = segment_size;
          end = 0;
          int3 current_coordinate = coordinate + int3(x, y, z);
          uint cell_id = hash_cell_coordinate(current_coordinate);
          uint start_index = hashed_grid_cell_starts[cell_id].start_index;
          uint end_index = hashed_grid_cell_starts[cell_id].end_index;
          if (start_index < begin && start_index != 0xFFFFFFFF) {
            begin = start_index;
          }
          if (end_index > end && end_index != 0xFFFFFFFF) {
            end = end_index;
          }

          while (begin < end) {
            uint neighbor_segment_handle = hashed_grid_elements[begin].segment_handle;
            Segment neighbor_segment = segments[neighbor_segment_handle];
            if (neighbor_segment.group_index != group_index) {
              Particle neighbor_particle0 = neighbor_segment.particle0;
              Particle neighbor_particle1 = neighbor_segment.particle1;

              float4 neighbor_color = neighbor_segment.color;
              float3 neighbor_p0 = neighbor_particle0.x;
              float3 neighbor_p1 = neighbor_particle1.x;

              float3 neighbor_segment_center_position = (neighbor_p0 + neighbor_p1) * 0.5f;

              float3 diff = segment_center_position - neighbor_segment_center_position;
              float3 diff0 = particle0.x - neighbor_segment_center_position;
              float3 diff1 = particle1.x - neighbor_segment_center_position;
              // float diff_length = length(diff);
              // float diff_length0 = length(diff0);
              // float diff_length1 = length(diff1);
              float diff_length = length(scaleAlong(diff, segment_direction, 1.0));
              float diff_length0 = length(scaleAlong(diff0, segment_direction, 1.0));
              float diff_length1 = length(scaleAlong(diff1, segment_direction, 1.0));
              float3 direction = float3(0, 0, 0);
              float3 direction0 = float3(0, 0, 0);
              float3 direction1 = float3(0, 0, 0);
              if (diff_length < 0.01f && neighbor_segment.reach_ground == 1) {
                segments[segment_handle].quasi_stable = 1;
              }
              if (diff_length < 0.0009f && neighbor_segment.quasi_stable == 1) {  // 0.0008-0.001
                segments[segment_handle].quasi_stable = 1;
              }
              // if (true) {
              if (any(uint4(clamp(neighbor_color * 255.0, 0.0, 255.0)) != uint4(clamp(color * 255.0, 0.0, 255.0)))) {
                // if (diff_length > 0.f && neighbor_color != color) {
                if (diff_length > 0.f) {
                  direction = normalize(diff);
                }
                if (diff_length0 > 0.f) {
                  direction0 = normalize(diff0);
                }
                if (diff_length1 > 0.f) {
                  direction1 = normalize(diff1);
                }
                if (diff_length < 0.05) {
                  // position_offset += direction * max(0.0f, 0.05 - diff_length) * max(0.0f, 0.05 - diff_length) *
                  // max(0.0f, 0.05 - diff_length) * 200.0;
                  position_offset += direction * max(0.0f, 0.05 - diff_length) * max(0.0f, 0.05 - diff_length);
                  position_offset_length += max(0.0f, 0.05 - diff_length) * max(0.0f, 0.05 - diff_length);
                  if (diff_length < 0.02) {
                    segments[segment_handle].particle0.v =
                        scaleAlongDirectional(segments[segment_handle].particle0.v, -direction, 50 * diff_length);
                    segments[segment_handle].particle1.v =
                        scaleAlongDirectional(segments[segment_handle].particle1.v, -direction, 50 * diff_length);
                  }
                }
                if (diff_length0 < 0.02) {
                  // segments[segment_handle].particle0.v = scaleAlongDirectional(segments[segment_handle].particle0.v,
                  //                                                              -direction0, 50 * diff_length);  //
                  //                                                              0.01, 0.1
                  if (segments[segment_handle].quasi_stable == 1 && neighbor_segment.quasi_stable == 1 &&
                      diff_length0 < 0.001) {  // 0.0015
                    exert_fricton_0 = 1;
                  }
                }
                if (diff_length1 < 0.02) {
                  // segments[segment_handle].particle1.v =
                  //     scaleAlongDirectional(segments[segment_handle].particle1.v, -direction1, 50 * diff_length);
                  if (segments[segment_handle].quasi_stable == 1 && neighbor_segment.quasi_stable == 1 &&
                      diff_length1 < 0.001) {
                    exert_fricton_1 = 1;
                  }  // segments[segment_handle].quasi_stable == 1 && neighbor_segment.quasi_stable == 1 &&
                }
                sum++;
              }
            }
            begin++;
          }
        }
      }
    }
    if (true) {
      // if (false) {
      position_offset /= segment_size;
      position_offset *= 2.f * 40.f;  // 2.f

      position_offset_length /= segment_size;
      position_offset_length *= 2.f * 40.f * 30.f;
      float3 dv0 = float3(0.0);
      float3 dv1 = float3(0.0);
      if (segments[segment_handle].quasi_stable == 1) {
        // if (false){
        // segments[segment_handle].particle0.x += 0.6 * position_offset * ground_damping;
        // segments[segment_handle].particle1.x += 0.6 * position_offset * ground_damping;

        float3 up = float3(0.0, 1.0, 0.0);
        // segments[segment_handle].particle0.v += scaleAlong(position_offset * 20.f, up, 0.05);
        // segments[segment_handle].particle1.v += scaleAlong(position_offset * 20.f, up, 0.05);

        // segments[segment_handle].particle0.v += position_offset * 30.f;
        // segments[segment_handle].particle1.v += position_offset * 30.f;
        // segments[segment_handle].particle0.v *= 0.2 * ground_damping;
        // segments[segment_handle].particle1.v *= 0.2 * ground_damping;

        // segments[segment_handle].particle0.v =
        //     scaleAlongDirectional(segments[segment_handle].particle0.v, -position_offset, 0.0);
        // segments[segment_handle].particle1.v =
        //     scaleAlongDirectional(segments[segment_handle].particle1.v, -position_offset, 0.0);
        if (exert_fricton_0 == 1) {
          float friction = 0.8 * 9.8;
          friction = position_offset_length * 0.8f;
          dv0 += length(float3(segments[segment_handle].particle0.v)) > friction
                     ? normalize(segments[segment_handle].particle0.v) * friction
                     : segments[segment_handle].particle0.v;
          segments[segment_handle].particle0.v -= dv0;

          // segments[segment_handle].particle0.v = float3(0.0);
          //  segments[segment_handle].particle0.v = scaleAlong(
          //      segments[segment_handle].particle0.v, segment_direction, 0.2);
        }
        if (exert_fricton_1 == 1) {
          float friction = 0.8 * 9.8;
          friction = position_offset_length * 0.8f;
          dv1 += length(float3(segments[segment_handle].particle1.v)) > friction
                     ? normalize(segments[segment_handle].particle1.v) * friction
                     : segments[segment_handle].particle1.v;
          segments[segment_handle].particle1.v -= dv1;

          // segments[segment_handle].particle1.v = float3(0.0);
          //  segments[segment_handle].particle1.v =
          //      scaleAlong(segments[segment_handle].particle1.v, segment_direction, 0.2);
        }

        // if (exert_fricton_0 == 1 || exert_fricton_1 == 1)
        //{
        //   float3 J0 = dv0 / 2.0 / segments[segment_handle].inv_mass;
        //   float3 J1 = dv1 / 2.0 / segments[segment_handle].inv_mass;
        //   float3 r0 = -segments[segment_handle].particle0.x + segment_center_position;
        //   float3 r1 = -segments[segment_handle].particle1.x + segment_center_position;
        //   float3 dL = cross(r0, J0) + cross(r1, J1);
        //   segments[segment_handle].angular_v =
        //       (segments[segment_handle].inv_inertia_w *
        //       (segments[segment_handle].inertia_w * float4(segments[segment_handle].angular_v, 0.0) + float4(dL,
        //       0.0))).xyz;
        // }

      } else {
        dv0 += position_offset * 30.f;
        dv1 += position_offset * 30.f;
        segments[segment_handle].particle0.v += position_offset * 30.f;  // 30.0f;
        segments[segment_handle].particle1.v += position_offset * 30.f;
        // segments[segment_handle].particle0.x += position_offset;
        // segments[segment_handle].particle1.x += position_offset;

        // segments[segment_handle].particle0.v =
        //    scaleAlongDirectional(segments[segment_handle].particle0.v, -position_offset, 0.0);
        // segments[segment_handle].particle1.v =
        //    scaleAlongDirectional(segments[segment_handle].particle1.v, -position_offset, 0.0);
      }
    }
  }
}

/*while (begin < segment_size) {
 uint neighbor_segment_handle = hashed_grid_elements[begin].segment_handle;
 Segment neighbor_segment = segments[neighbor_segment_handle];
 if (neighbor_segment.group_index != group_index) {
   Particle neighbor_particle0 = neighbor_segment.particle0;
   Particle neighbor_particle1 = neighbor_segment.particle1;

   float4 neighbor_color = neighbor_segment.color;
   float3 neighbor_p0 = neighbor_particle0.x;
   float3 neighbor_p1 = neighbor_particle1.x;

   float3 neighbor_segment_center_position = (neighbor_p0 + neighbor_p1) * 0.5f;

   float3 diff = segment_center_position - neighbor_segment_center_position;
   float diff_length = length(diff);
   float3 direction = float3(0, 0, 0);
   if (true) {
     if (diff_length > 0.f) {
       direction = normalize(diff);
     }

     sum++;
     position_offset += direction * max(0.0f, cell_size - diff_length);
   }
 }
 begin++;
}
if (sum != 0) {
 position_offset /= sum;
 segments[segment_handle].particle0.x += position_offset;
 segments[segment_handle].particle1.x += position_offset;
}*/

/*while (begin < segment_size) {  // TODO: Fix the bug with end index
      uint neighbor_segment_handle = hashed_grid_elements[begin].segment_handle;
      Segment neighbor_segment = segments[neighbor_segment_handle];
      if (neighbor_segment.group_index != group_index) {
        Particle neighbor_particle0 = neighbor_segment.particle0;
        Particle neighbor_particle1 = neighbor_segment.particle1;

        float4 neighbor_color = neighbor_segment.color;
        float3 neighbor_p0 = neighbor_particle0.x;
        float3 neighbor_p1 = neighbor_particle1.x;

        float3 neighbor_segment_center_position = (neighbor_p0 + neighbor_p1) * 0.5f;

        float3 diff = segment_center_position - neighbor_segment_center_position;
        float diff_length = length(diff);
        float3 direction = float3(0, 0, 0);
        if (diff_length < 0.01f && neighbor_segment.reach_ground == 1) {
          segments[segment_handle].quasi_stable = 1;
        }
        if (diff_length < 0.002f && neighbor_segment.quasi_stable == 1) {
          segments[segment_handle].quasi_stable = 1;
        }
        //if (true) {
        if (neighbor_color != color) {
          //if (diff_length > 0.f && neighbor_color != color) {
          if (diff_length > 0.f) {
            direction = normalize(diff);
          }
          if (diff_length < 0.05)
          {
            //position_offset += direction * max(0.0f, 0.05 - diff_length) * max(0.0f, 0.05 - diff_length) * max(0.0f,
   0.05 - diff_length) * 200.0; position_offset += direction * max(0.0f, 0.05 - diff_length); if
   (segments[segment_handle].quasi_stable == 1 && neighbor_segment.quasi_stable == 1 && diff_length < 0.01) {
              segments[segment_handle].particle0.v =
                  scaleAlongDirectional(segments[segment_handle].particle0.v, -direction, 0.1);
              segments[segment_handle].particle1.v =
                  scaleAlongDirectional(segments[segment_handle].particle1.v, -direction, 0.1);
            }
          }
          sum++;
        }
      }
      begin++;
    }*/