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

void closestPointsSegments(in float3 A, in float3 B, in float3 C, in float3 D, out float s, out float t, out float3 P, out float3 Q,
                           out float dist) {
  float3 u = B - A, v = D - C, w = A - C;
  float a = dot(u, u), b = dot(u, v), c = dot(v, v), d0 = dot(u, w), e = dot(v, w);
  float Dden = a * c - b * b;
  const float EPS = 1e-8;
  if (Dden < EPS) {
    s = clamp(-d0 / max(a, EPS), 0, 1);
    t = clamp((b * s - e) / max(c, EPS), 0, 1);
  } else {
    s = clamp((b * e - c * d0) / Dden, 0, 1);
    t = clamp((a * e - b * d0) / Dden, 0, 1);
  }
  P = A + s * u;
  Q = C + t * v;
  dist = length(P - Q);
}

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
    Particle segment_particle0 = segment_particle0s[segment_handle];
    Particle segment_particle1 = segment_particle1s[segment_handle];
    float ground_damping = segment.ground_damping;
    float quasi_damping = segment.quasi_damping;
    if (segment.reach_ground == 1) {
      segments[segment_handle].ground_damping *= 0.99;
    }
    if (segment.quasi_stable == 1) {
      segments[segment_handle].quasi_damping *= 0.995;
    }
    Particle particle0 = segment_particle0;
    Particle particle1 = segment_particle1;

    int exert_fricton_0 = 0;
    int exert_fricton_1 = 0;

    float3 p0 = particle0.x;
    float3 p1 = particle1.x;
    float4 color = segment.color;

    float3 segment_center_position = (p0 + p1) * 0.5f;

    int3 coordinate;
    coordinate.x = int(segment_center_position.x / cell_size);
    coordinate.y = int(segment_center_position.y / cell_size);
    coordinate.z = int(segment_center_position.z / cell_size);
    int sum = 0;
    uint begin = segment_size;
    uint end = 0;
    float3 position_offset = float3(0.0f, 0.0f, 0.0f);
    int group_index = segment.group_index;
    for (int x = -1; x <= 1; x++) {
      for (int y = -1; y <= 1; y++) {
        for (int z = -1; z <= 1; z++) {
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
        }
      }
    }
    while (begin < end) {
      uint neighbor_segment_handle = hashed_grid_elements[begin].segment_handle;
      Segment neighbor_segment = segments[neighbor_segment_handle];
      Particle neighbor_segment_particle0 = segment_particle0s[neighbor_segment_handle];
      Particle neighbor_segment_particle1 = segment_particle1s[neighbor_segment_handle];
      if (neighbor_segment.group_index != group_index) {
        Particle neighbor_particle0 = neighbor_segment_particle0;
        Particle neighbor_particle1 = neighbor_segment_particle1;

        float4 neighbor_color = neighbor_segment.color;
        float3 neighbor_p0 = neighbor_particle0.x;
        float3 neighbor_p1 = neighbor_particle1.x;

        float3 neighbor_segment_center_position = (neighbor_p0 + neighbor_p1) * 0.5f;

        float sP, tP, d;
        float3 Pi, Pj;

        closestPointsSegments(p0, p1, neighbor_p0, neighbor_p1, sP, tP, Pi, Pj, d);
        float diff_length = length(d);
        float3 diff = Pi - Pj;

        float3 diff0 = particle0.x - neighbor_segment_center_position;
        float3 diff1 = particle1.x - neighbor_segment_center_position;
        float diff_length0 = length(diff0);
        float diff_length1 = length(diff1);

        float3 direction = float3(0, 0, 0);

        if (diff_length < 0.01f && neighbor_segment.reach_ground == 1) {
          segments[segment_handle].quasi_stable = 1;
        }
        if (diff_length < 0.0015f && neighbor_segment.quasi_stable == 1) {
          segments[segment_handle].quasi_stable = 1;
        }
        if (any(uint4(clamp(neighbor_color * 255.0, 0.0, 255.0)) != uint4(clamp(color * 255.0, 0.0, 255.0)))) {
          if (diff_length > 0.f) {
            direction = normalize(diff);
          }

          if (diff_length < 0.05) {
            position_offset += direction * max(0.0f, 0.05 - diff_length);
            if (diff_length < 0.02) {
              segment_particle0s[segment_handle].v =
                  scaleAlongDirectional(segment_particle0s[segment_handle].v, -direction, 50 * diff_length);
              segment_particle1s[segment_handle].v =
                  scaleAlongDirectional(segment_particle1s[segment_handle].v, -direction, 50 * diff_length);
            }
          }
          if (diff_length0 < 0.02) {
            if (segments[segment_handle].quasi_stable == 1 && neighbor_segment.quasi_stable == 1 &&
                diff_length0 < 0.0015) {
              exert_fricton_0 = 1;
            }
          }
          if (diff_length1 < 0.02) {
            if (segments[segment_handle].quasi_stable == 1 && neighbor_segment.quasi_stable == 1 &&
                diff_length1 < 0.0015) {
              exert_fricton_1 = 1;
            }
          }
          sum++;
        }
      }
      begin++;
    }
    if (sum != 0) {
      position_offset /= sum;
      position_offset *= 10.f;
      if (segments[segment_handle].quasi_stable == 1) {
        float3 up = float3(0.0, 1.0, 0.0);

        if (exert_fricton_0 == 1) {
          float friction = 0.8 * 9.8;
          segment_particle0s[segment_handle].v = float3(0.0);
        }
        if (exert_fricton_1 == 1) {
          float friction = 0.8 * 9.8;
          segment_particle1s[segment_handle].v = float3(0.0);
        }

      } else {
        segment_particle0s[segment_handle].v += position_offset * 30.f;  // 30.0f;
        segment_particle1s[segment_handle].v += position_offset * 30.f;
      }
    }
  }
}
