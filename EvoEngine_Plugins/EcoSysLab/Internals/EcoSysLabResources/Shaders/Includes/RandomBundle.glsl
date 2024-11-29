
#define BUNDLE_MAX_CONNECTION 16

struct SegmentPair {
  int segment0_handle;
  int segment1_handle;
  int valid;
  float max_strain;
  float bending_alpha;
  float twisting_alpha;
  float bundle_weight;
  float bend_twist_weight;

  vec4 segment0_particle0_offset;
  vec4 segment0_particle1_offset;

  vec4 segment1_particle0_offset;
  vec4 segment1_particle1_offset;

  vec4 rest_darboux_vector;
};

layout(std430, set = 1, binding = 0) buffer SEGMENT_PAIR_BLOCK {
  SegmentPair segment_pairs[];
};

struct SegmentData {
  vec4 particle0_position_correction;
  vec4 particle1_position_correction;
  vec4 q_correction;
  int pair_handles[BUNDLE_MAX_CONNECTION];
};

layout(std430, set = 1, binding = 1) buffer SEGMENT_DATA_BLOCK {
  SegmentData segment_data_list[];
};


void BundleSegment(in uint segment_handle, in float inv_time_step, in float over_relaxation) {
  SegmentData segment_data = segment_data_list[segment_handle];
  vec3 movement0_sum = vec3(0.0f, 0.0f, 0.0f);
  vec3 movement1_sum = vec3(0.0f, 0.0f, 0.0f);

  float sum = 0.f;

  [[unroll]] for (uint i = 0; i < BUNDLE_MAX_CONNECTION; i++) {
    int pair_handle = segment_data.pair_handles[i];
    if (pair_handle < 0)
      break;
    SegmentPair segment_pair = segment_pairs[pair_handle];
    if (segment_pair.valid == 0)
      continue;
    bool is_segment0 = segment_handle == segment_pair.segment0_handle;
    Segment segment0 = segments[is_segment0 ? segment_pair.segment0_handle : segment_pair.segment1_handle];
    Segment segment1 = segments[is_segment0 ? segment_pair.segment1_handle : segment_pair.segment0_handle];

    int segment0_particle0_handle = floatBitsToInt(segment0.inertia_tensor_particle_0_handle.w);
    int segment0_particle1_handle = floatBitsToInt(segment0.inv_inertia_tensor_particle_1_handle.w);

    int segment1_particle0_handle = floatBitsToInt(segment1.inertia_tensor_particle_0_handle.w);
    int segment1_particle1_handle = floatBitsToInt(segment1.inv_inertia_tensor_particle_1_handle.w);

    Particle segment0_particle0 = particles[segment0_particle0_handle];
    Particle segment0_particle1 = particles[segment0_particle1_handle];
    Particle segment1_particle0 = particles[segment1_particle0_handle];
    Particle segment1_particle1 = particles[segment1_particle1_handle];

    vec3 segment1_center_position =
        (segment1_particle0.x_node_handle.xyz + segment1_particle1.x_node_handle.xyz) * 0.5f;

    vec3 target_segment0_particle0_position =
        segment1_center_position + rotate_vec3(segment1.q, is_segment0 ? segment_pair.segment0_particle0_offset.xyz
                                                                       : segment_pair.segment1_particle0_offset.xyz);
    vec3 target_segment0_particle1_position =
        segment1_center_position + rotate_vec3(segment1.q, is_segment0 ? segment_pair.segment0_particle1_offset.xyz
                                                                       : segment_pair.segment1_particle1_offset.xyz);

    float t2 = inv_time_step * inv_time_step;
    //float alpha_factor = t2 * segment_pair.bundle_alpha;
    float lambda = max(1e-9f, segment0.inv_mass + segment1.inv_mass);
    // Always enforce inf stiffness.
    float factor0 = segment0.inv_mass / lambda;

    vec3 segment0_particle0_position_correction =
        (target_segment0_particle0_position - segment0_particle0.x_node_handle.xyz) * factor0;
    vec3 segment0_particle1_position_correction =
        (target_segment0_particle1_position - segment0_particle1.x_node_handle.xyz) * factor0;

    sum += segment_pair.bundle_weight;
    movement0_sum += segment0_particle0_position_correction;
    movement1_sum += segment0_particle1_position_correction;
  }

  if (sum != 0) {
    segment_data_list[segment_handle].particle0_position_correction.xyz = movement0_sum / sum * over_relaxation;
    segment_data_list[segment_handle].particle1_position_correction.xyz = movement1_sum / sum * over_relaxation;
  } else {
    segment_data_list[segment_handle].particle0_position_correction.xyz = vec3(0.0f, 0.0f, 0.0f);
    segment_data_list[segment_handle].particle1_position_correction.xyz = vec3(0.0f, 0.0f, 0.0f);
  }
}

void BundleSegmentBendTwist(in uint segment_handle, in float inv_time_step, in float over_relaxation) {
  SegmentData segment_data = segment_data_list[segment_handle];
  vec4 q_correction_sum = vec4(0.0f, 0.0f, 0.0f, 0.0f);
  float sum = 0.f;
  [[unroll]] for (uint i = 0; i < BUNDLE_MAX_CONNECTION; i++) {
    int pair_handle = segment_data.pair_handles[i];
    if (pair_handle < 0)
      break;
    SegmentPair segment_pair = segment_pairs[pair_handle];
    if (segment_pair.valid == 0)
      continue;
    bool is_segment0 = segment_handle == segment_pair.segment0_handle;
    Segment segment0 = segments[is_segment0 ? segment_pair.segment0_handle : segment_pair.segment1_handle];
    Segment segment1 = segments[is_segment0 ? segment_pair.segment1_handle : segment_pair.segment0_handle];

    vec4 q0_correction, q1_correction;

    vec3 bend_twist_alpha = vec3(segment_pair.bending_alpha, segment_pair.bending_alpha, segment_pair.twisting_alpha);

    project_bend_twist_constraint(
        inv_time_step, segments[segment_pair.segment0_handle].q, segments[segment_pair.segment0_handle].inv_mass,
        segments[segment_pair.segment1_handle].q, segments[segment_pair.segment1_handle].inv_mass, bend_twist_alpha,
        segment_pair.rest_darboux_vector, q0_correction, q1_correction);

    vec4 q_correction = is_segment0 ? q0_correction : q1_correction;
    q_correction_sum += q_correction;
    sum += segment_pair.bend_twist_weight;
  }

  if (sum != 0) {
    segment_data_list[segment_handle].q_correction = q_correction_sum / sum * over_relaxation;
  } else {
    segment_data_list[segment_handle].q_correction = vec4(0.0f, 0.0f, 0.0f, 0.0f);
  }
}

void BundleSegmentStretchShear(in uint segment_handle, in float inv_time_step) {
  vec3 x0_correction, x1_correction;
  vec4 q_correction;

  Segment segment = segments[segment_handle];
  int particle0_handle = floatBitsToInt(segment.inertia_tensor_particle_0_handle.w);
  int particle1_handle = floatBitsToInt(segment.inv_inertia_tensor_particle_1_handle.w);
  Particle particle0 = particles[particle0_handle];
  Particle particle1 = particles[particle1_handle];

  vec3 p0 = particle0.x_node_handle.xyz;
  vec3 p1 = particle1.x_node_handle.xyz;

  float inv_mass_q = segment.inv_mass;
  float inv_mass_p0 = inv_mass_q;
  float inv_mass_p1 = inv_mass_q;
  if (segment.prev_handle != -1) {
    inv_mass_p0 = segments[segment.prev_handle].inv_mass;
  }
  if (segment.next_handle != -1) {
    inv_mass_p1 = segments[segment.next_handle].inv_mass;
  }
  vec4 q = segment.q;

  vec3 alpha = vec3(segment.shearing_alpha, segment.shearing_alpha, segment.stretching_alpha);

  //vec3 alpha = vec3(1.0, 1.0, 0.0);

  float rest_length = segment.torque_rest_length.w;

  project_stretch_shear_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q, alpha, rest_length,
                                   x0_correction, x1_correction, q_correction);

  segment_data_list[segment_handle].particle0_position_correction.xyz = x0_correction;
  segment_data_list[segment_handle].particle1_position_correction.xyz = x1_correction;
  segment_data_list[segment_handle].q_correction = q_correction;
}