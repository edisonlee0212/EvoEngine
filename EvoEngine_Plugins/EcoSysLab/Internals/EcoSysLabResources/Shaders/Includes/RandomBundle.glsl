
#define BUNDLE_MAX_CONNECTION 8

struct SegmentPair {
  int segment0_handle;
  int segment1_handle;
  int valid;
  float max_strain;
  vec4 alpha;

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

void BundleSegment(in uint segment_index, in float inv_time_step) {
  SegmentData segment_data = segment_data_list[segment_index];
  vec3 movement0 = vec3(0.0f, 0.0f, 0.0f);
  vec3 movement1 = vec3(0.0f, 0.0f, 0.0f);

  vec4 q_correction = vec4(0.0f, 0.0f, 0.0f, 0.0f);

  int sum = 0;

  [[unroll]]
  for (uint i = 0; i < BUNDLE_MAX_CONNECTION; i++) {
    int pair_handle = segment_data.pair_handles[i];
    if (pair_handle < 0) break;
    SegmentPair segment_pair = segment_pairs[pair_handle];
    if (segment_pair.valid == 0)
      continue;
    bool is_segment0 = segment_index == segment_pair.segment0_handle;
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

    vec3 segment1_center_position = (segment1_particle0.x_node_handle.xyz + segment1_particle1.x_node_handle.xyz) * 0.5f;

    vec3 target_segment0_particle0_position =
        segment1_center_position + rotate_vec3(segment1.q, is_segment0 ? segment_pair.segment0_particle0_offset.xyz
                                                                       : segment_pair.segment1_particle0_offset.xyz);
    vec3 target_segment0_particle1_position =
        segment1_center_position + rotate_vec3(segment1.q, is_segment0 ? segment_pair.segment0_particle1_offset.xyz
                                                                       : segment_pair.segment1_particle1_offset.xyz);

    float t2 = inv_time_step * inv_time_step;
    float alpha_factor = t2 * segment_pair.alpha.w;
    //float lambda = max(1e-9f, segment0.inv_mass + segment1.inv_mass + alpha_factor);
    //Always enforce inf stiffness.
    float lambda = max(1e-9f, segment0.inv_mass + segment1.inv_mass);
    float factor0 = segment0.inv_mass / lambda;

    vec3 segment0_particle0_position_correction =
        (target_segment0_particle0_position - segment0_particle0.x_node_handle.xyz) * factor0;
    vec3 segment0_particle1_position_correction =
        (target_segment0_particle1_position - segment0_particle1.x_node_handle.xyz) * factor0;

    target_segment0_particle0_position = segment0_particle0.x_node_handle.xyz + segment0_particle0_position_correction;
    target_segment0_particle1_position = segment0_particle1.x_node_handle.xyz + segment0_particle1_position_correction;
    
    vec4 q0_correction, q1_correction;

    vec3 bend_twist_alpha = vec3(segment_pair.alpha.x, segment_pair.alpha.x, segment_pair.alpha.y);

    project_bend_twist_constraint(inv_time_step, 
        segments[segment_pair.segment0_handle].q, segments[segment_pair.segment0_handle].inv_mass,
        segments[segment_pair.segment1_handle].q, segments[segment_pair.segment1_handle].inv_mass, bend_twist_alpha,
        segment_pair.rest_darboux_vector, q0_correction,
                                    q1_correction);

    if (is_segment0) {
      segment0.q = normalize(q0_correction + segment0.q);
    } else {
      segment0.q = normalize(q1_correction + segment0.q);
    }
    
    vec3 x0_correction, x1_correction;
    vec4 ss_q_correction;
    vec3 stretch_shear_alpha = vec3(segment_pair.alpha.z, segment_pair.alpha.z, segment_pair.alpha.w);
    project_stretch_shear_constraint(inv_time_step, target_segment0_particle0_position, 
        target_segment0_particle1_position, segment0.q, segment0.inv_mass,
        segment0.inv_mass, segment0.inv_mass, stretch_shear_alpha,
        segment0.torque_rest_length.w,
        x0_correction, x1_correction, ss_q_correction);

    target_segment0_particle0_position += x0_correction;
    target_segment0_particle1_position += x1_correction;

    segment0_particle0_position_correction =
        target_segment0_particle0_position - segment0_particle0.x_node_handle.xyz;
    segment0_particle1_position_correction =
        target_segment0_particle1_position - segment0_particle1.x_node_handle.xyz;

    sum++;
    movement0 += segment0_particle0_position_correction;
    movement1 += segment0_particle1_position_correction;
    q_correction += ss_q_correction + (is_segment0 ? q0_correction : q1_correction);

  }

  if (sum != 0) {
    segment_data_list[segment_index].particle0_position_correction.xyz = movement0 / sum;
    segment_data_list[segment_index].particle1_position_correction.xyz = movement1 / sum;
    segment_data_list[segment_index].q_correction = q_correction / sum;
  } else {
    segment_data_list[segment_index].particle0_position_correction.xyz = vec3(0.0f, 0.0f, 0.0f);
    segment_data_list[segment_index].particle1_position_correction.xyz = vec3(0.0f, 0.0f, 0.0f);
    segment_data_list[segment_index].q_correction = vec4(0.0f, 0.0f, 0.0f, 0.0f);
  }
}