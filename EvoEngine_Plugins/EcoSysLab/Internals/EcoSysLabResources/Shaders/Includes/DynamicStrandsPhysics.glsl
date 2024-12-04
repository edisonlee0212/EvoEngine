
#include "Math.glsl"
#include "DynamicStrands.glsl"

//Constraint Solvers Decl
void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle);
void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle, out vec3 x0_correction,
                                      out vec3 x1_correction,
                                      out vec4 q_correction);
void project_shear_stretch_constraint(in float inv_time_step, in vec3 p0, in vec3 p1, in vec4 q, in float inv_mass_p0,
                                      in float inv_mass_p1,
                                    in float inv_mass_q, in vec3 alpha, in float rest_length,
                                    out vec3 x0_correction, out vec3 x1_correction, out vec4 q_correction);

void project_bend_twist_constraint(in float inv_time_step, in int connection_handle);
void project_bend_twist_constraint(in float inv_time_step, in int connection_handle, out vec4 q0_correction,
                                 out vec4 q1_correction);
void project_bend_twist_constraint(in float inv_time_step, in vec4 q0, in float inv_mass_q0, in vec4 q1,
                                   in float inv_mass_q1,
                                   in vec3 alpha, in vec4 rest_darboux_vector, out vec4 q0_correction,
                                   out vec4 q1_correction);

void BundleSegment(in uint segment_handle, in float inv_time_step, in float over_relaxation);
void BundleSegmentBendTwist(in uint segment_handle, in float inv_time_step, in float over_relaxation);
void BundleSegmentShearStretch(in uint segment_handle, in float inv_time_step);

//Constraint Solvers Impl
void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle) {
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
  vec3 alpha =
      vec3(segment.shearing_alpha, segment.shearing_alpha, segment.stretching_alpha);
  float rest_length = segment.torque_rest_length.w;

  project_shear_stretch_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q, alpha,
                                   rest_length, x0_correction, x1_correction, q_correction);

  vec3 particle0_new_position = particles[particle0_handle].x_node_handle.xyz + x0_correction;
  vec3 particle1_new_position = particles[particle1_handle].x_node_handle.xyz + x1_correction;
  particles[particle0_handle].x_node_handle.xyz = particle0_new_position;
  particles[particle1_handle].x_node_handle.xyz = particle1_new_position;
  segments[segment_handle].q = normalize(q_correction + segment.q);

  if (segment.prev_handle != -1) {
    int copy_particle_handle = floatBitsToInt(segments[segment.prev_handle].inv_inertia_tensor_particle_1_handle.w);
    if (connections[particle0.connection_handle].bend_twist_strain_limit_connectivity_valid.w != 0.0)
      particles[copy_particle_handle].x_node_handle.xyz = particle0_new_position;
  }
  if (segment.next_handle != -1) {
    int copy_particle_handle = floatBitsToInt(segments[segment.next_handle].inertia_tensor_particle_0_handle.w);
    if (connections[particle1.connection_handle].bend_twist_strain_limit_connectivity_valid.w != 0.0)
      particles[copy_particle_handle].x_node_handle.xyz = particle1_new_position;
  }
}

void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle, out vec3 x0_correction,
                                      out vec3 x1_correction,
                                    out vec4 q_correction) {
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
  vec3 alpha =
      vec3(segment.shearing_alpha, segment.shearing_alpha, segment.stretching_alpha);
  float rest_length = segment.torque_rest_length.w;

  project_shear_stretch_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q,
                                   alpha,
                                 rest_length, x0_correction, x1_correction, q_correction);
}

void project_shear_stretch_constraint(in float inv_time_step, in vec3 p0, in vec3 p1, in vec4 q, in float inv_mass_p0,
                                      in float inv_mass_p1,
                                    in float inv_mass_q, in vec3 alpha, in float rest_length,
                                    out vec3 x0_correction, out vec3 x1_correction, out vec4 q_correction) {
  vec3 d3;
  d3[0] = -2.0f * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0f * (q.y * q.z - q.w * q.x);
  d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

  vec3 lambda = (p1 - p0) - d3 * rest_length;
  mat3 r = mat3_cast(q);
  lambda = transpose(r) * lambda;
  float factor = max(1e-9f, inv_mass_p0 + inv_mass_p1 + 4.0f * inv_mass_q * rest_length * rest_length);

  float t2 = inv_time_step * inv_time_step;
  vec3 alpha_factor = vec3(t2 * alpha.x, t2 * alpha.y, t2 * alpha.z);
  lambda.x /= (factor + alpha_factor.x);
  lambda.y /= (factor + alpha_factor.y);
  lambda.z /= (factor + alpha_factor.z);

  lambda = r * lambda;
  
  x0_correction = inv_mass_p0 * lambda;
  x1_correction = -inv_mass_p1 * lambda;

  vec4 q_e_3_bar = vec4(q.y, -q.x, q.w, -q.z);
  q_correction = quat_mul(vec4(lambda.x, lambda.y, lambda.z, 0.0f), q_e_3_bar);
  q_correction *= inv_mass_q * rest_length;
}

void project_bend_twist_constraint(in float inv_time_step, in int connection_handle) {
  Connection connection = connections[connection_handle];
  if (connection.bend_twist_strain_valid.w == 0.0f || connection.bend_twist_strain_limit_connectivity_valid.w == 0.0f)
    return;

  vec4 q0_correction, q1_correction;
  int segment0_handle = connection.segment0_handle;
  int segment1_handle = connection.segment1_handle;
  Segment segment0 = segments[segment0_handle];
  Segment segment1 = segments[segment1_handle];

  float rest_length = (segment0.torque_rest_length.w + segment1.torque_rest_length.w) * 0.5f;
  vec3 alpha =
      vec3(connection.bending_alpha, connection.bending_alpha, connection.twisting_alpha);
  project_bend_twist_constraint(inv_time_step, segment0.q, segment0.inv_mass, segment1.q, segment1.inv_mass,
                                alpha,
                                connection.rest_darboux_vector, q0_correction, q1_correction);

  segments[segment0_handle].q = normalize(q0_correction + segment0.q);
  segments[segment1_handle].q = normalize(q1_correction + segment1.q);
}

void project_bend_twist_constraint(in float inv_time_step, in int connection_handle, out vec4 q0_correction,
                                   out vec4 q1_correction) {
  Connection connection = connections[connection_handle];
  int segment0_handle = connection.segment0_handle;
  int segment1_handle = connection.segment1_handle;
  Segment segment0 = segments[segment0_handle];
  Segment segment1 = segments[segment1_handle];

  float rest_length = (segment0.torque_rest_length.w + segment1.torque_rest_length.w) * 0.5f;
  vec3 alpha =
      vec3(connection.bending_alpha, connection.bending_alpha, connection.twisting_alpha);
  project_bend_twist_constraint(inv_time_step, segment0.q, segment0.inv_mass, segment1.q, segment1.inv_mass,
                                alpha,
                                connection.rest_darboux_vector, q0_correction, q1_correction);
}

void project_bend_twist_constraint(in float inv_time_step, in vec4 q0, in float inv_mass_q0, in vec4 q1,
                                   in float inv_mass_q1,
                                 in vec3 alpha, in vec4 rest_darboux_vector, out vec4 q0_correction,
                                 out vec4 q1_correction) {
  vec4 lambda = quat_mul(conjugate(q0), q1);
  vec4 lambda_plus = lambda + rest_darboux_vector;
  lambda -= rest_darboux_vector;
  if (squared_norm(lambda) > squared_norm(lambda_plus))
    lambda = lambda_plus;
  float factor = max(1e-9f, inv_mass_q0 + inv_mass_q1);

  float t2 = inv_time_step * inv_time_step;
  vec3 alpha_factor = vec3(t2 * alpha.x, t2 * alpha.y, t2 * alpha.z);
  lambda.x /= (factor + alpha_factor.x);
  lambda.y /= (factor + alpha_factor.y);
  lambda.z /= (factor + alpha_factor.z);


  lambda.w = 0.0f;

  q0_correction = quat_mul(q1, lambda) * inv_mass_q0;
  q1_correction = quat_mul(q0, lambda) * inv_mass_q1 * -1.0f;
}

vec3 shear_stretch_strain(in vec3 p0, in vec3 p1, in vec4 q, in float rest_length) {
  vec3 d3;
  d3[0] = -2.0f * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0f * (q.y * q.z - q.w * q.x);
  d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

  vec3 lambda = (p1 - p0) / rest_length - d3;
  return lambda;
}

vec3 bend_twist_strain(in vec4 q0, in vec4 q1, in vec4 rest_darboux_vector) {
  vec4 lambda = quat_mul(conjugate(q0), q1);
  vec4 lambda_plus = lambda + rest_darboux_vector;
  lambda -= rest_darboux_vector;
  if (squared_norm(lambda) > squared_norm(lambda_plus))
    lambda = lambda_plus;
  return lambda.xyz;
}

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
    // float alpha_factor = t2 * segment_pair.bundle_alpha;
    float lambda = max(1e-9f, segment0.inv_mass + segment1.inv_mass);
    // Always enforce inf stiffness.
    float factor0 = segment0.inv_mass / lambda;

    vec3 segment0_particle0_position_correction =
        (target_segment0_particle0_position - segment0_particle0.x_node_handle.xyz) * factor0;
    vec3 segment0_particle1_position_correction =
        (target_segment0_particle1_position - segment0_particle1.x_node_handle.xyz) * factor0;

    sum += 1.0f;
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
    sum += 1.0f;
  }

  if (sum != 0) {
    segment_data_list[segment_handle].q_correction = q_correction_sum / sum * over_relaxation;
  } else {
    segment_data_list[segment_handle].q_correction = vec4(0.0f, 0.0f, 0.0f, 0.0f);
  }
}

void BundleSegmentShearStretch(in uint segment_handle, in float inv_time_step) {
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

  // vec3 alpha = vec3(1.0, 1.0, 0.0);

  float rest_length = segment.torque_rest_length.w;

  project_shear_stretch_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q, alpha, rest_length,
                                   x0_correction, x1_correction, q_correction);

  segment_data_list[segment_handle].particle0_position_correction.xyz = x0_correction;
  segment_data_list[segment_handle].particle1_position_correction.xyz = x1_correction;
  segment_data_list[segment_handle].q_correction = q_correction;
}