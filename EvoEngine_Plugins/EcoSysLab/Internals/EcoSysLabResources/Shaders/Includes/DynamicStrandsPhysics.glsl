
#include "Math.glsl"
#include "DynamicStrands.glsl"

struct PerStrandData {
  int front_propagate_begin_connection_handle;
  int back_propagate_begin_connection_handle;
  int front_propagate_begin_segment_handle;
  int back_propagate_begin_segment_handle;
};

layout(std430, set = DYNAMIC_STRANDS_PHYSICS_SET, binding = 0) readonly buffer PER_STRAND_DATA_LIST_BLOCK {
  PerStrandData per_strand_data_list[];
};

void project_stretch_shear_constraint(in int segment_handle);
void project_bend_twist_constraint(in int connection_handle);

void project_stretch_shear_constraint(in int segment_handle) {
  Segment segment = segments[segment_handle];
  int particle0_handle = floatBitsToInt(segment.inertia_tensor_particle_0_handle.w);
  int particle1_handle = floatBitsToInt(segment.inv_inertia_tensor_particle_1_handle.w);
  Particle particle0 = particles[particle0_handle];
  Particle particle1 = particles[particle1_handle];

  vec3 x0_correction, x1_correction;
  vec4 q_correction;

  vec3 p0 = particle0.x.xyz;
  vec3 p1 = particle1.x.xyz;

  float inv_mass_p0 = particle0.acceleration_inv_mass.w;
  float inv_mass_p1 = particle1.acceleration_inv_mass.w;
  vec4 q = segment.q;
  float inv_mass_q = segment.inv_mass;
  vec3 stretching_and_shearing_k =
      vec3(segment.shearing_stiffness, segment.shearing_stiffness, segment.stretching_stiffness);
  float rest_length = segment.torque_rest_length.w;

  vec3 d3;
  d3[0] = -2.0 * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0 * (q.y * q.z - q.w * q.x);
  d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

  vec3 gamma = (p1 - p0) / rest_length - d3;
  segments[segment_handle].stretch_shear_strain = max(segment.stretch_shear_strain, length(gamma));

  gamma /= (inv_mass_p0 + inv_mass_p1) / rest_length + inv_mass_q * 4.0 * rest_length + 1e-6;
  if (abs(stretching_and_shearing_k.x - stretching_and_shearing_k.y) < 1e-6 &&
      abs(stretching_and_shearing_k.x - stretching_and_shearing_k.z) < 1e-6) {
    gamma.x *= stretching_and_shearing_k.x;
    gamma.y *= stretching_and_shearing_k.y;
    gamma.z *= stretching_and_shearing_k.z;
  } else {
    mat3 r = mat3_cast(q);
    gamma = transpose(r) * gamma;
    gamma.x *= stretching_and_shearing_k.x;
    gamma.y *= stretching_and_shearing_k.y;
    gamma.z *= stretching_and_shearing_k.z;
    gamma = r * gamma;
  }
  x0_correction = inv_mass_p0 * gamma;
  x1_correction = -inv_mass_p1 * gamma;

  vec4 q_e_3_bar = vec4(q.y, -q.x, q.w, -q.z);
  q_correction = quat_mul(vec4(gamma.x, gamma.y, gamma.z, 0.0), q_e_3_bar);
  q_correction *= 2.0 * inv_mass_q * rest_length;

  vec3 particle0_new_position = particles[particle0_handle].x.xyz + x0_correction;
  vec3 particle1_new_position = particles[particle1_handle].x.xyz + x1_correction;
  particles[particle0_handle].x.xyz = particle0_new_position;
  particles[particle1_handle].x.xyz = particle1_new_position;
  segments[segment_handle].q = normalize(q_correction + segment.q);

  if (segment.prev_handle != -1) {
    int copy_particle_handle = floatBitsToInt(segments[segment.prev_handle].inv_inertia_tensor_particle_1_handle.w);
    particles[copy_particle_handle].x.xyz = particle0_new_position;
  }
  if (segment.next_handle != -1) {
    int copy_particle_handle = floatBitsToInt(segments[segment.next_handle].inertia_tensor_particle_0_handle.w);
    particles[copy_particle_handle].x.xyz = particle1_new_position;
  }
}

void project_bend_twist_constraint(in int connection_handle) {
  Connection connection = connections[connection_handle];
  int segment0_handle = connection.segment0_handle;
  int segment1_handle = connection.segment1_handle;
  Segment segment0 = segments[segment0_handle];
  Segment segment1 = segments[segment1_handle];

  vec4 q0_correction, q1_correction;

  float rest_length = (segment0.torque_rest_length.w + segment1.torque_rest_length.w) * 0.5;

  vec3 bending_and_twisting_k =
      vec3(connection.bending_stiffness, connection.bending_stiffness, connection.twisting_stiffness);
  float inv_mass_q0 = segment0.inv_mass;
  float inv_mass_q1 = segment1.inv_mass;

  vec4 omega = quat_mul(conjugate(segment0.q), segment1.q);
  vec4 omega_plus = omega + connection.rest_darboux_vector;
  omega -= connection.rest_darboux_vector;
  if (squared_norm(omega) > squared_norm(omega_plus))
    omega = omega_plus;

  segments[segment0_handle].bend_twist_strain1 = segments[segment1_handle].bend_twist_strain0 =
      max(segment0.bend_twist_strain1, sqrt(squared_norm(omega)) * 10.0);

  float mul_factor = inv_mass_q0 + inv_mass_q1 + 1e-6;
  omega.x *= bending_and_twisting_k.x / mul_factor;
  omega.y *= bending_and_twisting_k.y / mul_factor;
  omega.z *= bending_and_twisting_k.z / mul_factor;
  omega.w = 0.0;

  q0_correction = quat_mul(segment1.q, omega) * inv_mass_q0;
  q1_correction = quat_mul(segment0.q, omega) * inv_mass_q1 * -1.0;

  segments[segment0_handle].q = normalize(q0_correction + segment0.q);
  segments[segment1_handle].q = normalize(q1_correction + segment1.q);
}


void solve_bend_twist_constraint(in vec4 q0, in float inv_mass_q0, in vec4 q1, in float inv_mass_q1,
                                 in vec3 bending_and_twisting_k, in vec4 rest_darboux_vector, out vec4 q0_correction,
                                 out vec4 q1_correction) {
  vec4 omega = quat_mul(conjugate(q0), q1);
  vec4 omega_plus = omega + rest_darboux_vector;
  omega -= rest_darboux_vector;
  if (squared_norm(omega) > squared_norm(omega_plus))
    omega = omega_plus;

  float mul_factor = inv_mass_q0 + inv_mass_q1 + 1e-6;
  omega.x *= bending_and_twisting_k.x / mul_factor;
  omega.y *= bending_and_twisting_k.y / mul_factor;
  omega.z *= bending_and_twisting_k.z / mul_factor;
  omega.w = 0.0;

  q0_correction = quat_mul(q1, omega) * inv_mass_q0;
  q1_correction = quat_mul(q0, omega) * inv_mass_q1 * -1.0;
}