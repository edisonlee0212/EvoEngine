
#include "Math.glsl"
#include "DynamicStrands.glsl"

#define USE_XPBD

//Constraint Solvers Decl
void project_stretch_shear_constraint(in float inv_time_step, in int segment_handle);
void project_stretch_shear_constraint(in float inv_time_step, in int segment_handle, out vec3 x0_correction,
                                      out vec3 x1_correction,
                                      out vec4 q_correction);
void project_stretch_shear_constraint(in float inv_time_step, in vec3 p0, in vec3 p1, in vec4 q, in float inv_mass_p0,
                                      in float inv_mass_p1,
                                    in float inv_mass_q, in vec3 stiffness, in float rest_length,
                                    out vec3 x0_correction, out vec3 x1_correction, out vec4 q_correction);

void project_bend_twist_constraint(in float inv_time_step, in int connection_handle);
void project_bend_twist_constraint(in float inv_time_step, in int connection_handle, out vec4 q0_correction,
                                 out vec4 q1_correction);
void project_bend_twist_constraint(in float inv_time_step, in vec4 q0, in float inv_mass_q0, in vec4 q1,
                                   in float inv_mass_q1,
                                   in vec3 stiffness, in vec4 rest_darboux_vector, out vec4 q0_correction,
                                   out vec4 q1_correction);

//Constraint Solvers Impl
void project_stretch_shear_constraint(in float inv_time_step, in int segment_handle) {
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
  vec3 stiffness =
      vec3(segment.shearing_stiffness, segment.shearing_stiffness, segment.stretching_stiffness);
  float rest_length = segment.torque_rest_length.w;

  project_stretch_shear_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q,
                                   stiffness,
                                   rest_length, x0_correction, x1_correction, q_correction);

  vec3 particle0_new_position = particles[particle0_handle].x_node_handle.xyz + x0_correction;
  vec3 particle1_new_position = particles[particle1_handle].x_node_handle.xyz + x1_correction;
  particles[particle0_handle].x_node_handle.xyz = particle0_new_position;
  particles[particle1_handle].x_node_handle.xyz = particle1_new_position;
  segments[segment_handle].q = normalize(q_correction + segment.q);

  if (segment.prev_handle != -1) {
    int copy_particle_handle = floatBitsToInt(segments[segment.prev_handle].inv_inertia_tensor_particle_1_handle.w);
    if (connections[particle0.connection_handle].bend_twist_strain_valid.w != 0.0)
      particles[copy_particle_handle].x_node_handle.xyz = particle0_new_position;
  }
  if (segment.next_handle != -1) {
    int copy_particle_handle = floatBitsToInt(segments[segment.next_handle].inertia_tensor_particle_0_handle.w);
    if (connections[particle1.connection_handle].bend_twist_strain_valid.w != 0.0)
      particles[copy_particle_handle].x_node_handle.xyz = particle1_new_position;
  }
}

void project_stretch_shear_constraint(in float inv_time_step, in int segment_handle, out vec3 x0_correction,
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
  vec3 stiffness =
      vec3(segment.shearing_stiffness, segment.shearing_stiffness, segment.stretching_stiffness);
  float rest_length = segment.torque_rest_length.w;

  project_stretch_shear_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q,
                                   stiffness,
                                 rest_length, x0_correction, x1_correction, q_correction);
}



void project_stretch_shear_constraint(in float inv_time_step, in vec3 p0, in vec3 p1, in vec4 q, in float inv_mass_p0,
                                      in float inv_mass_p1,
                                    in float inv_mass_q, in vec3 stiffness, in float rest_length,
                                    out vec3 x0_correction, out vec3 x1_correction, out vec4 q_correction) {
  vec3 d3;
  d3[0] = -2.0 * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0 * (q.y * q.z - q.w * q.x);
  d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

  vec3 lambda = (p1 - p0) - d3 * rest_length;
  mat3 r = mat3_cast(q);
  lambda = transpose(r) * lambda;
  float factor = inv_mass_p0 + inv_mass_p1 + 4.0 * inv_mass_q * rest_length * rest_length + 1e-6;

#ifdef USE_XPBD
  float t2 = inv_time_step * inv_time_step;
  vec3 stiffness_factor = vec3(t2 / stiffness.x, t2 / stiffness.y, t2 / stiffness.z);
  lambda.x /= (factor + stiffness_factor.x);
  lambda.y /= (factor + stiffness_factor.y);
  lambda.z /= (factor + stiffness_factor.z);
#else
  lambda /= factor;

  lambda.x *= stiffness.x;
  lambda.y *= stiffness.y;
  lambda.z *= stiffness.z;
#endif
  lambda = r * lambda;
  
  x0_correction = inv_mass_p0 * lambda;
  x1_correction = -inv_mass_p1 * lambda;

  vec4 q_e_3_bar = vec4(q.y, -q.x, q.w, -q.z);
  q_correction = quat_mul(vec4(lambda.x, lambda.y, lambda.z, 0.0), q_e_3_bar);
  q_correction *= inv_mass_q * rest_length;
}

void project_bend_twist_constraint(in float inv_time_step, in int connection_handle) {
  Connection connection = connections[connection_handle];
  if (connections[connection_handle].bend_twist_strain_valid.w == 0.0)
    return;

  vec4 q0_correction, q1_correction;
  int segment0_handle = connection.segment0_handle;
  int segment1_handle = connection.segment1_handle;
  Segment segment0 = segments[segment0_handle];
  Segment segment1 = segments[segment1_handle];

  float rest_length = (segment0.torque_rest_length.w + segment1.torque_rest_length.w) * 0.5;
  vec3 stiffness =
      vec3(connection.bending_stiffness, connection.bending_stiffness, connection.twisting_stiffness);
  project_bend_twist_constraint(inv_time_step, segment0.q, segment0.inv_mass, segment1.q, segment1.inv_mass,
                                stiffness,
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

  float rest_length = (segment0.torque_rest_length.w + segment1.torque_rest_length.w) * 0.5;
  vec3 stiffness =
      vec3(connection.bending_stiffness, connection.bending_stiffness, connection.twisting_stiffness);
  project_bend_twist_constraint(inv_time_step, segment0.q, segment0.inv_mass, segment1.q, segment1.inv_mass,
                                stiffness,
                                connection.rest_darboux_vector, q0_correction, q1_correction);
}


void project_bend_twist_constraint(in float inv_time_step, in vec4 q0, in float inv_mass_q0, in vec4 q1,
                                   in float inv_mass_q1,
                                 in vec3 stiffness, in vec4 rest_darboux_vector, out vec4 q0_correction,
                                 out vec4 q1_correction) {
  vec4 lambda = quat_mul(conjugate(q0), q1);
  vec4 lambda_plus = lambda + rest_darboux_vector;
  lambda -= rest_darboux_vector;
  if (squared_norm(lambda) > squared_norm(lambda_plus))
    lambda = lambda_plus;
  float factor = inv_mass_q0 + inv_mass_q1 + 1e-6;

#ifdef USE_XPBD
  float t2 = inv_time_step * inv_time_step;
  vec3 stiffness_factor = vec3(t2 / stiffness.x, t2 / stiffness.y, t2 / stiffness.z);
  lambda.x /= (factor + stiffness_factor.x);
  lambda.y /= (factor + stiffness_factor.y);
  lambda.z /= (factor + stiffness_factor.z);
#else
  lambda /= factor;
  lambda.x *= stiffness.x;
  lambda.y *= stiffness.y;
  lambda.z *= stiffness.z;
#endif

  lambda.w = 0.0;

  q0_correction = quat_mul(q1, lambda) * inv_mass_q0;
  q1_correction = quat_mul(q0, lambda) * inv_mass_q1 * -1.0;
}





vec3 stretch_shear_strain(in vec3 p0, in vec3 p1, in vec4 q, in float rest_length) {
  vec3 d3;
  d3[0] = -2.0 * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0 * (q.y * q.z - q.w * q.x);
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
