
#include "Math.glsl"
struct RodProperties {
  float density;
  float i1;
  float j;
  float e;

  float g;
  float padding0;
  float padding1;
  float padding2;
};

layout(set = DYNAMIC_STRANDS_PHYSICS_SET, binding = 0) readonly uniform ROD_PROPERTIES_BLOCK {
  RodProperties rod_properties;
};

struct PerStrandData {
  int begin_rod_constraint_handle;
  int end_rod_constraint_handle;
  int segment_size;
  int padding1;
};

layout(std430, set = DYNAMIC_STRANDS_PHYSICS_SET, binding = 1) readonly buffer PER_STRAND_DATA_LIST_BLOCK {
  PerStrandData per_strand_data_list[];
};

struct RodConstraint {
  vec4 stiffness_coefficient_k_segment0_index;
  vec4 rest_darboux_vector_segment1_index;
  vec4 stretch_compliance_average_segment_length;
  vec4 bending_and_torsion_compliance_next_constraint_handle;
  /**
   * \brief
   * constraintInfo contains
   * 0: connector in segment 0 (local)
   * 1: connector in segment 1 (local)
   * 2: connector in segment 0 (global)
   * 3: connector in segment 1 (global)
   */
  mat4 constraint_info;

  float lambda_sum0;
  float lambda_sum1;
  float lambda_sum2;
  float lambda_sum3;

  float lambda_sum4;
  float lambda_sum5;
  float padding0;
  float padding1;
};

layout(std430, set = DYNAMIC_STRANDS_PHYSICS_SET, binding = 2) buffer ROD_CONSTRAINT_BLOCK {
  RodConstraint rod_constraints[];
};

struct PerSegmentData {
  vec4 lambda_ss;
};

layout(std430, set = DYNAMIC_STRANDS_PHYSICS_SET, binding = 3) buffer PER_SEGMENT_DATA_LIST_BLOCK {
  PerSegmentData per_segment_data_list[];
};

struct vec6 {
  float v[6];
};

struct mat6 {
  vec6 v[6];
};

void compute_bending_and_torsion_jacobians(in vec4 q0, in vec4 q1, float segment_length, out mat4 j_omega_0,
                                           out mat4 j_omega_1);
void compute_matrix_g(in vec4 q, out mat4 g);
void compute_matrix_k(in vec3 connector, in float inv_mass, in vec3 x, in mat3 inverse_inertia, out mat3 k);
mat3 cross_product_matrix(in vec3 v);
void solve_ldlt(in mat6 a, out vec6 x, in vec6 b);

mat3 compute_inverse_inertia_tensor_w(in StrandSegment strand_segment);

void init_stretch_bending_twisting_constraint(in vec3 stiffness_coefficient_k, in float inverse_time_step_size,
                                              in float average_segment_length, out vec3 stretch_compliance,
                                              out vec3 bending_and_torsion_compliance, out vec6 lambda_sum);

void update_stretch_bending_twisting_constraint(in vec3 x0, in vec4 q0, in vec3 x1, in vec4 q1,
                                                inout mat4 constraint_info);

void solve_stretch_bending_twisting_constraints(in float inv_mass0, in vec3 x0, in mat3 inverse_inertia0, in vec4 q0,
                                                in float inv_mass1, in vec3 x1, in mat3 inverse_inertia1, in vec4 q1,
                                                in vec3 rest_darboux_vector, in float average_segment_length,
                                                in vec3 stretch_compliance, in vec3 bending_and_torsion_compliance,
                                                in mat4 constraint_info, out vec3 x0_correction, out vec4 q0_correction,
                                                out vec3 x1_correction, out vec4 q1_correction, inout vec6 lambda_sum);

//-------------------
//| Implementations |
//-------------------

void compute_bending_and_torsion_jacobians(in vec4 q0, in vec4 q1, float segment_length, out mat4 j_omega_0,
                                           out mat4 j_omega_1) {
  j_omega_0 = mat4(-q1[3], -q1[2], q1[1], q1[0], q1[2], -q1[3], -q1[0], q1[1], -q1[1], q1[0], -q1[3], q1[2], 0.0, 0.0,
                   0.0, 0.0);
  j_omega_1 = mat4(q0[3], q0[2], -q0[1], -q0[0], -q0[2], q0[3], q0[0], -q0[1], q0[1], -q0[0], q0[3], -q0[2], 0.0, 0.0,
                   0.0, 0.0);
  j_omega_0 = j_omega_0 * 2.0 / segment_length;
  j_omega_1 = j_omega_1 * 2.0 / segment_length;
}

void compute_matrix_g(in vec4 q, out mat4 g) {
  g = mat4(0.5 * q[3], 0.5 * q[2], -0.5 * q[1], 0.0, -0.5 * q[2], 0.5 * q[3], 0.5 * q[0], 0.0, 0.5 * q[1], -0.5 * q[0],
           0.5 * q[3], 0.0, -0.5 * q[0], -0.5 * q[1], -0.5 * q[2], 0.0);
}

void compute_matrix_k(in vec3 connector, in float inv_mass, in vec3 x, in mat3 inverse_inertia, out mat3 k) {
  if (inv_mass != 0.0) {
    vec3 v = connector - x;
    float a = v[0];
    float b = v[1];
    float c = v[2];

    float j11 = inverse_inertia[0][0];
    float j12 = inverse_inertia[0][1];
    float j13 = inverse_inertia[0][2];
    float j22 = inverse_inertia[1][1];
    float j23 = inverse_inertia[1][2];
    float j33 = inverse_inertia[2][2];

    k[0][0] = c * c * j22 - b * c * (j23 + j23) + b * b * j33 + inv_mass;
    k[0][1] = -(c * c * j12) + a * c * j23 + b * c * j13 - a * b * j33;
    k[0][2] = b * c * j12 - a * c * j22 - b * b * j13 + a * b * j23;
    k[1][0] = k[0][1];
    k[1][1] = c * c * j11 - a * c * (j13 + j13) + a * a * j33 + inv_mass;
    k[1][2] = -(b * c * j11) + a * c * j12 + a * b * j13 - a * a * j23;
    k[2][0] = k[0][2];
    k[2][1] = k[1][2];
    k[2][2] = b * b * j11 - a * b * (j12 + j12) + a * a * j22 + inv_mass;
  } else {
    k = mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
}

mat3 cross_product_matrix(in vec3 v) {
  return mat3(0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0);
}

void solve_ldlt(in mat6 A, out vec6 x, in vec6 b) {
  mat6 L;
  L.v[0].v[0] = 1.0;
  L.v[0].v[1] = 1.0;
  L.v[0].v[2] = 1.0;
  L.v[0].v[3] = 1.0;
  L.v[0].v[4] = 1.0;
  L.v[0].v[5] = 1.0;

  L.v[1].v[0] = 1.0;
  L.v[1].v[1] = 1.0;
  L.v[1].v[2] = 1.0;
  L.v[1].v[3] = 1.0;
  L.v[1].v[4] = 1.0;
  L.v[1].v[5] = 1.0;

  L.v[2].v[0] = 1.0;
  L.v[2].v[1] = 1.0;
  L.v[2].v[2] = 1.0;
  L.v[2].v[3] = 1.0;
  L.v[2].v[4] = 1.0;
  L.v[2].v[5] = 1.0;

  L.v[3].v[0] = 1.0;
  L.v[3].v[1] = 1.0;
  L.v[3].v[2] = 1.0;
  L.v[3].v[3] = 1.0;
  L.v[3].v[4] = 1.0;
  L.v[3].v[5] = 1.0;

  L.v[4].v[0] = 1.0;
  L.v[4].v[1] = 1.0;
  L.v[4].v[2] = 1.0;
  L.v[4].v[3] = 1.0;
  L.v[4].v[4] = 1.0;
  L.v[4].v[5] = 1.0;

  L.v[5].v[0] = 1.0;
  L.v[5].v[1] = 1.0;
  L.v[5].v[2] = 1.0;
  L.v[5].v[3] = 1.0;
  L.v[5].v[4] = 1.0;
  L.v[5].v[5] = 1.0;

  vec6 D;
  D.v[0] = 0.0;
  D.v[1] = 0.0;
  D.v[2] = 0.0;
  D.v[3] = 0.0;
  D.v[4] = 0.0;
  D.v[5] = 0.0;

  // Decomposition
  for (int j = 0; j < 6; j++) {
    // Compute diagonal elements of D
    D.v[j] = A.v[j].v[j];
    for (int k = 0; k < j; k++) {
      D.v[j] -= L.v[j].v[k] * L.v[j].v[k] * D.v[k];
    }
    // Compute elements of L (below the diagonal)
    for (int i = j + 1; i < 6; i++) {
      L.v[i].v[j] = A.v[i].v[j];
      for (int k = 0; k < j; k++) {
        L.v[i].v[j] -= L.v[i].v[k] * L.v[j].v[k] * D.v[k];
      }
      L.v[i].v[j] /= D.v[j];
    }
  }

  // Forward substitution
  vec6 y;
  for (int i = 0; i < 6; i++) {
    y.v[i] = b.v[i];
    for (int j = 0; j < i; j++) {
      y.v[i] -= L.v[i].v[j] * y.v[j];
    }
  }

  // Diagonal solve
  vec6 z;
  for (int i = 0; i < 6; i++) {
    z.v[i] = y.v[i] / D.v[i];
  }

  // Backward substitution
  for (int i = 5; i >= 0; i--) {
    x.v[i] = z.v[i];
    for (int j = i + 1; j < 6; j++) {
      x.v[i] -= L.v[j].v[i] * x.v[j];
    }
  }
}

mat3 compute_inverse_inertia_tensor_w(in StrandSegment strand_segment) {
  mat3 rot = mat3_cast(strand_segment.q);
  mat3 diag =
      mat3(
          strand_segment.inv_inertia_tensor_inv_mass.x, 0.0, 0.0, 
          0.0, strand_segment.inv_inertia_tensor_inv_mass.y, 0.0, 
          0.0, 0.0, strand_segment.inv_inertia_tensor_inv_mass.z);
  return rot * diag * transpose(rot);
}

void init_stretch_bending_twisting_constraint(in vec3 stiffness_coefficient_k, in float inverse_time_step_size,
                                              in float average_segment_length, out vec3 stretch_compliance,
                                              out vec3 bending_and_torsion_compliance, out vec6 lambda_sum) {
  float inverse_time_step_quadratic = inverse_time_step_size * inverse_time_step_size;
  float stretch_regularization_parameter = 0.1;

  stretch_compliance = vec3(stretch_regularization_parameter * inverse_time_step_quadratic,
                            stretch_regularization_parameter * inverse_time_step_quadratic,
                            stretch_regularization_parameter * inverse_time_step_quadratic);

  bending_and_torsion_compliance = vec3(inverse_time_step_quadratic / stiffness_coefficient_k[0],
                                        inverse_time_step_quadratic / stiffness_coefficient_k[1],
                                        inverse_time_step_quadratic / stiffness_coefficient_k[2]);

  lambda_sum.v[0] = 0.0;
  lambda_sum.v[1] = 0.0;
  lambda_sum.v[2] = 0.0;
  lambda_sum.v[3] = 0.0;
  lambda_sum.v[4] = 0.0;
  lambda_sum.v[5] = 0.0;
}

void update_stretch_bending_twisting_constraint(in vec3 x0, in vec4 q0, in vec3 x1, in vec4 q1,
                                                inout mat4 constraint_info) {
  // constraintInfo contains
  // 0:	connector in segment 0 (local)
  // 1:	connector in segment 1 (local)
  // 2:	connector in segment 0 (global)
  // 3:	connector in segment 1 (global)

  // compute world space positions of connectors
  mat3 rot0 = mat3_cast(q0);
  mat3 rot1 = mat3_cast(q1);

  constraint_info[2].xyz = rot0 * constraint_info[0].xyz + x0;
  constraint_info[3].xyz = rot1 * constraint_info[1].xyz + x1;
}

void solve_stretch_bending_twisting_constraints(in float inv_mass0, in vec3 x0, in mat3 inverse_inertia0, in vec4 q0,
                                                in float inv_mass1, in vec3 x1, in mat3 inverse_inertia1, in vec4 q1,
                                                in vec3 rest_darboux_vector, in float average_segment_length,
                                                in vec3 stretch_compliance, in vec3 bending_and_torsion_compliance,
                                                in mat4 constraint_info, out vec3 x0_correction, out vec4 q0_correction,
                                                out vec3 x1_correction, out vec4 q1_correction, inout vec6 lambda_sum) {
  vec3 omega = compute_darboux_vector(q0, q1, average_segment_length);
  mat4 j_omega0, j_omega1;
  compute_bending_and_torsion_jacobians(q0, q1, average_segment_length, j_omega0, j_omega1);
  mat4 g0, g1;
  compute_matrix_g(q0, g0);
  compute_matrix_g(q1, g1);

  mat3 j_omega_g0 = mat3(j_omega0 * g0);
  mat3 j_omega_g1 = mat3(j_omega1 * g1);

  vec3 connector0 = constraint_info[2].xyz;
  vec3 connector1 = constraint_info[3].xyz;

  vec3 stretch_violation = connector0 - connector1;
  vec3 bending_and_torsion_violation = omega - rest_darboux_vector;

  stretch_violation = stretch_violation + stretch_compliance * vec3(lambda_sum.v[0], lambda_sum.v[1], lambda_sum.v[2]);
  bending_and_torsion_violation =
      bending_and_torsion_violation +
      bending_and_torsion_compliance * vec3(lambda_sum.v[3], lambda_sum.v[4], lambda_sum.v[5]);

  vec6 rhs;
  rhs.v[0] = -stretch_violation.x;
  rhs.v[1] = -stretch_violation.y;
  rhs.v[2] = -stretch_violation.z;
  rhs.v[3] = -bending_and_torsion_violation.x;
  rhs.v[4] = -bending_and_torsion_violation.y;
  rhs.v[5] = -bending_and_torsion_violation.z;

  mat3 k0, k1;
  compute_matrix_k(connector0, inv_mass0, x0, inverse_inertia0, k0);
  compute_matrix_k(connector1, inv_mass1, x1, inverse_inertia1, k1);
  mat3 k = k0 + k1;
  mat6 jmjt;
  jmjt.v[0].v[0] = k[0][0];
  jmjt.v[0].v[1] = k[0][1];
  jmjt.v[0].v[2] = k[0][2];
  jmjt.v[1].v[0] = k[1][0];
  jmjt.v[1].v[1] = k[1][1];
  jmjt.v[1].v[2] = k[1][2];
  jmjt.v[2].v[0] = k[2][0];
  jmjt.v[2].v[1] = k[2][1];
  jmjt.v[2].v[2] = k[2][2];

  vec3 r0 = connector0 - x0;
  vec3 r1 = connector1 - x1;

  mat3 r0_cross_t = cross_product_matrix(-r0);
  mat3 r1_cross_t = cross_product_matrix(-r1);

  mat3 off_diag = mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
  if (inv_mass0 != 0.0) {
    off_diag = j_omega_g0 * inverse_inertia0 * r0_cross_t * -1.0;
  }
  if (inv_mass1 != 0.0) {
    off_diag = off_diag + j_omega_g1 * inverse_inertia1 * r1_cross_t * -1.0;
  }
  jmjt.v[3].v[0] = off_diag[0][0];
  jmjt.v[3].v[1] = off_diag[0][1];
  jmjt.v[3].v[2] = off_diag[0][2];
  jmjt.v[4].v[0] = off_diag[1][0];
  jmjt.v[4].v[1] = off_diag[1][1];
  jmjt.v[4].v[2] = off_diag[1][2];
  jmjt.v[5].v[0] = off_diag[2][0];
  jmjt.v[5].v[1] = off_diag[2][1];
  jmjt.v[5].v[2] = off_diag[2][2];

  mat3 off_diag_t = transpose(off_diag);
  jmjt.v[0].v[3] = off_diag_t[0][0];
  jmjt.v[0].v[4] = off_diag_t[0][1];
  jmjt.v[0].v[5] = off_diag_t[0][2];
  jmjt.v[1].v[3] = off_diag_t[1][0];
  jmjt.v[1].v[4] = off_diag_t[1][1];
  jmjt.v[1].v[5] = off_diag_t[1][2];
  jmjt.v[2].v[3] = off_diag_t[2][0];
  jmjt.v[2].v[4] = off_diag_t[2][1];
  jmjt.v[2].v[5] = off_diag_t[2][2];

  mat3 m_inv_jt0 = inverse_inertia0 * transpose(j_omega_g0);
  mat3 m_inv_jt1 = inverse_inertia1 * transpose(j_omega_g1);

  mat3 jmjt_omega;
  if (inv_mass0 != 0.0) {
    jmjt_omega = j_omega_g0 * m_inv_jt0;
  }
  if (inv_mass1 != 0.0) {
    jmjt_omega = jmjt_omega + j_omega_g1 * m_inv_jt1;
  }

  jmjt.v[3].v[3] = jmjt_omega[0][0];
  jmjt.v[3].v[4] = jmjt_omega[0][1];
  jmjt.v[3].v[5] = jmjt_omega[0][2];
  jmjt.v[4].v[3] = jmjt_omega[1][0];
  jmjt.v[4].v[4] = jmjt_omega[1][1];
  jmjt.v[4].v[5] = jmjt_omega[1][2];
  jmjt.v[5].v[3] = jmjt_omega[2][0];
  jmjt.v[5].v[4] = jmjt_omega[2][1];
  jmjt.v[5].v[5] = jmjt_omega[2][2];

  vec6 delta_lambda;
  solve_ldlt(jmjt, delta_lambda, rhs);

  lambda_sum.v[0] += delta_lambda.v[0];
  lambda_sum.v[1] += delta_lambda.v[1];
  lambda_sum.v[2] += delta_lambda.v[2];
  lambda_sum.v[3] += delta_lambda.v[3];
  lambda_sum.v[4] += delta_lambda.v[4];
  lambda_sum.v[5] += delta_lambda.v[5];

  vec3 delta_lambda_stretch = vec3(delta_lambda.v[0], delta_lambda.v[1], delta_lambda.v[2]);
  vec3 delta_lambda_bending_and_torsion = vec3(delta_lambda.v[3], delta_lambda.v[4], delta_lambda.v[5]);

  x0_correction = vec3(0, 0, 0);
  q0_correction = vec4(0, 0, 0, 0);
  x1_correction = vec3(0, 0, 0);
  q1_correction = vec4(0, 0, 0, 0);

  if (inv_mass0 != 0.0) {
    x0_correction = inv_mass0 * delta_lambda_stretch;
    vec3 v =
        inverse_inertia0 * r0_cross_t * (-1.0 * delta_lambda_stretch) + m_inv_jt0 * delta_lambda_bending_and_torsion;
    q0_correction[0] = g0[0][0] * v[0] + g0[0][1] * v[1] + g0[0][2] * v[2];
    q0_correction[1] = g0[1][0] * v[0] + g0[1][1] * v[1] + g0[1][2] * v[2];
    q0_correction[2] = g0[2][0] * v[0] + g0[2][1] * v[1] + g0[2][2] * v[2];
    q0_correction[3] = g0[3][0] * v[0] + g0[3][1] * v[1] + g0[3][2] * v[2];
  }

  if (inv_mass1 != 0.0) {
    x1_correction = -inv_mass1 * delta_lambda_stretch;
    vec3 v = inverse_inertia1 * r1_cross_t * delta_lambda_stretch + m_inv_jt1 * delta_lambda_bending_and_torsion;
    q1_correction[0] = g1[0][0] * v[0] + g1[0][1] * v[1] + g1[0][2] * v[2];
    q1_correction[1] = g1[1][0] * v[0] + g1[1][1] * v[1] + g1[1][2] * v[2];
    q1_correction[2] = g1[2][0] * v[0] + g1[2][1] * v[1] + g1[2][2] * v[2];
    q1_correction[3] = g1[3][0] * v[0] + g1[3][1] * v[1] + g1[3][2] * v[2];
  }
}