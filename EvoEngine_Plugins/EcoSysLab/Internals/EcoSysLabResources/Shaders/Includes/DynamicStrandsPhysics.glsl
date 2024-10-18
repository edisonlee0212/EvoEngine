
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

struct vec6 {
  float v[6];
};

struct mat6 {
  vec6 v[6];
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

  /*
  vec4 omega;

  mat4 j_omega0;
  mat4 j_omega1;

  mat4 g0;
  mat4 g1;

  mat4 j_omega_g0;
  mat4 j_omega_g1;

  vec4 connector0;
  vec4 connector1;

  vec4 stretch_violation;
  vec4 bending_and_torsion_violation;

  vec4 delta_lambda_stretch;
  vec4 delta_lambda_bending_and_torsion;

  vec4 x0_correction;
  vec4 x1_correction;

  vec4 q0_correction;
  vec4 q1_correction;

  mat6 A;
  float x[8];
  float y[8];
  float z[8];
  float D[8];
  float rhs[8];
  */
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




mat3 compute_inverse_inertia_tensor_w(in StrandSegment strand_segment) {
  mat3 rot = mat3_cast(strand_segment.q);
  mat3 diag =
      mat3(strand_segment.inv_inertia_tensor_inv_mass.x, 0.0, 0.0, 0.0, strand_segment.inv_inertia_tensor_inv_mass.y,
           0.0, 0.0, 0.0, strand_segment.inv_inertia_tensor_inv_mass.z);
  return rot * diag * transpose(rot);
}