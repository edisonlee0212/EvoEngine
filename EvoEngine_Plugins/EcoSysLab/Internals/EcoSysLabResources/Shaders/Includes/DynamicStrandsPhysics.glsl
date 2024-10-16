
#include "Math.glsl"

struct PerStrandData {
  int front_propagate_begin_constraint_handle;
  int back_propagate_begin_constraint_handle;
  int front_propagate_begin_segment_handle;
  int back_propagate_begin_segment_handle;
};

layout(std430, set = DYNAMIC_STRANDS_PHYSICS_SET, binding = 0) readonly buffer PER_STRAND_DATA_LIST_BLOCK {
  PerStrandData per_strand_data_list[];
};

struct Constraint {
  vec4 rest_darboux_vector;
  vec4 indices_length;

  float bending_stiffness;
  float twisting_stiffness;
  int next_constraint_handle;
  int prev_constraint_handle;
};

layout(std430, set = DYNAMIC_STRANDS_PHYSICS_SET, binding = 1) buffer CONSTRAINTS_BLOCK {
  Constraint constraints[];
};