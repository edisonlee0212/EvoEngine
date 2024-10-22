
#include "Math.glsl"

struct PerStrandData {
  int front_propagate_begin_connection_handle;
  int back_propagate_begin_connection_handle;
  int front_propagate_begin_segment_handle;
  int back_propagate_begin_segment_handle;
};

layout(std430, set = DYNAMIC_STRANDS_PHYSICS_SET, binding = 0) readonly buffer PER_STRAND_DATA_LIST_BLOCK {
  PerStrandData per_strand_data_list[];
};