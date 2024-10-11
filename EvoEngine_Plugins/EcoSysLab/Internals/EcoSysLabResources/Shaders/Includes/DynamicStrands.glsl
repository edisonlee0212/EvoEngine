
struct StrandSegment {
  int prev_handle;
  int next_handle;
  int strand_handle;
  int index;
  
  vec4 color_radius;

  vec4 q0;
  vec4 q;
  vec4 last_q;
  vec4 old_q;
  vec4 torque;

  vec4 x0_length;
  vec4 x;
  vec4 last_x;
  vec4 old_x;
  vec4 acceleration;

  vec4 inertia_tensor_mass;

  vec4 inv_inertia_tensor_inv_mass;

  mat4 inertia_w;
  mat4 inv_inertia_w;

  int neighbors[8];
};

struct Strand {
  int begin_segment_handle;
  int end_segment_handle;

  int segment_size;
  int padding0;
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 0) buffer STRAND_SEGMENTS_BLOCK {
  StrandSegment strand_segments[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 1) buffer STRANDS_BLOCK {
  Strand strands[];
};
