


struct Strand {
  int begin_segment_handle;
  int end_segment_handle;

  int segment_size;
  int padding0;
};

struct Segment {
  int prev_handle;
  int next_handle;
  int strand_handle;
  float inv_mass;

  vec4 color_radius;

  vec4 q0;
  vec4 q;
  vec4 last_q;
  vec4 old_q;
  vec4 torque_rest_length;

  float shearing_stiffness;
  float stretching_stiffness;
  float damping;
  float padding1;

  vec4 inertia_tensor_particle_0_handle;
  vec4 inv_inertia_tensor_particle_1_handle;
  
  mat4 inertia_w;
  mat4 inv_inertia_w;

  int neighbors[8];

  vec4 p0;
  vec4 p1;
  vec4 original_gamma;
  vec4 d3;
  vec4 gamma;
};

struct Particle {
  vec4 x0_damping;
  vec4 x;
  vec4 last_x;
  vec4 old_x;
  vec4 acceleration_inv_mass;
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 0) buffer STRANDS_BLOCK {
  Strand strands[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 1) buffer SEGMENTS_BLOCK {
  Segment segments[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 2) buffer PARTICLES_BLOCK {
  Particle particles[];
};