


struct Strand {
  int begin_segment_handle;
  int end_segment_handle;

  int begin_connection_handle;
  int end_connection_handle;
};

struct Segment {
  int prev_handle;
  int next_handle;
  int strand_handle;
  float inv_mass;

  vec4 color;

  vec4 q0;
  vec4 q;
  vec4 last_q;
  vec4 old_q;
  vec4 torque_rest_length;

  float radius;
  float shearing_stiffness;
  float stretching_stiffness;
  float damping;

  vec4 inertia_tensor_particle_0_handle;
  vec4 inv_inertia_tensor_particle_1_handle;
  
  mat4 inertia_w;
  mat4 inv_inertia_w;

  float bend_twist_strain0;
  float bend_twist_strain1;
  float stretch_shear_strain;
  float padding;
};

struct Particle {
  vec4 x0_damping;
  vec4 x;
  vec4 last_x;
  vec4 old_x;
  vec4 acceleration_inv_mass;

  int node_handle;
  int strand_handle;
  int segment_handle;
  float connectivity_strain;
};

struct Connection {
  int segment0_handle;
  int segment1_handle;
  int segment0_particle_handle;
  int segment1_particle_handle;

  vec4 rest_darboux_vector;
  float bending_stiffness;
  float twisting_stiffness;
  int prev_handle;
  int next_handle;
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

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 3) buffer CONNECTIONS_BLOCK {
  Connection connections[];
};