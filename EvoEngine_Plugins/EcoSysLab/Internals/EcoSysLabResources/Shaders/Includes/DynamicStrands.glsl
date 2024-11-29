


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
  vec4 angular_v;

  vec4 torque_rest_length;

  float radius;
  float shearing_alpha;
  float stretching_alpha;
  float damping;

  vec4 inertia_tensor_particle_0_handle;
  vec4 inv_inertia_tensor_particle_1_handle;
  
  mat4 inertia_w;
  mat4 inv_inertia_w;

  vec4 stretch_shear_strain_original_inv_mass;

  vec4 max_stretch_shear_strain;
};

struct Particle {
  vec4 x0_damping;
  vec4 x_node_handle;
  vec4 last_x_strand_handle;
  vec4 v_segment_handle;
  vec4 acceleration_connectivity_strain;

  int selected;
  int highlighted;
  int connection_handle;
  int padding2;

};

struct UniformParticle {
  vec4 position_t;
  int segment_handle;
  int node_index;
  int segment_index;
  float distance_to_boundary;
};

struct Connection {
  int segment0_handle;
  int segment1_handle;
  int segment0_particle_handle;
  int segment1_particle_handle;

  vec4 rest_darboux_vector;
  float bending_alpha;
  float twisting_alpha;
  int prev_handle;
  int next_handle;

  vec4 bend_twist_strain_valid;

  vec4 max_bend_twist_strain;
};

struct DelaunayTetrahedron {
  int indices[4];
  int neighbors[4];
  int render_neighbor[4];
  float neighbor_circumference[4];
  vec4 color;  // for debugging
  uint task_looked_at;
  uint mesh_looked_at;
  int inside;
  int triangles_accepted;
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

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 3) buffer UNIFORM_PARTICLES_BLOCK {
  UniformParticle uniform_particles[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 4) buffer CONNECTIONS_BLOCK {
  Connection connections[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 5) buffer DELAUNAY_TETRAHEDRON_BLOCK {
  DelaunayTetrahedron delaunay_tetrahedrons[];
};