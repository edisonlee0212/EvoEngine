struct UniformParticle {
  vec4 position_t;
  vec4 normal_deg;
  vec4 tangent;

  vec2 profile_position;
  vec2 profile_polar_coordinate;

  vec4 override_color;

  int segment_handle;
  int node_index;
  int segment_index;
  float boundary_distance;
  int next_particle_handle;
  int prev_particle_handle;
  int next_node_index;
  int strand_index;

  int is_single_strand_particle;  // amount of connected tetrahedrons
  float local_extrusion_distance;
  int is_on_surface;  // mark if particle is on the surface of the mesh to initialize the normal
  int is_bark;

  vec4 initial_position;
  vec4 normal_q;
};

struct DelaunayTetrahedron {
  int indices[4];
  int neighbor_tet_ids[4];
  int render_neighbor[4];
  int is_bark[4];
  vec4 color;  // for debugging
  uint task_looked_at;
  uint mesh_looked_at;
  int inside;
  int triangles_accepted;
  float sidelengths[6];
  int padding0;
  int padding1;
  int segment_pair_index[6];
  int inside_at_init;
  int padding2;
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 8) buffer UNIFORM_PARTICLES_BLOCK {
  UniformParticle uniform_particles[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 9) buffer DELAUNAY_TETRAHEDRON_BLOCK {
  DelaunayTetrahedron delaunay_tetrahedrons[];
};
