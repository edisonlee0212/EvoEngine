


struct Strand {
  int begin_segment_handle;
  int end_segment_handle;

  int begin_connection_handle;
  int end_connection_handle;

  int front_propagate_begin_segment_pair_handle;
  int back_propagate_begin_segment_pair_handle;
  int front_propagate_begin_segment_handle;
  int back_propagate_begin_segment_handle;

  int alternative_front_propagate_begin_segment_pair_handle;
  int alternative_back_propagate_begin_segment_pair_handle;
  int alternative_front_propagate_begin_segment_handle;
  int alternative_back_propagate_begin_segment_handle;

};

struct Node {
  int prev_handle;
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

  vec3 angular_v;
  float shear_stretch_valid;

  vec3 torque;
  float rest_length;

  float radius;
  float shearing_alpha;
  float stretching_alpha;
  float original_inv_mass;

  float max_shearing_modulus;
  float max_stretching_modulus;
  float moisture_content;
  float boundary_distance;

  vec3 inertia_tensor;
  int particle_0_handle;
  vec3 inv_inertia_tensor;
  int particle_1_handle;
  
  mat4 inertia_w;
  mat4 inv_inertia_w;

  vec2 shear_stretch_strain;
  float padding;
  int group_index;

  vec2 max_shear_stretch_strain;
  vec2 shear_stretch_strain_limit;
};

struct Particle {
  vec4 x0;
  vec3 x;
  int node_handle2;
  vec3 last_x;
  int strand_handle2;

  vec3 v;
  int segment_handle2;

  vec4 acceleration;

  int selected;
  int highlighted;
  int connection_handle;
  int hop_distance_to_root;

  int node_handle;
  int strand_handle;
  int segment_handle;
  int padding3;
};

struct SegmentPair {
  int segment0_handle;
  int segment1_handle;
  uint bend_twist_bundle_valid;
  uint connectivity_valid;

  float bending_alpha;
  float twisting_alpha;
  float max_bending_modulus;
  float max_torsion_modulus;

  vec4 segment0_particle0_offset;
  vec4 segment0_particle1_offset;

  vec4 segment1_particle0_offset;
  vec4 segment1_particle1_offset;

  vec4 rest_darboux_vector;

  vec4 bending_twist_bundle_strain;
  vec4 max_bending_twist_bundle_strain;
  vec4 bending_twist_bundle_strain_limit;
};

#define BUNDLE_MAX_CONNECTION 16
struct SegmentData {
  vec4 particle0_position_correction;
  vec4 particle1_position_correction;
  vec4 q_correction;
  int pair_handles[BUNDLE_MAX_CONNECTION];
};

struct UniformParticle {
  vec4 position_t;
  int segment_handle;
  int node_index;
  int segment_index;
  float boundary_distance;
  int next_particle_handle;
  int prev_particle_handle;
  int next_node_index;
  int strand_index;
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

struct HashedGridElement {
  uint cell_id;
  uint segment_handle;
  uint padding0;
  uint padding1;
};

struct HashedGridCellStart {
  uint start_index;
  uint padding0;
  uint padding1;
  uint padding2;
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 0) buffer STRANDS_BLOCK {
  Strand strands[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 1) buffer NODES_BLOCK {
  Node nodes[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 2) buffer SEGMENTS_BLOCK {
  Segment segments[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 3) buffer PARTICLES_BLOCK {
  Particle particles[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 4) buffer SEGMENT_PAIR_BLOCK {
  SegmentPair segment_pairs[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 5) buffer SEGMENT_DATA_BLOCK {
  SegmentData segment_data_list[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 6) buffer UNIFORM_PARTICLES_BLOCK {
  UniformParticle uniform_particles[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 7) buffer DELAUNAY_TETRAHEDRON_BLOCK {
  DelaunayTetrahedron delaunay_tetrahedrons[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 8) buffer HASHED_GRID_ELEMENTS_BLOCK {
  HashedGridElement hashed_grid_elements[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 9) buffer HASHED_GRID_CELL_STARTS_BLOCK {
  HashedGridCellStart hashed_grid_cell_starts[];
};


#define HASH_GRID_CELL_SIZE 2 << 15

uint hash_cell_coordinate(in ivec3 cell_coordinate) {
  uint p1 = 73856093;  // some large primes
  uint p2 = 19349663;
  uint p3 = 83492791;
  int n = int(p1 * cell_coordinate.x ^ p2 * cell_coordinate.y ^ p3 * cell_coordinate.z);
  n %= HASH_GRID_CELL_SIZE;
  return uint(n);
}
