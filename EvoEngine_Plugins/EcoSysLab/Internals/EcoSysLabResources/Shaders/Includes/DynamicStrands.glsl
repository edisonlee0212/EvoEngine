
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
  int padding0;
  int padding1;
  int padding2;
};

struct Particle {
  vec3 x0;
  int padding0;

  vec3 x;
  int selected;

  vec3 last_x;
  int highlighted;

  vec3 v;
  int hop_distance_to_root;

  vec3 acceleration;
  int node_handle;
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
  float radius;

  vec3 torque;
  float rest_length;

  float max_young_modulus;
  float shear_stretch_alpha;
  float strength;
  float boundary_distance;

  vec2 profile_position;
  vec2 profile_polar_coordinate;

  vec3 inertia_tensor;
  float max_shear_stretch_strain;
  vec3 inv_inertia_tensor;
  float shear_stretch_strain_limit;

  mat4 inertia_w;
  mat4 inv_inertia_w;

  float shear_stretch_strain;
  int node_handle;
  float original_mass;
  int group_index;

  float extra_mass;
  float property1;
  float property2;
  float property3;

  Particle particle0;
  Particle particle1;
};

struct SegmentPair {
  int segment0_handle;
  int segment1_handle;
  float bend_twist_bundle_integrity;
  float connectivity_integrity;

  float bending_alpha;
  float twisting_alpha;
  float max_bending_modulus;
  float max_torsion_modulus;

  vec4 segment0_offset;
  vec4 segment1_offset;

  vec4 rest_darboux_vector;

  vec3 bending_twist_bundle_strain;
  float connectivity_strain;
  vec3 max_bending_twist_bundle_strain;
  float max_connectivity_strain;
  vec3 bending_twist_bundle_strain_limit;
  float connectivity_strain_limit;

  int tensile_lock;
  int compression_lock;
  int positional_lock;
  int rotational_lock;
};

#define BUNDLE_MAX_CONNECTION 64
struct SegmentData {
  vec4 particle0_position_correction;
  vec4 particle1_position_correction;
  vec4 q_correction;
  int pair_handles[BUNDLE_MAX_CONNECTION];
};

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
  int padding3;

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

struct Leaf {
  vec3 x0;
  int segment_handle;
  vec3 x;
  float attachment_integrity;
  vec3 last_x;
  float rotation_integrity;
  vec4 q0;
  vec4 q;
  vec4 last_q;

  vec3 scale;
  float inv_mass;

  vec3 position_offset;
  float original_mass;

  vec3 v;
  float position_strain;

  vec3 acceleration;
  float rotation_strain;

  vec3 angular_v;
  float position_alpha;

  vec3 torque;
  float rotation_alpha;

  vec3 inertia_tensor;
  float rotation_strain_limit;

  vec3 inv_inertia_tensor;
  float position_strain_limit;

  mat4 inertia_w;
  mat4 inv_inertia_w;

  float extra_mass;
  float property1;
  float property2;
  float property3;

  int selected;
  int highlighted;
  int padding0;
  int padding1;
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

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 3) buffer SEGMENT_PAIR_BLOCK {
  SegmentPair segment_pairs[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 4) buffer SEGMENT_DATA_BLOCK {
  SegmentData segment_data_list[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 5) buffer UNIFORM_PARTICLES_BLOCK {
  UniformParticle uniform_particles[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 6) buffer DELAUNAY_TETRAHEDRON_BLOCK {
  DelaunayTetrahedron delaunay_tetrahedrons[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 7) buffer HASHED_GRID_ELEMENTS_BLOCK {
  HashedGridElement hashed_grid_elements[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 8) buffer HASHED_GRID_CELL_STARTS_BLOCK {
  HashedGridCellStart hashed_grid_cell_starts[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 9) buffer FOLIAGE_BLOCK {
  Leaf foliage[];
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
