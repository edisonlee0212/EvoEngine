// SurfaceMeshGeneration.glsl — Shared GPU structs for developmental strand surface mesh generation.
// Used by ExtractAndProject.comp and StitchContours.comp.

// Re-declare particle/profile structs (identical to ProfilePacking.glsl)
// so that surface mesh shaders are self-contained with their own binding layout.

struct ProfileParticle {
  vec2 position;
  vec2 last_position;
  vec2 acceleration;
  vec2 delta_position;
  uint enable;        // 0=disabled, 1=active (boundary), 2=frozen (interior)
  int strand_handle;
  int strand_segment_handle;
  int node_handle;
  uint birth_step;
  uint _pad1;
};

struct ProfileInfo {
  uint particle_offset;
  uint particle_count;
  uint grid_offset;
  uint is_active;

  vec2 grid_min_bound;
  float grid_cell_size;
  int grid_resolution_x;

  int grid_resolution_y;
  float particle_softness;
  float damping;
  float max_speed;
};

// Skeleton node (matches C++ GpuSkeletonNode, 80 bytes).
struct SkeletonNode {
  vec3 global_position;
  float length;

  vec4 regulated_global_rotation;   // quaternion (x, y, z, w)

  vec3 global_end_position;
  float strand_radius;

  float root_distance;
  float max_root_distance;
  int parent_handle;
  uint is_end_node;

  uint boundary_particle_count;
  uint total_particle_count;
  uint profile_index;               // index into contour_infos / profiles
  float _pad0;
};

// Contour info per profile (matches C++ GpuContourInfo, 16 bytes).
struct ContourInfo {
  uint vertex_offset;   // start index in vertices[]
  uint vertex_count;    // number of boundary vertices in sorted angular order
  int node_handle;      // skeleton node handle
  uint _pad;
};

// Surface vertex (matches engine Vertex struct, 80 bytes).
struct SurfaceVertex {
  vec3 position;
  float vertex_info1;   // tissue_type
  vec3 normal;
  float vertex_info2;   // wound_state
  vec3 tangent;
  float vertex_info3;   // normalized root_distance
  vec4 color;
  vec2 tex_coord;
  vec2 vertex_info4;    // (birth_step, is_boundary)
};

// SSBO bindings — all on descriptor set SURFACE_MESH_SET.
layout(std430, set = SURFACE_MESH_SET, binding = 0) readonly buffer PARTICLES_BLOCK {
  ProfileParticle particles[];
};

layout(std430, set = SURFACE_MESH_SET, binding = 1) readonly buffer PROFILES_BLOCK {
  ProfileInfo profiles[];
};

layout(std430, set = SURFACE_MESH_SET, binding = 2) readonly buffer SKELETON_BLOCK {
  SkeletonNode skeleton_nodes[];
};

layout(std430, set = SURFACE_MESH_SET, binding = 3) buffer CONTOUR_INFO_BLOCK {
  ContourInfo contour_infos[];
};

layout(std430, set = SURFACE_MESH_SET, binding = 4) buffer VERTEX_BLOCK {
  SurfaceVertex vertices[];
};

layout(std430, set = SURFACE_MESH_SET, binding = 5) buffer INDEX_BLOCK {
  uint index_data[];  // flat uint array — avoids uvec3 stride-16 padding in std430
};

layout(std430, set = SURFACE_MESH_SET, binding = 6) buffer COUNTERS_BLOCK {
  uint total_vertices;
  uint total_triangles;
};

// Quaternion rotation helper.
vec3 quat_rotate(vec4 q, vec3 v) {
  vec3 t = 2.0 * cross(q.xyz, v);
  return v + q.w * t + cross(q.xyz, t);
}
