// ProfilePacking.glsl — Shared GPU structs for 2D profile packing simulation.
// Ports the CPU StrandModelProfile particle physics to GPU compute shaders.

#define GRID_CELL_CAPACITY 4

struct ProfileParticle {
  vec2 position;
  vec2 last_position;
  vec2 acceleration;
  vec2 delta_position;
  uint enable;        // 0 or 1
  int strand_handle;
  int strand_segment_handle;
  int node_handle;
};

struct ProfileInfo {
  uint particle_offset;     // start index into global particle buffer
  uint particle_count;      // number of particles in this profile
  uint grid_offset;         // start index into global grid buffer (in cells)
  uint is_active;           // 0 = frozen, 1 = simulate this step

  vec2 grid_min_bound;      // spatial hash grid minimum bound
  float grid_cell_size;     // cell size (= 2.0, matching particle diameter)
  int grid_resolution_x;

  int grid_resolution_y;
  float particle_softness;
  float damping;
  float max_speed;
};

struct GridCell {
  uint count;
  int handles[GRID_CELL_CAPACITY];
  // Pad to 24 bytes total (uint + 4*int = 20 bytes, pad to align)
  int padding0;
  int padding1;
  int padding2;
};

layout(std430, set = PROFILE_PACKING_SET, binding = 0) buffer PARTICLES_BLOCK {
  ProfileParticle particles[];
};

layout(std430, set = PROFILE_PACKING_SET, binding = 1) buffer PROFILES_BLOCK {
  ProfileInfo profiles[];
};

layout(std430, set = PROFILE_PACKING_SET, binding = 2) buffer GRID_BLOCK {
  GridCell grid_cells[];
};

// Binary search to find which profile a global particle index belongs to.
uint find_profile(uint global_idx, uint total_profiles) {
  uint lo = 0;
  uint hi = total_profiles;
  while (lo < hi) {
    uint mid = (lo + hi) / 2;
    if (profiles[mid].particle_offset + profiles[mid].particle_count <= global_idx)
      lo = mid + 1;
    else
      hi = mid;
  }
  return lo;
}

// Get the grid cell index for a position within a profile.
int get_grid_cell_index(uint profile_idx, vec2 pos) {
  ProfileInfo info = profiles[profile_idx];
  ivec2 coord = ivec2(floor((pos - info.grid_min_bound) / info.grid_cell_size));
  coord = clamp(coord, ivec2(0), ivec2(info.grid_resolution_x - 1, info.grid_resolution_y - 1));
  return int(info.grid_offset) + coord.y * info.grid_resolution_x + coord.x;
}
