
struct StrandSegment {
  int prev_handle;
  int next_handle;
  int strand_handle;
  int index;

  vec4 rotation;

  int start_particle_index;
  int end_particle_index;

  int neighbors[10];
};

struct Strand {
  int first_strand_segment_handle;
  int last_strand_segment_handle;
};

struct StrandSegmentParticle {
  vec4 position_thickness;
  vec4 color;
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 0) readonly buffer REF_STRAND_SEGMENTS_BLOCK {
  StrandSegment ref_strand_segments[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 1) readonly buffer REF_STRANDS_BLOCK {
  Strand ref_strands[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 2) readonly buffer REF_STRAND_SEGMENT_PARTICLES_BLOCK {
  StrandSegmentParticle ref_strand_segment_particles[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 3) buffer STRAND_SEGMENTS_BLOCK {
  StrandSegment strand_segments[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 4) buffer STRANDS_BLOCK {
  Strand strands[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 5) buffer STRAND_SEGMENT_PARTICLES_BLOCK {
  StrandSegmentParticle strand_segment_particles[];
};

