
struct StrandSegment {
  int prev_handle;
  int next_handle;
  int strand_handle;
  uint index;
  int neighbots[8];
  vec4 end_position_thickness;
  vec4 end_color;
  quat rotation;
};

struct Strand {
  int strand_segment_handles_offset;
  int strand_segment_handles_size;
  int first_strand_segment_handle;
  int last_strand_segment_handle;

  vec3 start_position_thickness;

  vec4 start_color;
};

layout(std430, set = STRAND_SEGMENTS_BLOCK_SET,
       binding = STRAND_SEGMENTS_BLOCK_BINDING) readonly buffer STRAND_SEGMENTS_BLOCK {
  StrandSegment strand_segments[];
};

layout(std430, set = STRANDS_BLOCK_SET,
       binding = STRANDS_BLOCK_BINDING) readonly buffer STRANDS_BLOCK {
  Strand strands[];
};

layout(std430, set = STRAND_SEGMENT_HANDLE_BLOCK_SET,
       binding = STRAND_SEGMENT_HANDLE_BLOCK_BINDING) readonly buffer STRAND_SEGMENT_HANDLE_BLOCK {
  int strand_segment_handles[];
};