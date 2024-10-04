
struct StrandSegment {
  int prev_handle;
  int next_handle;
  int strand_handle;
  int index;
  int neighbots[8];
  vec4 end_position_thickness;
  vec4 end_color;
  vec4 rotation;
};

struct Strand {
  int strand_segment_handles_offset;
  int strand_segment_handles_size;
  int first_strand_segment_handle;
  int last_strand_segment_handle;

  vec4 start_position_thickness;

  vec4 start_color;
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 0) readonly buffer REF_STRAND_SEGMENTS_BLOCK {
  StrandSegment ref_strand_segments[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 1) readonly buffer REF_STRANDS_BLOCK {
  Strand ref_strands[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 2) readonly buffer REF_STRAND_SEGMENT_HANDLE_BLOCK {
  int ref_strand_segment_handles[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 3) buffer STRAND_SEGMENTS_BLOCK {
  StrandSegment strand_segments[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 4) buffer STRANDS_BLOCK {
  Strand strands[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 5) buffer STRAND_SEGMENT_HANDLE_BLOCK {
  int strand_segment_handles[];
};

