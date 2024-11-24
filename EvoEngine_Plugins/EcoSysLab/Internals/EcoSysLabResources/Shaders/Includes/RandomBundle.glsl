
#define BUNDLE_MAX_CONNECTION 16

struct SegmentPair {
  int handle0;
  int handle1;
  int valid;
  int padding;
  vec4 stiffness;
};

layout(std430, set = 1, binding = 0) buffer SEGMENT_PAIR_BLOCK {
  SegmentPair segment_pairs[];
};

struct SegmentData {
  vec4 particle0_position_correction_max_strain;
  vec4 particle1_position_correction_max_strain;
  vec4 q_correction;
  int pair_handles[BUNDLE_MAX_CONNECTION];
  vec4 particle0_offset[BUNDLE_MAX_CONNECTION];
  vec4 particle1_offset[BUNDLE_MAX_CONNECTION];
  vec4 rest_darboux_vectors[BUNDLE_MAX_CONNECTION];
};

layout(std430, set = 1, binding = 1) buffer SEGMENT_DATA_BLOCK {
  SegmentData segment_data_list[];
};