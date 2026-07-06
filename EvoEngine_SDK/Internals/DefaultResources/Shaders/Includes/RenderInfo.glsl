
struct RenderInfo {
  float shadow_split_0;
  float shadow_split_1;
  float shadow_split_2;
  float shadow_split_3;

  int shadow_sample_size;
  int debug_visualization;
  float shadow_cascade_transition_width;
  float ddgi_indirect_intensity;

  float strand_subdivision_x;
  float strand_subdivision_y;
  int strand_subdivision_max_x;
  int strand_subdivision_max_y;

  int directional_light_size;
  int point_light_size;
  int spot_light_size;
  int brdf_lut_map_index;

  vec4 ddgi_first_probe;
  vec4 ddgi_probe_step_x;
  vec4 ddgi_probe_step_y;
  vec4 ddgi_probe_step_z;
  vec4 ddgi_probe_counts;
  vec4 ddgi_probe_scroll_offset;
  vec4 ddgi_atlas_parameters;
  vec4 ddgi_volume_parameters;
  vec4 ddgi_sampling_parameters;
  ivec4 shadow_debug_parameters;
  vec4 shadow_fade_parameters;
};

layout(set = EE_RENDER_INFO_BLOCK_SET, binding = EE_RENDER_INFO_BLOCK_BINDING) uniform EE_RENDER_INFO_BLOCK {
  RenderInfo EE_RENDER_INFO;
};
