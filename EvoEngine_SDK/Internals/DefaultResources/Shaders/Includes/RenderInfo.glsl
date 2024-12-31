
struct RenderInfo {
  float shadow_split_0;
  float shadow_split_1;
  float shadow_split_2;
  float shadow_split_3;

  int shadow_sample_size;
  int debug_visualization;
  float shadow_seam_fix;
  float padding;

  float strand_subdivision_x;
  float strand_subdivision_y;
  int strand_subdivision_max_x;
  int strand_subdivision_max_y;

  int directional_light_size;
  int point_light_size;
  int spot_light_size;
  int brdf_lut_map_index;
};

layout(set = EE_RENDER_INFO_BLOCK_SET, binding = EE_RENDER_INFO_BLOCK_BINDING) uniform EE_RENDER_INFO_BLOCK {
  RenderInfo EE_RENDER_INFO;
};