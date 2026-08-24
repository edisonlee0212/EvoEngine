
const uint EE_DDGI_MAX_VOLUME_COUNT = 8u;
const uint EE_REFLECTION_PROBE_MAX_COUNT = 32u;

struct DdgiVolumeInfo {
  vec4 first_probe;
  vec4 probe_step_x;
  vec4 probe_step_y;
  vec4 probe_step_z;
  vec4 probe_counts;
  ivec4 probe_scroll_and_priority;
  uvec4 atlas_parameters;
  vec4 volume_parameters;
  vec4 lighting_parameters;
  uvec4 identity_and_flags;
};

struct ReflectionProbeInfo {
  mat4 world_to_probe;
  vec4 shape_parameters;
  vec4 projection_parameters;
  vec4 lighting_parameters;
  uvec4 identity_and_flags;
};

struct RenderInfo {
  float shadow_split_0;
  float shadow_split_1;
  float shadow_split_2;
  float shadow_split_3;

  int reserved_0;
  int debug_visualization;
  float shadow_cascade_transition_width;
  float indirect_lighting_intensity;

  float strand_subdivision_x;
  float strand_subdivision_y;
  int strand_subdivision_max_x;
  int strand_subdivision_max_y;

  int directional_light_size;
  int point_light_size;
  int spot_light_size;
  int brdf_lut_map_index;

  ivec4 shadow_debug_parameters;
  vec4 shadow_fade_parameters;
  uvec4 emissive_triangle_parameters;
  uvec4 ddgi_volume_header;
  DdgiVolumeInfo ddgi_volumes[EE_DDGI_MAX_VOLUME_COUNT];
  uvec4 reflection_probe_header;
  ReflectionProbeInfo reflection_probes[EE_REFLECTION_PROBE_MAX_COUNT];
};

layout(set = EE_RENDER_INFO_BLOCK_SET, binding = EE_RENDER_INFO_BLOCK_BINDING) uniform EE_RENDER_INFO_BLOCK {
  RenderInfo EE_RENDER_INFO;
};
