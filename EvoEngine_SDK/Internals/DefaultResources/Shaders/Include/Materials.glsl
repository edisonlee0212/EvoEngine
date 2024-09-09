
struct MaterialProperties {
  int albedo_map_index;
  int normal_map_index;
  int metallic_map_index;
  int roughness_map_index;

  int ao_texture_index;
  bool cast_shadow;
  bool receive_shadow;
  bool enable_shadow;

  vec4 albedo;
  vec4 sss_c;
  vec4 sss_r;

  float metallic;
  float roughness;
  float ambient_occulusion;
  float emission;
};

layout(set = EE_PER_FRAME_SET, binding = EE_MATERIAL_BLOCK_BINDING) readonly buffer EE_MATERIAL_BLOCK {
  MaterialProperties EE_MATERIAL_PROPERTIES[];
};