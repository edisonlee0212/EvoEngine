
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

  int displacement_map_index;
  float displacement_intensity;

  float subsurface_factor;
  float specular;
  float specular_tint;
  float sheen;

  float sheen_tint;
  float clear_coat;
  float clear_coat_roughness;
  float ior;

  float transmission;
  float transmission_roughness;
  int vertex_color_only;
  int padding_mat_1;
};

layout(set = EE_MATERIALS_BLOCK_SET, binding = EE_MATERIALS_BLOCK_BINDING) readonly buffer EE_MATERIAL_BLOCK {
  MaterialProperties EE_MATERIAL_PROPERTIES[];
};