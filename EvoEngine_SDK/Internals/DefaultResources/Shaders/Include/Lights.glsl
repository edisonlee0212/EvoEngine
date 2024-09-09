
// Lights
struct DirectionalLight {
  vec3 direction;
  vec4 diffuse;
  vec3 specular;
  mat4 light_space_matrix[4];
  vec4 light_frustum_width;
  vec4 light_frustum_distance;
  vec4 reserved_parameters;
  int viewport_x_offset;
  int viewport_y_offset;
  int viewport_x_size;
  int viewport_y_size;
};

struct PointLight {
  vec3 position;
  vec4 constant_linear_quadratic_far;
  vec4 diffuse;
  vec3 specular;
  mat4 light_space_matrix[6];
  vec4 reserved_parameters;
  int viewport_x_offset;
  int viewport_y_offset;
  int viewport_x_size;
  int viewport_y_size;
};

struct SpotLight {
  vec3 position;
  float padding0;

  vec3 direction;
  float padding1;

  mat4 light_space_matrix;
  vec4 cutoff_outer_inner_size_bias;
  vec4 constant_linear_quadratic_far;
  vec4 diffuse;
  vec3 specular;
  float padding3;
  int viewport_x_offset;
  int viewport_y_offset;
  int viewport_x_size;
  int viewport_y_size;
};

layout(set = EE_PER_FRAME_SET, binding = EE_DIRECTIONAL_LIGHT_BLOCK_BINDING) readonly buffer EE_DIRECTIONAL_LIGHT_BLOCK {
  DirectionalLight EE_DIRECTIONAL_LIGHTS[];
};

layout(set = EE_PER_FRAME_SET, binding = EE_POINT_LIGHT_BLOCK_BINDING) readonly buffer EE_POINT_LIGHT_BLOCK {
  PointLight EE_POINT_LIGHTS[];
};

layout(set = EE_PER_FRAME_SET, binding = EE_SPOT_LIGHT_BLOCK_BINDING) readonly buffer EE_SPOT_LIGHT_BLOCK {
  SpotLight EE_SPOT_LIGHTS[];
};