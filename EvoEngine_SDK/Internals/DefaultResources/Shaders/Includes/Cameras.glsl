
struct Camera {
  mat4 projection;
  mat4 view;
  mat4 projection_view;
  mat4 inverse_projection;
  mat4 inverse_view;
  mat4 inverse_projection_view;
  mat4 previous_projection_view;
  mat4 unjittered_projection_view;
  mat4 previous_unjittered_projection_view;
  vec4 clear_color;
  vec4 jitter;
  float resolution_x;
  float resolution_y;
  float fade_ratio;
  float fade_factor;
  int skybox_tex_index;
  int irradiance_map_index;
  int prefiltered_map_index;
  int use_clear_color;

  // Ray tracing
  uint firefly_clamp_enabled;
  float gamma;
  uint sample_size;
  uint bounce;
  float firefly_clamp_threshold;
  uint auto_spp_enabled;
  uint auto_spp_min_samples;
  uint auto_spp_max_samples;
  float auto_spp_convergence_threshold;
  uint emissive_triangle_nee_enabled;
  uint auto_spp_padding1;
  uint auto_spp_padding2;
};

// Camera
layout(set = EE_CAMERAS_BLOCK_SET, binding = EE_CAMERAS_BLOCK_BINDING) readonly buffer EE_CAMERA_BLOCK {
  Camera EE_CAMERAS[];
};

vec3 EE_DEPTH_TO_CLIP_POS(vec2 tex_coords, float ndcDepth);
vec3 EE_DEPTH_TO_WORLD_POS(int camera_index, vec2 tex_coords, float ndcDepth);
vec3 EE_DEPTH_TO_VIEW_POS(int camera_index, vec2 tex_coords, float ndcDepth);

vec3 EE_CAMERA_LEFT(int camera_index) {
  return EE_CAMERAS[camera_index].view[0].xyz;
}

vec3 EE_CAMERA_RIGHT(int camera_index) {
  return -EE_CAMERAS[camera_index].view[0].xyz;
}

vec3 EE_CAMERA_UP(int camera_index) {
  return EE_CAMERAS[camera_index].view[1].xyz;
}

vec3 EE_CAMERA_DOWN(int camera_index) {
  return -EE_CAMERAS[camera_index].view[1].xyz;
}

vec3 EE_CAMERA_BACK(int camera_index) {
  return EE_CAMERAS[camera_index].view[2].xyz;
}

vec3 EE_CAMERA_FRONT(int camera_index) {
  return -EE_CAMERAS[camera_index].view[2].xyz;
}

vec3 EE_CAMERA_POSITION(int camera_index) {
  return EE_CAMERAS[camera_index].inverse_view[3].xyz;
}

float EE_CAMERA_NEAR(int camera_index) {
  float a = EE_CAMERAS[camera_index].projection[2][2];
  float b = EE_CAMERAS[camera_index].projection[2][3];
  return b / (a - 1.f);
}

float EE_CAMERA_FAR(int camera_index) {
  float a = EE_CAMERAS[camera_index].projection[2][2];
  float b = EE_CAMERAS[camera_index].projection[2][3];
  return b / (a + 1.f);
}

float EE_CAMERA_TAN_HALF_FOV(int camera_index) {
  return EE_CAMERAS[camera_index].projection[1][1];
}

float EE_CAMERA_RESOLUTION_X(int camera_index) {
  return EE_CAMERAS[camera_index].resolution_x;
}

float EE_CAMERA_RESOLUTION_Y(int camera_index) {
  return EE_CAMERAS[camera_index].resolution_y;
}

float EE_CAMERA_RESOLUTION_RATIO(int camera_index) {
  return EE_CAMERAS[camera_index].resolution_x / EE_CAMERAS[camera_index].resolution_y;
}

float EE_CAMERA_FADE_RATIO(int camera_index) {
  return EE_CAMERAS[camera_index].fade_ratio;
}

float EE_CAMERA_FADE_FACTOR(int camera_index) {
  return EE_CAMERAS[camera_index].fade_factor;
}

float EE_LINEARIZE_DEPTH(int camera_index, float ndcDepth) {
  float near = EE_CAMERA_NEAR(camera_index);
  float far = EE_CAMERA_FAR(camera_index);
  return near * far / (far - ndcDepth * (far - near));
}

vec3 EE_DEPTH_TO_WORLD_POS(int camera_index, vec2 tex_coords, float ndcDepth) {
  vec4 clipPos = vec4(EE_DEPTH_TO_CLIP_POS(tex_coords, ndcDepth), 1.0);
  vec4 worldPos = EE_CAMERAS[camera_index].inverse_projection_view * clipPos;
  worldPos = worldPos / worldPos.w;
  return worldPos.xyz;
}

vec3 EE_DEPTH_TO_VIEW_POS(int camera_index, vec2 tex_coords, float ndcDepth) {
  vec4 clipPos = vec4(EE_DEPTH_TO_CLIP_POS(tex_coords, ndcDepth), 1.0);
  vec4 viewPos = EE_CAMERAS[camera_index].inverse_projection * clipPos;
  viewPos = viewPos / viewPos.w;
  return viewPos.xyz;
}

vec3 EE_DEPTH_TO_CLIP_POS(vec2 tex_coords, float ndcDepth) {
  vec4 clipPos = vec4(tex_coords * 2 - vec2(1), ndcDepth, 1.0);
  return clipPos.xyz;
}

float EE_PIXEL_DISTANCE(int camera_index, in vec3 worldPosA, in vec3 worldPosB) {
  vec4 coordA = EE_CAMERAS[camera_index].projection_view * vec4(worldPosA, 1.0);
  vec4 coordB = EE_CAMERAS[camera_index].projection_view * vec4(worldPosB, 1.0);
  vec2 screenSize = vec2(EE_CAMERA_RESOLUTION_X(camera_index), EE_CAMERA_RESOLUTION_Y(camera_index));
  coordA = coordA / coordA.w;
  coordB = coordB / coordB.w;
  return distance(coordA.xy * screenSize / 2.0, coordB.xy * screenSize / 2.0);
}
