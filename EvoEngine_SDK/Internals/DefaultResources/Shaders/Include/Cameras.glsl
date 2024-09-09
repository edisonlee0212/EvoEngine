
struct Camera {
  mat4 projection;
  mat4 view;
  mat4 projection_view;
  mat4 inverse_projection;
  mat4 inverse_view;
  mat4 inverse_projection_view;
  vec4 clear_color;
  vec4 reserved_1;
  vec4 reserved_2;
  int skybox_tex_index;
  int irradiance_map_index;
  int prefiltered_map_index;
  int use_clear_color;
};

// Camera
layout(set = EE_PER_FRAME_SET, binding = EE_CAMERA_BLOCK_BINDING) readonly buffer EE_CAMERA_BLOCK {
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
  return EE_CAMERAS[camera_index].reserved_1.x;
}

float EE_CAMERA_FAR(int camera_index) {
  return EE_CAMERAS[camera_index].reserved_1.y;
}

float EE_CAMERA_TAN_FOV(int camera_index) {
  return EE_CAMERAS[camera_index].reserved_1.z;
}

float EE_CAMERA_TAN_HALF_FOV(int camera_index) {
  return EE_CAMERAS[camera_index].reserved_1.w;
}

float EE_CAMERA_RESOLUTION_X(int camera_index) {
  return EE_CAMERAS[camera_index].reserved_2.x;
}

float EE_CAMERA_RESOLUTION_Y(int camera_index) {
  return EE_CAMERAS[camera_index].reserved_2.y;
}

float EE_CAMERA_RESOLUTION_RATIO(int camera_index) {
  return EE_CAMERAS[camera_index].reserved_2.z;
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