layout(push_constant) uniform STRANDS_RENDER_CONSTANTS {
  int EE_INSTANCE_INDEX;
  int EE_CAMERA_INDEX;
  uint vertex_count;
  uint triangle_count;
  int color_mode;
  int bark_material_index;
  int inner_wood_material_index;
  float uv_height_factor;
  float uv_circum_factor;
};

// color mode constants
#define COLOR_STANDARD 0
#define COLOR_NORMALS 1
#define COLOR_UVS 2
