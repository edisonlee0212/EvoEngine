
layout(push_constant) uniform STRANDS_RENDER_CONSTANTS {
  int EE_INSTANCE_INDEX;
  int EE_CAMERA_INDEX;

  float u_multiplier;
  float v_multiplier;

  uint tetrahedrons_size;
  float alpha;
  float bifurcation_alpha;
  float max_dist_squared;

  int render_complex;
  int vertex_colors;

  int inner_wood_material_index;
  int snow_material_index;
  float global_extrusion_distance;
  float break_threshold;
  float texture_diameter;

  int use_polar_coordinates_for_uv;
  int bark_material_index;
};

// Matches BranchesRenderParameters::VertexColors
#define COLOR_DEFAULT 0
#define COLOR_NORMALS 1
#define COLOR_TANGENTS 2
#define COLOR_GROUPS 3
#define COLOR_DEGREE 4
#define COLOR_BARK 5
#define COLOR_NORMAL_QUATERNION 6
#define COLOR_UP 7
#define COLOR_INIT_UP 8
#define COLOR_AXIS 9
#define COLOR_INIT_AXIS 10
#define COLOR_INIT_ANGLE 11
#define COLOR_VOLUME_CHANGE_HEATMAP 12
