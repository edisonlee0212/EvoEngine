
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

  int use_polar_coordinates_for_uv;
};