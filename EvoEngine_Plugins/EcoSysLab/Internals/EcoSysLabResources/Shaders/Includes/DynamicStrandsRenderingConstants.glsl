
layout(push_constant) uniform STRANDS_RENDER_CONSTANTS {
  int EE_INSTANCE_INDEX;
  int EE_CAMERA_INDEX;
  float u_multiplier;
  float v_multiplier;
  uint tetrahedrons_size;
  float alpha;
  float bifurcation_alpha;
  int render_complex;
  int vertex_colors;
};