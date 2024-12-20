
layout(push_constant) uniform STRANDS_RENDER_CONSTANTS {
  int base_index;
  int EE_CAMERA_INDEX;
  uint tetrahedrons_size;
  float alpha;
  float bifurcation_alpha;
  int render_complex;
  int vertex_colors;
};