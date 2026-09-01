[[vk::push_constant]] public cbuffer STRANDS_RENDER_CONSTANTS {
  public int EE_INSTANCE_INDEX;
  public int EE_CAMERA_INDEX;

  public float u_multiplier;
  public float v_multiplier;

  public uint tetrahedrons_size;
  public float alpha;
  public float bifurcation_alpha;
  public float max_dist_squared;

  public int render_complex;
  public int vertex_colors;

  public int inner_wood_material_index;
  public int snow_material_index;
  public float global_extrusion_distance;
  public float break_threshold;

  public int use_polar_coordinates_for_uv;
  public int bark_material_index;
  public int render_material_index;
};
