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
  public float texture_diameter;

  public int use_polar_coordinates_for_uv;
  public int bark_material_index;
  public int render_material_index;
};

// Matches BranchesRenderParameters::VertexColors
public static const int COLOR_DEFAULT = 0;
public static const int COLOR_NORMALS = 1;
public static const int COLOR_TANGENTS = 2;
public static const int COLOR_GROUPS = 3;
public static const int COLOR_DEGREE = 4;
public static const int COLOR_BARK = 5;
public static const int COLOR_NORMAL_QUATERNION = 6;
public static const int COLOR_UP = 7;
public static const int COLOR_INIT_UP = 8;
public static const int COLOR_AXIS = 9;
public static const int COLOR_INIT_AXIS = 10;
public static const int COLOR_INIT_ANGLE = 11;
public static const int COLOR_VOLUME_CHANGE_HEATMAP = 12;
