[[vk::push_constant]] public cbuffer STRANDS_RENDER_CONSTANTS {
  public int EE_INSTANCE_INDEX;
  public int EE_CAMERA_INDEX;
  public uint vertex_count;
  public uint triangle_count;
  public int color_mode;
  public int bark_material_index;
  public int inner_wood_material_index;
  public float uv_height_factor;
  public float uv_circum_factor;
  public float fracture_distance;
};

// color mode constants
public static const uint COLOR_STANDARD = 0u;
public static const uint COLOR_NORMALS = 1u;
public static const uint COLOR_UVS = 2u;
public static const uint COLOR_NEIGHBORS = 3u;
