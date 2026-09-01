public struct EeSsrConstants {
  public int camera_index;
  public float max_distance;
  public float distance_confidence;
  public int iteration_count;
  public int binary_search_iteration_count;
  public float thickness;
  public float start_bias;
  public int debug_mode;
  public int temporal_enabled;
  public int temporal_history_valid;
};

[[vk::push_constant]]
public ConstantBuffer<EeSsrConstants> EE_SSR_CONSTANTS;
