public struct EeSsrConstants {
  public int camera_index;
  public float max_distance;
  public float distance_confidence;
  public int iteration_count;
  public int initial_steps;
  public float thickness;
};

[[vk::push_constant]]
public ConstantBuffer<EeSsrConstants> EE_SSR_CONSTANTS;
