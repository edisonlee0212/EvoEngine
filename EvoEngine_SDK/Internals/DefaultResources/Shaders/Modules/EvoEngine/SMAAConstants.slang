// @evoengine-dialect native
public struct EeSmaaConstants {
  public float4 metrics;
  public float4 subsample_indices;
  public int tone_mapped;
  public int debug_mode;
  public int padding0;
  public int padding1;
};

[[vk::push_constant]]
public ConstantBuffer<EeSmaaConstants> EE_SMAA_CONSTANTS;
