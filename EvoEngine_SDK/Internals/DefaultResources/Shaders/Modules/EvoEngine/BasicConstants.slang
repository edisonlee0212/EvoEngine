public struct EeBasicConstants {
  public int instance_index;
  public int camera_index;
  public int light_split_index;
  public uint meshlet_culling_flags;
};

[[vk::push_constant]]
public ConstantBuffer<EeBasicConstants> EE_BASIC_CONSTANTS;
