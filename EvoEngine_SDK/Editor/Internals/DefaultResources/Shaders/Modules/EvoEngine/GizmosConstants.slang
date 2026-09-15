public struct EeGizmosConstants {
  public float4x4 model_matrix;
  public float4 color;
  public float size;
  public int camera_index;
  public uint strand_meshlet_offset;
  public uint strand_color_mode;
};

[[vk::push_constant]]
public ConstantBuffer<EeGizmosConstants> EE_GIZMOS_CONSTANTS;

public float4x4 EE_GET_GIZMO_SCALE_MATRIX() {
  const float size = EE_GIZMOS_CONSTANTS.size;
  return float4x4(size, 0.0f, 0.0f, 0.0f,
                  0.0f, size, 0.0f, 0.0f,
                  0.0f, 0.0f, size, 0.0f,
                  0.0f, 0.0f, 0.0f, 1.0f);
}
