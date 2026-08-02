// @evoengine-dialect native
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
  float4x4 result = float4x4(1.0f);
  result[0] *= EE_GIZMOS_CONSTANTS.size;
  result[1] *= EE_GIZMOS_CONSTANTS.size;
  result[2] *= EE_GIZMOS_CONSTANTS.size;
  return result;
}
