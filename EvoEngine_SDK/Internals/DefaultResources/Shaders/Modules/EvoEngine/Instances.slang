// @evoengine-dialect native
public struct Instance {
  public float4x4 model;
  public int material_index;
  public int triangle_offset;
  public int meshlet_offset;
  public int meshlet_size;

  public int info_index;
  public uint entity_index;
  public uint64_t renderer_handle;
  public int4 ray_tracing_geometry;
};

[[vk::binding(4, 0)]]
public StructuredBuffer<Instance, Std430DataLayout> EE_INSTANCES;

public float EE_TRANSFORM_HANDEDNESS(float4x4 transform) {
  return determinant(float3x3(transform)) < 0.0 ? -1.0 : 1.0;
}
