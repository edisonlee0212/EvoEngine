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
  public float4 world_bound_min;
  public float4 world_bound_max;
};

[[vk::binding(4, 0)]]
public StructuredBuffer<Instance, Std430DataLayout> EE_INSTANCES;

[[vk::binding(14, 0)]]
public StructuredBuffer<uint> EE_RASTER_DRAW_INSTANCE_INDICES;

public static const uint EE_RASTER_DRAW_INSTANCE_MAPPING_BIT = 0x80000000u;

public uint EE_RASTER_DRAW_INSTANCE_INDEX(uint draw_id, int instance_index, uint flags) {
  const uint index = uint(instance_index) + draw_id;
  return (flags & EE_RASTER_DRAW_INSTANCE_MAPPING_BIT) != 0u ? EE_RASTER_DRAW_INSTANCE_INDICES[index] : index;
}

public float EE_TRANSFORM_HANDEDNESS(float4x4 transform) {
  return determinant(float3x3(transform)) < 0.0 ? -1.0 : 1.0;
}
