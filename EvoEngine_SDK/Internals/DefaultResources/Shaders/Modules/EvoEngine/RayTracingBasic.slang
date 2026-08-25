__exported import EvoEngine.PerFrame;
__exported import EvoEngine.Vertex;

[[vk::binding(0, 1)]]
public StructuredBuffer<Vertex, Std430DataLayout> EE_VERTICES;

[[vk::binding(1, 1)]]
public StructuredBuffer<uint, Std430DataLayout> EE_INDICES;

[[vk::binding(2, 1)]]
public RaytracingAccelerationStructure EE_TLAS;

public struct EmissiveInstanceInfo {
  public uint instance_index;
  public uint distribution_index;
  public float power_alias_probability;
  public uint power_alias_index;
  public float uniform_alias_probability;
  public uint uniform_alias_index;
  public float power_selection_probability;
  public float uniform_selection_probability;
};

public struct EmissiveTriangleDistributionInfo {
  public uint triangle_offset;
  public uint triangle_count;
};

public struct EmissiveTriangleDistributionEntry {
  public uint primitive_id;
  public float alias_probability;
  public uint alias_index;
  public float selection_probability;
};

[[vk::binding(3, 1)]]
public StructuredBuffer<EmissiveInstanceInfo, Std430DataLayout> EE_EMISSIVE_INSTANCES;

[[vk::binding(6, 1)]]
public StructuredBuffer<EmissiveTriangleDistributionInfo, Std430DataLayout> EE_EMISSIVE_DISTRIBUTIONS;

[[vk::binding(7, 1)]]
public StructuredBuffer<EmissiveTriangleDistributionEntry, Std430DataLayout> EE_EMISSIVE_TRIANGLES;

public struct EmissiveTriangleInfo {
  public uint instance_index;
  public uint primitive_id;
  public float power_selection_probability;
  public float uniform_selection_probability;
};

#if EE_RAY_TRACING_LINEAR_SWEPT_SPHERES_SUPPORTED
public struct RayTracingStrandPoint {
  public float3 position;
  public float thickness;
  public float3 normal;
  public float tex_coord;
  public float4 color;
};

[[vk::binding(4, 1)]]
public StructuredBuffer<RayTracingStrandPoint, Std430DataLayout> EE_RAY_TRACING_STRAND_POINTS;

[[vk::binding(5, 1)]]
public StructuredBuffer<uint, Std430DataLayout> EE_RAY_TRACING_STRAND_INDICES;
#endif
