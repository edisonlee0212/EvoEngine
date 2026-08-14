// @evoengine-dialect native
__exported import EvoEngine.PerFrame;
__exported import EvoEngine.Vertex;

[[vk::binding(0, 1)]]
public StructuredBuffer<Vertex, Std430DataLayout> EE_VERTICES;

[[vk::binding(1, 1)]]
public StructuredBuffer<uint, Std430DataLayout> EE_INDICES;

[[vk::binding(2, 1)]]
public RaytracingAccelerationStructure EE_TLAS;

public struct EmissiveTriangleInfo {
  public uint instance_index;
  public uint primitive_id;
  public float alias_probability;
  public uint alias_index;
  public float area_pdf;
};

[[vk::binding(3, 1)]]
public StructuredBuffer<EmissiveTriangleInfo, Std430DataLayout> EE_EMISSIVE_TRIANGLES;

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
