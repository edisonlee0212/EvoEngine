// @evoengine-dialect native
public struct HitInfo {
  public float3 position;
  public float vertex_info1;
  public float3 normal;
  public float vertex_info2;
  public float3 tangent;
  public float vertex_info3;
  public float4 color;
  public float2 tex_coord;
  public float2 vertex_info4;
};

public struct PointCloudRayTracingPayload {
  public uint hit_count;
  public uint seed;
  public uint64_t handle;
  public HitInfo hit_info;
};
