// @evoengine-dialect native

public static const float EE_DDGI_MIN_RAY_TRANSMISSION = 0.01f;

public struct DdgiProbeHitInfo {
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

public struct DdgiProbeRayPayload {
  public uint hit_count;
  public uint seed;
  public uint64_t handle;
  public DdgiProbeHitInfo hit_info;
  public float3 transmission;
  public float previous_hit_t;
  public uint transmission_is_inside;
};
