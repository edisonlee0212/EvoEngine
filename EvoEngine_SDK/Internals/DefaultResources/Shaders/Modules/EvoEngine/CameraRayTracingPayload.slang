// @evoengine-dialect native
public struct CameraRayTracingPayload {
  public float3 shadow_transmission;
  public uint seed;

  public float2 barycentrics;
  public float hit_t;
  public uint instance_index;

  public float shadow_previous_hit_t;
  public uint primitive_id;
  public uint type;
  public uint shadow_is_inside;
  public uint camera_index;
};
